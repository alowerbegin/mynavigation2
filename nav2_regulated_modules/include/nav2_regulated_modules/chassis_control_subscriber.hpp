// 定义底盘遥控、双导航速度输入和统一输出的仲裁接口。控制权、接收时间与任务代次共同决定每周期可发送的指令。

#ifndef NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_
#define NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "byd_custom_msgs/msg/chassis_control.hpp"
#include "byd_custom_msgs/msg/control_res.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_regulated_modules/motion_state_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

// 对单轴速度按加速度与加加速度约束渐变；线速度和角速度各使用一个实例。
class SCurvePlanner {
public:
    // dt 为积分周期秒数，sm 为速度幅值，后续参数给出正常加／减速度和 jerk 上限。
    // 构造时保存约束并清零运动状态，实际每周期约束由 update 按运动阶段选择。
    SCurvePlanner(double dt, double sm, 
                  double norm_accel_amax, double norm_accel_jmax, 
                  double norm_decel_amax, double norm_decel_jmax)
                //   double emrg_amax, double emrg_jmax)
        : dt_(dt), sm_(sm), 
          am_(norm_accel_amax), jm_(norm_accel_jmax),
          norm_accel_amax_(norm_accel_amax), norm_accel_jmax_(norm_accel_jmax), 
          norm_decel_amax_(norm_decel_amax), norm_decel_jmax_(norm_decel_jmax), 
          emrg_amax_(8.0), emrg_jmax_(8.0),
          sc_(0.0), ac_(0.0), jc_(0.0), d_(0.0) {}

    // 清空控制或检查器历史状态，避免跨任务沿用上次进度。
    // 将速度 sc_、加速度 ac_、jerk jc_ 和方向比例 d_ 一并归零；只影响数值规划器，不发 ROS 消息。
    void reset(){
        sc_ = 0.0;
        ac_ = 0.0;
        jc_ = 0.0;
        d_  = 0.0;
    }

    // 用实测速度重置 sc_ 并清零 ac_，其他历史量按原状态保留。
    void init(double sc){
        sc_ = sc;
        ac_ = 0.0;
    }

    // sm 提供目标方向和速度幅值，反向请求先将 d_ 置零以减速。
    // am 与 emrgstate 更新候选约束，每周期再按实际加速或减速阶段选取上限。
    void setDirection(double sm, double am, bool emrgstate = false) {
        if(std::abs(sm) < 1e-3){
            d_ = 0.0;
        } else {
            sm_ = std::abs(sm);
            am_ = std::abs(am);
            d_ = std::max(-1.0, std::min(1.0, sm/sm_));
        }
        if (sc_ * d_ < 0.0 && std::abs(sc_) > 1e-6) {
            d_ = 0.0; 
        }
        if(prestate_emrg == true && emrgstate == false){
            am_ = norm_accel_amax_;   
            jm_ = norm_accel_jmax_;   
            prestate_emrg = false;
        }
        if(prestate_emrg == false && emrgstate == true){
            am_ = emrg_amax_;  
            jm_ = emrg_jmax_;   
            prestate_emrg = true;
        }
    }

    // 每次推进 dt_ 秒，选择 jerk 后积分并限制加速度，再用梯形积分更新速度。
    // 周期起始达到目标时清零导数，积分越界时先钳制速度，下一周期再清理导数。
    double update() {
        // vg 是带方向目标速度，sc_ 是当前规划速度；两者差值不等于实测跟踪误差。
        double vg = d_ * sm_;

        if (std::abs(sc_ - vg) <= 1e-6 ) {
            sc_ = vg;
            ac_ = 0.0;
            jc_ = 0.0;
            return sc_;
        }
        
        if (std::abs(ac_) < 1e-6) {
            bool is_speeding_up = (sc_ * vg >= 0.0) && (std::abs(sc_) < std::abs(vg));
            
            if (is_speeding_up) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        } else {
            if (sc_ * ac_ >= 0.0) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        }

        double vel_err = vg - sc_;
        double dr = 0;
        if( vel_err < -1e-6 ) dr = -1.0;
        if( vel_err >  1e-6 ) dr =  1.0;

        
        // 加速度已沿误差方向作用时，估算撤去加速度所需的速度余量 ac_²÷(2×jm_)，决定何时翻转 jerk。
        if ( dr*ac_>=0.0 && std::abs(vel_err)-0.5*std::abs(ac_*ac_/jm_) <= 1e-4) {
            if( std::abs(dr) > 1e-6){
              jc_ =  -dr*jm_;
            } else {
              if (ac_ > 0.0) jc_ = -jm_;  
              if (ac_ < 0.0) jc_ =  jm_;  
            }
        } else {
            if( std::abs(dr) > 1e-6){
              jc_ =  dr*jm_;
            }
            else{
              if (ac_ > 0.0) jc_ =  jm_;
              if (ac_ < 0.0) jc_ = -jm_;
            }
            
        }

        double tmp = ac_;
        ac_ += jc_ * dt_;
        
        ac_ = std::max(-am_, std::min(am_, ac_));
        if( dr*ac_>=0.0 && dr*jc_<=0.0 )
        {
          if( jc_<0.0 ) {
            ac_ = std::max(ac_, 0.0);
          }
          if( jc_>0.0 ) {
            ac_ = std::min(ac_, 0.0);
          }
        }

        sc_ += 0.5*(tmp + ac_)*dt_ ;

        if (dr >= 1e-6) {
            sc_ = std::min(sc_, vg);
        } else if (dr <= -1e-6) {
            sc_ = std::max(sc_, vg);
        }
        else{
          sc_ = vg;
        }
        sc_ = std::max(-sm_, std::min(sm_, sc_));
        return sc_;
    }

    // 读取本轮 S 曲线输出速度。
    // 只读取规划器内部速度；并非 MotionState 的实测速度。
    double getVelocity() const { return sc_; }
    // 读取本轮 S 曲线输出加速度。
    // 返回内部积分得到的加速度，线／角轴单位分别为 m/s²／rad/s²。
    double getAcceleration() const { return ac_; }
    // 读取本轮 S 曲线输出加加速度。
    // 返回本轮使用的 jerk，线／角轴单位分别为 m/s³／rad/s³。
    double getJerk() const { return jc_; }

private:
    
    bool prestate_emrg = false;
    double dt_;
    double sm_;
    double am_;
    double jm_;

    double norm_accel_amax_;
    double norm_accel_jmax_;
    double norm_decel_amax_;
    double norm_decel_jmax_;
    double emrg_amax_;
    double emrg_jmax_;

    double sc_;
    double ac_;
    double jc_;
    double d_;
};

// 底盘指令入口、输出与安全限幅；速度和加速度参数分别对应线运动及旋转。
struct ChassisControlConfig
{
  // 仅 fixed_path 启用；所有回调由导航器默认 MutuallyExclusive 回调组串行执行。
  bool unified_output{false};
  std::string navigation_topic{"/cmd_vel"};
  std::string lattice_navigation_topic;
  std::function<int()> navigation_source;
  std::function<uint64_t()> navigation_generation;
  std::function<void()> request_takeover;
  std::function<bool()> navigation_active;
  std::function<bool()> navigation_drained;
  std::string input_topic{"/downstream/chassis_control"};
  std::string output_topic{"/control_to_uart"};
  // 反馈最大允许接收年龄 s；与消息 Header 时间不是同一计时依据。
  double motion_state_timeout{0.2};
  // 有效远程指令接收间隔上限 s；超时将目标置零，不凭旧命令继续行驶。
  double command_timeout{0.15};
  double publish_rate{100.0};
  double default_linear_speed_max{1.5};
  double linear_speed_max = 0.3;
  double default_angular_speed_max{0.5};
  double angular_speed_max = 0.3;
  double default_linear_accel_max{2.0};
  double linear_accel_max{3.0};
  double linear_decel_max{4.0};
  double default_angular_accel_max{1.5};
  double angular_accel_max{2.0};
  double angular_decel_max{3.0};
  double linear_accel_jerk_max{6.0};
  double linear_decel_jerk_max{8.0};
  double angular_accel_jerk_max{4.0};
  double angular_decel_jerk_max{6.0};
};

class ChassisControlSubscriber
{
  public:
  // 保存配置并建立 ChassisControl 订阅和 ControlRes 发布器；构造不启动输出定时器。
  // 两个 SCurvePlanner 各自管理线／角速度，参数单位对应 m 和 rad。
  ChassisControlSubscriber(nav2_util::LifecycleNode & node, MotionStateSubscriber & motion_state_subscriber, ChassisControlConfig config);
  // 先清理 MotionState 和旧命令，再按 publish_rate 创建周期处理定时器。
  // 重置后需要新反馈和新有效命令才能进入远程输出。
  void activate();
  // 停用并取消定时器；统一出口发送零速，不等同于机械停稳。
  void deactivate();
  // 使支路 inactive、清理目标与反馈快照；本函数不取消定时器也不发布零速。
  void reset();
  // 检查统一出口是否唯一、遥控是否释放及交接是否完成，已有导航允许合法的新路径请求抢占。
  bool canStartNavigation();
  // 检查两条导航速度输入的零速与静默时间，避免旧通道的迟到非零命令进入下一任务。
  bool navigationOutputStopped(std::chrono::steady_clock::time_point since);

private:
  enum class Authority
  {
    IDLE,
    NAVIGATION,
    HANDOVER,
    REMOTE,
    STOPPING
  };
  // 按通道记录接收状态，将当前选中通道的有限速度与接收时的跟踪代次一起缓存。
  void onNavigationVelocity(int source, const geometry_msgs::msg::Twist & message);
  struct ChannelVelocity
  {
    std::chrono::steady_clock::time_point received{};
    std::chrono::steady_clock::time_point last_nonzero{};
    bool stopped = false;
  };
  ChannelVelocity channel_velocity_[2];
  int cached_navigation_source_ = -1;
  uint64_t cached_navigation_generation_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr lattice_navigation_subscription_;
  // 每周期按发布者冲突、反馈质量、交接、遥控与导航的顺序仲裁最终输出，遥控超时沿既定速度规划减速。
  void processUnifiedCommand();
  // 以单调接收时间判断底盘反馈的新鲜度，避免 ROS 时间变化干扰控制权接管条件。
  bool motionFresh(const MotionStateSnapshot & state) const;
  // 在反馈新鲜的前提下同时要求线速度与角速度低于停稳阈值。
  bool motionStopped(const MotionStateSnapshot & state) const;
  void publishOutput(double linear_velocity, double angular_velocity);
  Authority authority_{Authority::IDLE};
  geometry_msgs::msg::Twist navigation_velocity_;
  std::chrono::steady_clock::time_point navigation_receive_time_{};
  std::chrono::steady_clock::time_point handover_since_{};
  bool has_navigation_velocity_{false};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_subscription_;
  // 缓存最近一次已解析指令；超时和运动状态检查先于周期输出。
  struct TargetCommand
  {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    double linear_acceleration{1.0};
    double angular_acceleration{1.0};
    uint8_t operation{0};
  };

  // 校验非负幅值、有限数与 op，将方向转换为速度符号后限幅并存入目标。
  // 非法请求只被拒绝，不刷新 last_command_tmie_；有效请求才延长指令有效期。
  void onChassisControl(const byd_custom_msgs::msg::ChassisControl::ConstSharedPtr message);
  // 在 mutex_ 下检查激活、发布者冲突与命令超时，再推进两轴 S 曲线并发布。
  // 开始远程阶段及退出阶段会检查 MotionState；不能误写为所有中间周期都检查反馈超时。
  void processControlCommand();
  // 只清除目标、has_command 与远程阶段标志，调用方必须已持 mutex_。
  // 不重新初始化两个规划器的内部速度，也不自动发布命令。
  void clearControlStateLocked();
  // 将线／角输出再次按远程上限钳位，升降与货架旋转置零后发布 ControlRes。
  // 输出是 ROS 控制请求，不包含 UART 写入确认或实际轮速。
  void publishControl(double linear_velocity, double angular_velocity);
  // 复用 publishControl 发出一帧全零底盘请求；不等待该命令执行。
  void publishZero();
  // 检查话题非空、时间／频率／规划约束和线角限速范围。
  // 本函数只返回配置有效性，不检测发布者、定位或反馈是否就绪。
  bool validateConfig() const;

  nav2_util::LifecycleNode & node_;
  MotionStateSubscriber & motion_state_subscriber_;
  ChassisControlConfig config_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  TargetCommand target_command_;
  std::chrono::steady_clock::time_point last_command_tmie_;
  bool active_{false};
  bool has_command_{false};
  bool is_remote_control_{false};
  bool publisher_conflict_{false};

  SCurvePlanner linear_planner_;
  SCurvePlanner angular_planner_;

  rclcpp::Subscription<byd_custom_msgs::msg::ChassisControl>::SharedPtr subscription_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_
