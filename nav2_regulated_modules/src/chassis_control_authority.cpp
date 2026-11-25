// 仲裁双导航通道和遥控的最终底盘输出。交接检查旧子任务收口及反馈停稳，导航缓存按通道与跟踪代次隔离。
#include "nav2_regulated_modules/chassis_control_subscriber.hpp"

#include <cmath>

#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

// 以单调接收时间判断底盘反馈的新鲜度，避免 ROS 时间变化干扰控制权接管条件。
bool ChassisControlSubscriber::motionFresh(const MotionStateSnapshot & state) const
{
  return state.valid && std::chrono::duration<double>(std::chrono::steady_clock::now() - state.receive_time).count() <= config_.motion_state_timeout;
}

// 在反馈新鲜的前提下同时要求线速度与角速度低于停稳阈值。
bool ChassisControlSubscriber::motionStopped(const MotionStateSnapshot & state) const
{
  return motionFresh(state) && std::abs(state.linear_velocity) < 1e-3 && std::abs(state.angular_velocity) < 1e-3;
}

// 检查统一出口是否唯一、遥控是否释放及交接是否完成，已有导航允许合法的新路径请求抢占。
bool ChassisControlSubscriber::canStartNavigation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.unified_output)
  {
    return true;
  }
  if (!active_ || (authority_ != Authority::IDLE && authority_ != Authority::NAVIGATION) || node_.count_publishers(config_.output_topic) != 1U)
  {
    return false;
  }
  // 同一导航控制权内保留原路径抢占语义；遥控释放后的首个任务必须确认已停稳。
  return config_.navigation_active() || (config_.navigation_drained() && motionStopped(motion_state_subscriber_.latestState()));
}

// 检查两条导航速度输入的零速与静默时间，避免旧通道的迟到非零命令进入下一任务。
bool ChassisControlSubscriber::navigationOutputStopped(
  const std::chrono::steady_clock::time_point since)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto quiet = std::chrono::milliseconds(200);
  const auto now = std::chrono::steady_clock::now();
  for (const auto & sample : channel_velocity_)
  {
    if (sample.last_nonzero + quiet > now ||
      (!sample.stopped && sample.received != std::chrono::steady_clock::time_point{}))
    {
      return false;
    }
  }
  return since + quiet <= now;
}

// 按通道记录接收状态，将当前选中通道的有限速度与接收时的跟踪代次一起缓存。
void ChassisControlSubscriber::onNavigationVelocity(
  const int source, const geometry_msgs::msg::Twist & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & sample = channel_velocity_[source];
  const auto received = std::chrono::steady_clock::now();
  sample.received = received;
  const bool finite = std::isfinite(message.linear.x) && std::isfinite(message.angular.z);
  sample.stopped = finite && std::abs(message.linear.x) < 1e-3 &&
    std::abs(message.angular.z) < 1e-3;
  if (!sample.stopped)
  {
    sample.last_nonzero = received;
  }
  const int selected = config_.navigation_source ? config_.navigation_source() : 0;
  if (finite && active_ && selected == source && config_.navigation_active() &&
    (authority_ == Authority::IDLE || authority_ == Authority::NAVIGATION))
  {
    navigation_velocity_ = message;
    navigation_receive_time_ = received;
    cached_navigation_source_ = source;
    cached_navigation_generation_ = config_.navigation_generation ?
      config_.navigation_generation() : 0;
    has_navigation_velocity_ = true;
    authority_ = Authority::NAVIGATION;
  }
}

// 每周期按发布者冲突、反馈质量、交接、遥控与导航的顺序仲裁最终输出，遥控超时沿既定速度规划减速。
void ChassisControlSubscriber::processUnifiedCommand()
{
  const auto now = std::chrono::steady_clock::now();
  const auto state = motion_state_subscriber_.latestState();
  if (node_.count_publishers(config_.output_topic) > 1U)
  {
    if (!publisher_conflict_)
    {
      LOG_ERROR("统一出口发现额外 {} 发布者，取消导航并保持零速", config_.output_topic);
      config_.request_takeover();
    }
    publisher_conflict_ = true;
    clearControlStateLocked();
    authority_ = Authority::HANDOVER;
    handover_since_ = now;
    has_navigation_velocity_ = false;
    publishZero();
    return;
  }
  publisher_conflict_ = false;
  if (!motionFresh(state))
  {
    if (authority_ != Authority::HANDOVER)
    {
      config_.request_takeover();
      authority_ = Authority::HANDOVER;
      handover_since_ = now;
    }
    clearControlStateLocked();
    has_navigation_velocity_ = false;
    publishZero();
    return;
  }
  const bool command_fresh = has_command_ && std::chrono::duration<double>(now - last_command_tmie_).count() <= config_.command_timeout;
  if (authority_ == Authority::HANDOVER)
  {
    publishZero();
    if (!config_.navigation_drained() || state.receive_time <= handover_since_ || !motionStopped(state))
    {
      return;
    }
    if (!command_fresh)
    {
      clearControlStateLocked();
      authority_ = Authority::IDLE;
      LOG_INFO("控制权交接已停稳，等待新任务或新遥控指令");
      return;
    }
    linear_planner_.init(state.linear_velocity);
    angular_planner_.init(state.angular_velocity);
    authority_ = Authority::REMOTE;
    LOG_INFO("FollowPath 已结束且底盘停稳，后台遥控取得控制权");
    return;
  }
  if (authority_ == Authority::REMOTE || authority_ == Authority::STOPPING)
  {
    if (!command_fresh)
    {
      has_command_ = false;
      target_command_.linear_velocity = 0.0;
      target_command_.angular_velocity = 0.0;
      authority_ = Authority::STOPPING;
    }
    linear_planner_.setDirection(target_command_.linear_velocity, target_command_.linear_acceleration);
    angular_planner_.setDirection(target_command_.angular_velocity, target_command_.angular_acceleration);
    const double linear = linear_planner_.update();
    const double angular = angular_planner_.update();
    publishControl(linear, angular);
    if (authority_ == Authority::STOPPING && std::abs(linear) < 1e-3 && std::abs(angular) < 1e-3 && motionStopped(state))
    {
      publishZero();
      clearControlStateLocked();
      authority_ = Authority::IDLE;
      LOG_INFO("后台遥控超时减速完成，已停稳并释放控制权；旧路径不恢复");
    }
    return;
  }
  const int selected = config_.navigation_source ? config_.navigation_source() : 0;
  const auto generation = config_.navigation_generation ? config_.navigation_generation() : 0;
  if (config_.navigation_active() && has_navigation_velocity_ && selected >= 0 &&
    selected == cached_navigation_source_ && generation == cached_navigation_generation_)
  {
    if (std::chrono::duration<double>(now - navigation_receive_time_).count() <= config_.motion_state_timeout)
    {
      publishOutput(navigation_velocity_.linear.x, navigation_velocity_.angular.z);
      return;
    }
    config_.request_takeover();
    authority_ = Authority::HANDOVER;
    handover_since_ = now;
    LOG_ERROR("导航最终速度输入超时，取消任务并保持零速");
  }
  has_navigation_velocity_ = false;
  if (authority_ == Authority::NAVIGATION)
  {
    authority_ = Authority::IDLE;
  }
  publishZero();
}

}  // namespace nav2_regulated_modules
