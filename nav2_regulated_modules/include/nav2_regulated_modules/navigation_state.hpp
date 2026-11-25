// 定义常驻双 Action 协调器的任务类型、执行阶段和共享快照，generation、规划序号与跟踪序号共同过滤旧回调。

#ifndef NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
#define NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace nav2_regulated_modules
{

// 遥控与两类导航入口同时可用，实际运动权限由底盘仲裁器逐周期决定。

// 单个任务的执行阶段；异步回调必须结合 generation 判断状态是否仍属于当前任务。
enum class NavigationState
{
  IDLE,
  PLANNING,
  SMOOTHING,
  CONTROLLING,
  REPLANNING,
  CLEARING_COSTMAP,
  LOCALIZATION_LOST,
  CANCELING,
  SUCCEEDED,
  FAILED
};

// 区分空闲、半自主单点 Action 与业务固定路径 Action，结果返回到各自的上层句柄。
enum class TaskType
{
  NONE,
  TO_POSE,
  NAVIGATION_SERVICE
};

// 当前任务的共享快照；代次、路径和计时字段用于丢弃过时回调并监控进度。
struct NavigationTask
{
  // 每次任务接管递增，异步回调必须匹配本任务代次和对应的规划／跟踪序号。
  uint64_t generation { 0 };
  // TO_POSE 对应半自主 Lattice，NAVIGATION_SERVICE 对应业务分段路径。
  TaskType type { TaskType::NONE };
  // 状态字段描述当前任务阶段，未使用的历史枚举值仍保留在类型中。
  NavigationState state { NavigationState::IDLE };
  geometry_msgs::msg::PoseStamped goal;
  // 历史多目标缓存字段保留，当前上层入口接收单点 Action 或业务分段路径。
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  nav_msgs::msg::Path active_path;
  // 以下时间用于任务耗时、单次规划等待和路径执行无进展超时判断。
  // ROS 时间用于导航耗时；暂停仿真／时钟回跳会影响此类计时，与 steady_clock 的接收超时不同。
  rclcpp::Time start_time { 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_replan_time { 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_progress_time { 0, 0, RCL_ROS_TIME };
  geometry_msgs::msg::PoseStamped last_progress_pose;
  // 记录恢复与规划失败的历史计数，当前单次规划失败直接结束半自主任务。
  int recovery_count { 0 };
  int localization_recovery_count = 0;
  int consecutive_planning_failures { 0 };
  double distance_remaining { 0.0 };
  std::string task_id;
  double total_path_length { 0.0 };
  // 固定路径请求速度与前后向标记不直接改变普通导航控制器参数。
  // 正的任务速度基准 m/s；运动方向由 motion_direction 与路径姿态另行编码。
  double requested_speed { 0.0 };
  uint8_t motion_direction { 0 };
  float progress { 0.0F };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__NAVIGATION_STATE_HPP_
