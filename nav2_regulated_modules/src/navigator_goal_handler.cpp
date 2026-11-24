// 标准自主导航 Action 与 goal_pose 的接入、取消标记和反馈。
// 目标校验不执行规划；接受回调才接管任务。当前模式不匹配会拒绝／忽略入口，不会自动切换运行模式。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "nav2_regulated_modules/navigation_utils.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 仅接受 autonomous 且激活的有效单点 Pose；behavior_tree 非空会拒绝。
// 这里只校验请求，规划发生在接受后的处理函数，不通过此函数切换启动模式。
rclcpp_action::GoalResponse RegulatedNavigator::handlePoseGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateToPose::Goal> goal)
{
  if ((!admitUnified(true)) || !active_ || !navigation_utils::validPose(goal->pose) || !goal->behavior_tree.empty())
  {
    LOG_WARN("拒绝单点目标：节点未激活、位姿无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 逐点检查非空目标序列的有效性，并拒绝行为树 XML 请求。
// 多点任务的目标数量不等于已经计算出的路径点数量。
rclcpp_action::CancelResponse RegulatedNavigator::handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal)
{
  if (goal == pending_pose_)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  if (goal == active_pose_goal_)
  {
    cancel_requested_ = true;
    LOG_DEBUG("收到单点导航取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 仅为当前多点任务设置取消标记，避免旧客户端取消新任务。
// 该回调本身不等待规划、平滑或控制子 Goal 结束。
// 先抢占现有任务，再递增 generation，保存目标并启动规划。
// 旧子 Goal 以异步方式取消，旧回调靠代次过滤，不把句柄重置视为取消握手完成。
void RegulatedNavigator::handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal)
{
    if (!admitUnified(true))
    {
      goal->abort(std::make_shared<NavigateToPose::Result>());
      return;
    }
    // Check transform availability before interrupting a valid previous task.
    if (goal->get_goal()->pose.header.frame_id != global_frame_)
    {
      try
      {
        tf_buffer_->transform(goal->get_goal()->pose, global_frame_, tf2::durationFromSec(0.0));
      }
      catch (const tf2::TransformException & error)
      {
        LOG_WARN("半自主目标无法转换至地图坐标系：{}", error.what());
        goal->abort(std::make_shared<NavigateToPose::Result>());
        return;
      }
    }
    pending_pose_ = goal;
    pending_since_ = std::chrono::steady_clock::now();
    cancelTask("被新的半自主 Action 抢占");
}

void RegulatedNavigator::startPose(const std::shared_ptr<NavigatePoseHandle> & goal)
{
  active_pose_goal_ = goal;
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::TO_POSE;
  task_.goal = goal->get_goal()->pose;
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  LOG_INFO("接受单点导航任务，generation={}，frame={}，goal=({:.3f}, {:.3f})", task_.generation, task_.goal.header.frame_id, task_.goal.pose.position.x, task_.goal.pose.position.y);
  beginFixedAutonomy();
}

void RegulatedNavigator::publishFeedback()
{
  if (!active_ || task_.type == TaskType::NONE)
  {
    return;
  }
  geometry_msgs::msg::PoseStamped current_pose;
  if (!lookupCurrentPose(current_pose))
  {
    return;
  }
  const double navigation_time = (now() - task_.start_time).seconds();
  double remaining = task_.distance_remaining;
  double eta = current_speed_ > 0.03 ? task_.distance_remaining / current_speed_ : 0.0;
  int16_t recoveries = static_cast<int16_t>(task_.recovery_count);
  if (task_.type == TaskType::TO_POSE)
  {
    auto goal = task_.goal;
    goal.header.stamp = current_pose.header.stamp;
    if (goal.header.frame_id != current_pose.header.frame_id)
    {
      try
      {
        goal = tf_buffer_->transform(goal, current_pose.header.frame_id, tf2::durationFromSec(0.0));
      }
      catch (const tf2::TransformException & error)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Autonomous feedback unavailable: %s", error.what());
        return;
      }
    }
    const double goal_distance = navigation_utils::poseDistance(current_pose, goal);
    const double yaw_error = navigation_utils::normalizeAngle(navigation_utils::yawFromPose(goal) - navigation_utils::yawFromPose(current_pose));
    if (!std::isfinite(goal_distance) || !std::isfinite(yaw_error))
    {
      return;
    }
    remaining = detail::autonomousFeedbackDistance(task_.distance_remaining, goal_distance);
    eta = detail::autonomousFeedbackEta(remaining, yaw_error, current_speed_);
    recoveries = detail::autonomousFeedbackRecoveries(task_.recovery_count, task_.localization_recovery_count);
  }

  if (active_pose_goal_)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = current_pose;
    feedback->navigation_time = navigation_utils::durationFromSeconds(navigation_time);
    feedback->estimated_time_remaining = navigation_utils::durationFromSeconds(eta);
    feedback->number_of_recoveries = recoveries;
    feedback->distance_remaining = remaining;
    active_pose_goal_->publish_feedback(feedback);
  }
}

}
// namespace nav2_regulated_modules
