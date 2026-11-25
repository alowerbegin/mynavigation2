// 接收半自主单点 Action，完成准入和位姿校验后排队交接，处理本次句柄的取消并发布单点反馈。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "nav2_regulated_modules/navigation_utils.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 仅接收通过统一准入且位姿有效的单点 Goal，behavior_tree 必须为空。
// 接收回调只做准入校验，规划在 Goal 排队并完成控制权交接后触发。
rclcpp_action::GoalResponse RegulatedNavigator::handlePoseGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigateToPose::Goal> goal)
{
  if ((!admitUnified(true)) || !active_ || !navigation_utils::validPose(goal->pose) || !goal->behavior_tree.empty())
  {
    LOG_WARN("拒绝单点目标：节点未激活、位姿无效或请求了行为树 XML");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

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

// 接受后的 Goal 先进入待执行队列并请求取消旧任务，交接完成才递增代次并启动单次规划。
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
