// 向所选执行通道发送 FollowPath，按任务代次和请求序号过滤响应、反馈与结果，并处理旧请求的取消收口。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <memory>

namespace nav2_regulated_modules
{

// 按任务类别选择控制器与 GoalChecker，再异步发送完整 Path。
// FollowPath 的响应、反馈和结果均校验 generation／follow_sequence；固定路径绝对限速在接受后发布。
void RegulatedNavigator::sendFollowPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || task_.type == TaskType::NONE)
  {
    handlePlanningFailure("控制路径为空");
    return;
  }
  task_.active_path = path;
  task_.state = NavigationState::CONTROLLING;
  const auto generation = task_.generation;
  const auto sequence = ++follow_sequence_;
  const bool replacing_path = active_follow_goal_ != nullptr;
  const std::string controller_id = task_.type == TaskType::TO_POSE ? "AutonomousFixedPathController" : fixed_path_controller_id_;
  const std::string goal_checker_id = task_.type == TaskType::TO_POSE ? (frozen_segment_ + 1 == frozen_segments_.size() ? "autonomous_fixed_goal_checker" : "autonomous_fixed_cusp_checker") : fixed_path_goal_checker_id_;
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id;
  goal.goal_checker_id = goal_checker_id;
  if (replacing_path)
  {
    LOG_DEBUG("更新 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}", generation, sequence, path.poses.size(), controller_id);
  }
  else
  {
    LOG_INFO("下发 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}，goal_checker_id={}", generation, sequence, path.poses.size(), controller_id, goal_checker_id);
  }
  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  const auto client = followClient();
  const std::weak_ptr<rclcpp_action::Client<FollowPath>> weak_client = client;
  auto * outstanding = task_.type == TaskType::NAVIGATION_SERVICE ? &fixed_path_outstanding_follows_ :
    task_.type == TaskType::TO_POSE ? &frozen_outstanding_follows_ : nullptr;
  options.goal_response_callback = [this, generation, sequence, weak_client, outstanding](auto handle)
  {
    if (outstanding)
    {
      if (!handle)
      {
        --*outstanding;
      }
      if (!isCurrentFollow(generation, sequence))
      {
        if (handle)
        {
          if (const auto sender = weak_client.lock())
          {
            sender->async_cancel_goal(handle);
          }
        }
        return;
      }
    }
    if (isCurrentFollow(generation, sequence))
    {
      active_follow_goal_ = handle;
      if (!handle)
      {
        startRecovery("控制 Goal 被拒绝");
        return;
      }
      const auto speed_pub = speedPublisher();
      if ((task_.type == TaskType::NAVIGATION_SERVICE || task_.type == TaskType::TO_POSE) && speed_pub)
      {
        nav2_msgs::msg::SpeedLimit speed_limit;
        speed_limit.percentage = false;
        speed_limit.speed_limit = task_.requested_speed;
        speed_pub->publish(speed_limit);
      }
    }
  };
  options.feedback_callback = [this, generation, sequence](auto, const std::shared_ptr<const FollowPath::Feedback> controller_feedback)
  {
    if (isCurrentFollow(generation, sequence))
    {
      if (task_.type != TaskType::TO_POSE)
      {
        task_.distance_remaining = static_cast<double>(controller_feedback->distance_to_goal);
      }
      current_speed_ = controller_feedback->speed;
      if (task_.total_path_length > 0.0)
      {
        const double completed_ratio = (task_.total_path_length - task_.distance_remaining) / task_.total_path_length;
        task_.progress = std::max(task_.progress, static_cast<float>(std::clamp(completed_ratio, 0.0, 1.0)));
      }
      if (active_navigation_service_goal_)
      {
        auto feedback = std::make_shared<NavigationService::Feedback>();
        feedback->cur_task_id = task_.task_id;
        feedback->cur_seg_id = "";
        feedback->progress = task_.progress;
        active_navigation_service_goal_->publish_feedback(feedback);
      }
    }
  };
  // 下游结果只影响匹配当前 generation／sequence 的任务；迟到结果不能结束新任务。
  options.result_callback = [this, generation, sequence, outstanding](const auto & result)
  {
    if (outstanding)
    {
      --*outstanding;
    }
    if (!isCurrentFollow(generation, sequence))
    {
      return;
    }
    active_follow_goal_.reset();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      if (task_.type == TaskType::TO_POSE)
      {
        finishFrozenSegment();
      }
      else
      {
        succeedTask();
      }
    }
    else if (task_.state != NavigationState::CANCELING)
    {
      startRecovery("控制器执行路径失败");
    }
  };
  if (outstanding)
  {
    ++*outstanding;
  }
  try
  {
    client->async_send_goal(goal, options);
  }
  catch (const std::exception & error)
  {
    if (outstanding)
    {
      --*outstanding;
    }
    failTask(error.what());
  }
}

// 比较任务代次与跟踪轮次；同一任务换路径也会使旧 sequence 失效。
// 同一任务可能经历分段或原路径恢复，必须同时核对 follow_sequence，防止旧跟踪结果干扰当前段。
bool RegulatedNavigator::isCurrentFollow(const uint64_t generation, const uint64_t sequence) const
{
  return generation == task_.generation && sequence == follow_sequence_;
}

// 向速度链入口连续发布三帧零 Twist，增加停车请求被下游收到的机会。
// 不检查发布确认或实际轮速；若没有发布器直接返回，不承诺已经机械停稳。
void RegulatedNavigator::stopRobot()
{
  const auto stop_pub = task_.type == TaskType::TO_POSE ?
    lattice_stop_pub_ : stop_cmd_pub_;
  if (!stop_pub)
  {
    return;
  }
  geometry_msgs::msg::Twist stop;
  stop_pub->publish(stop);
  stop_pub->publish(stop);
  stop_pub->publish(stop);
  LOG_DEBUG("已向速度链入口连续发布 3 帧零速度");
}

// 在 velocity_mutex 保护下缓存控制器最新 Twist 与曾接收标记。
// 缓存用于日志，不代表与平滑器反馈来自严格同一控制周期。
void RegulatedNavigator::onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_controller_velocity_ = *velocity;
  has_controller_velocity_ = true;
}

// 记录平滑后 Twist 供诊断；不再对速度限幅或二次发布。
// 旧快照可持续存在，消费者应另行理解采样新鲜度。
void RegulatedNavigator::onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_smoothed_velocity_ = *velocity;
  has_smoothed_velocity_ = true;
}

// 保留源 Header 和实测速度，供速度观测及 autonomous_fixed 停稳判断。
void RegulatedNavigator::onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  latest_velocity_odometry_ = *odometry;
  has_velocity_odometry_ = true;
}

void RegulatedNavigator::onVelocityMotionState(const byd_custom_msgs::msg::MotionState::SharedPtr motion)
{
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->header = motion->header;
  odometry->twist.twist.linear.x = motion->v_car;
  odometry->twist.twist.angular.z = motion->w_car;
  onVelocityOdometry(odometry);
}

// 先在锁内复制三路快照，再在锁外写日志，减少回调持锁时间。
// 没有任何样本时不打印；独立订阅的 latest 值不能当作同步传感器帧。
void RegulatedNavigator::logVelocityChain()
{
  if (!active_)
  {
    return;
  }
  geometry_msgs::msg::Twist controller_velocity;
  geometry_msgs::msg::Twist smoothed_velocity;
  nav_msgs::msg::Odometry velocity_odometry;
  bool has_controller_velocity;
  bool has_smoothed_velocity;
  bool has_velocity_odometry;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    controller_velocity = latest_controller_velocity_;
    smoothed_velocity = latest_smoothed_velocity_;
    velocity_odometry = latest_velocity_odometry_;
    has_controller_velocity = has_controller_velocity_;
    has_smoothed_velocity = has_smoothed_velocity_;
    has_velocity_odometry = has_velocity_odometry_;
  }
  if (!has_controller_velocity && !has_smoothed_velocity && !has_velocity_odometry)
  {
    return;
  }
  LOG_DEBUG("关键速度链：反馈 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}] -> 控制器输出/平滑器输入 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}] -> 平滑器输出 {} [vx={:.3f} m/s, wz={:.3f} rad/s, received={}]", velocity_odom_topic_, velocity_odometry.twist.twist.linear.x, velocity_odometry.twist.twist.angular.z, has_velocity_odometry, controller_cmd_vel_topic_, controller_velocity.linear.x, controller_velocity.angular.z, has_controller_velocity, smoothed_cmd_vel_topic_, smoothed_velocity.linear.x, smoothed_velocity.angular.z, has_smoothed_velocity);
}

}
// namespace nav2_regulated_modules
