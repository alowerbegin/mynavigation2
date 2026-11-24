// 自主任务的规划、可选平滑与跟踪衔接，使用异步 Action 回调避免把旧结果写入新任务。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "nav2_regulated_modules/navigation_utils.hpp"

namespace nav2_regulated_modules
{

// 按 server_timeout 等待规划、可选平滑及 FollowPath 服务发现。
// 服务可发现只证明通信入口存在，不证明地图、TF 或底盘反馈有效。
bool RegulatedNavigator::dependenciesReady()
{
  return executionReady(true);
}

// generation 标识任务，plan_sequence 标识该任务的规划轮次；两者均匹配才接受回调。
void RegulatedNavigator::startPlanning()
{
  if (task_.type != TaskType::TO_POSE)
  {
    LOG_ERROR("仅半自主单点任务可请求 Lattice 规划");
    return;
  }
  if (!active_ || planning_active_)
  {
    return;
  }
  if (frozen_plan_requested_)
  {
    failTask("autonomous_fixed 禁止再次调用全局规划");
    return;
  }
  frozen_plan_requested_ = true;
  if (!dependenciesReady())
  {
    failTask("Lattice 规划或控制 Action Server 不可用");
    return;
  }

  planning_active_ = true;
  task_.state = NavigationState::PLANNING;
  task_.last_replan_time = now();
  const auto generation = task_.generation;
  const auto sequence = ++plan_sequence_;
  ComputePathToPose::Goal goal;
  goal.goal = task_.goal;
  goal.planner_id = planning_module_.plannerId();
  goal.use_start = false;
  auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  // 接受响应先验证任务及规划轮次；本轮被拒绝才计入失败，旧响应不能覆盖当前句柄。
  options.goal_response_callback = [this, generation, sequence](auto handle)
  {
    if (!handle)
    {
      --outstanding_plans_;
    }
    if (!isCurrentPlan(generation, sequence) && handle)
    {
      compute_pose_client_->async_cancel_goal(handle);
      return;
    }
    if (isCurrentPlan(generation, sequence))
    {
      active_compute_pose_goal_ = handle;
      if (!handle)
      {
        planning_active_ = false;
        handlePlanningFailure("单点规划 Goal 被拒绝");
      }
    }
  };
  // 规划结果先验证归属，再检查结果码、对象与非空 Path；通过后才进入可选平滑／跟踪。
  options.result_callback = [this, generation, sequence](const auto & result)
  {
    --outstanding_plans_;
    if (!isCurrentPlan(generation, sequence))
    {
      return;
    }
    planning_active_ = false;
    active_compute_pose_goal_.reset();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || result.result->path.poses.empty())
    {
      handlePlanningFailure("单点规划失败或返回空路径");
      return;
    }
    onPathReady(result.result->path);
  };
  ++outstanding_plans_;
  try
  {
    compute_pose_client_->async_send_goal(goal, options);
  }
  catch (const std::exception & error)
  {
    --outstanding_plans_;
    failTask(error.what());
  }
}

// 同时比较 generation 和规划 sequence，拒绝旧任务或旧规划轮次的异步结果。
// 它不检查机器人位置或路径质量，只承担回调归属判断。
bool RegulatedNavigator::isCurrentPlan(const uint64_t generation, const uint64_t sequence) const
{
  return generation == task_.generation && sequence == plan_sequence_;
}

void RegulatedNavigator::onPathReady(const nav_msgs::msg::Path & path)
{
  freezeAutonomousPath(path);
}

void RegulatedNavigator::handlePlanningFailure(const std::string & reason)
{
  startRecovery(reason);
}

}
// namespace nav2_regulated_modules
