#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav2_regulated_modules
{


rclcpp_action::Client<RegulatedNavigator::FollowPath>::SharedPtr RegulatedNavigator::followClient() const
{
  return task_.type == TaskType::TO_POSE ? lattice_follow_client_ : follow_client_;
}

rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr RegulatedNavigator::speedPublisher() const
{
  return task_.type == TaskType::TO_POSE ? lattice_speed_pub_ : speed_limit_pub_;
}

bool RegulatedNavigator::unifiedDrained() const
{
  return outstanding_plans_ == 0 && fixed_path_outstanding_follows_ == 0 &&
    frozen_outstanding_follows_ == 0;
}

bool RegulatedNavigator::unifiedStopped() const
{
  const auto feedback = motion_state_subscriber_->latestState();
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - feedback.receive_time).count();
  return feedback.valid && feedback.receive_time > stopping_since_ &&
    age <= get_parameter("chassis_motion_state_timeout").as_double() &&
    std::abs(feedback.linear_velocity) < 1e-3 && std::abs(feedback.angular_velocity) < 1e-3 &&
    chassis_control_subscriber_->navigationOutputStopped(stopping_since_);
}

void RegulatedNavigator::pollExecutionStates()
{
  const auto time = std::chrono::steady_clock::now();
  for (auto * group : {&fixed_execution_, &lattice_execution_})
  {
    for (std::size_t i = 0; i < group->size(); ++i)
    {
      auto & node = group->at(i);
      if (node.request && time - node.last_request >= std::chrono::seconds(1))
      {
        node.client->remove_pending_request(*node.request);
        node.request.reset();
        node.active = false;
      }
      if (node.request || !node.client->service_is_ready())
      {
        continue;
      }
      node.last_request = time;
      node.request = node.client->async_send_request(std::make_shared<StateService::Request>(),
        [group, i](rclcpp::Client<StateService>::SharedFuture future)
        {
          auto & state = group->at(i);
          state.active = future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
          state.last_response = std::chrono::steady_clock::now();
          state.request.reset();
        }).request_id;
    }
  }
}

bool RegulatedNavigator::executionReady(const bool lattice) const
{
  const auto & nodes = lattice ? lattice_execution_ : fixed_execution_;
  const auto time = std::chrono::steady_clock::now();
  const auto & client = lattice ? lattice_follow_client_ : follow_client_;
  return client && client->action_server_is_ready() &&
    (!lattice || compute_pose_client_->action_server_is_ready()) && !nodes.empty() &&
    std::all_of(nodes.begin(), nodes.end(),
      [time](const auto & node)
      {
        return node.active && time - node.last_response <= std::chrono::seconds(1);
      });
}

bool RegulatedNavigator::admitUnified(const bool lattice)
{
  if (!active_ || pending_fixed_ || pending_pose_ || task_.state == NavigationState::CANCELING ||
    !executionReady(lattice) || !chassis_control_subscriber_->canStartNavigation())
  {
    return false;
  }
  if (task_.type == TaskType::NONE && (!unifiedDrained() || !unifiedStopped()))
  {
    return false;
  }
  geometry_msgs::msg::PoseStamped pose;
  return lookupCurrentPose(pose);
}

void RegulatedNavigator::finishPending(const bool client_cancel)
{
  const auto finish = [client_cancel](auto & handle, const auto & result)
  {
    if (!handle)
    {
      return;
    }
    if (handle->is_canceling())
    {
      handle->canceled(result);
      handle.reset();
    }
    else if (!client_cancel)
    {
      handle->abort(result);
      handle.reset();
    }
  };
  if (pending_pose_)
  {
    finish(pending_pose_, std::make_shared<NavigateToPose::Result>());
  }
  if (pending_fixed_)
  {
    finish(pending_fixed_, std::make_shared<NavigationService::Result>());
    if (!pending_fixed_)
    {
      pending_path_ = nav_msgs::msg::Path();
    }
  }
}

bool RegulatedNavigator::monitorUnified()
{
  pollExecutionStates();
  finishPending(true);
  const auto time = std::chrono::steady_clock::now();
  if (task_.state == NavigationState::CANCELING)
  {
    const bool stopped = unifiedDrained() && unifiedStopped();
    const bool timed_out = std::chrono::duration<double>(time - stopping_since_).count() >
      cancel_timeout_;
    if (!stopped && !timed_out)
    {
      return true;
    }
    if (timed_out && !stopped)
    {
      LOG_ERROR("导航交接超时，保持停车，等待终态与新鲜停稳反馈");
      finishPending(false);
    }
    finishCancellation();
  }
  if (task_.type != TaskType::NONE)
  {
    if (!executionReady(task_.type == TaskType::TO_POSE))
    {
      cancelTask("当前导航通道生命周期失效");
      return true;
    }
    return false;
  }
  if (!pending_pose_ && !pending_fixed_)
  {
    return true;
  }
  if (std::chrono::duration<double>(time - pending_since_).count() > server_timeout_)
  {
    LOG_ERROR("等待启动待执行任务超时");
    finishPending(false);
    return true;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!unifiedDrained() || !unifiedStopped() || !chassis_control_subscriber_->canStartNavigation() ||
    !lookupCurrentPose(pose))
  {
    return true;
  }
  if (!executionReady(pending_pose_ != nullptr))
  {
    finishPending(false);
    return true;
  }
  if (pending_pose_)
  {
    auto goal = std::exchange(pending_pose_, nullptr);
    startPose(goal);
  }
  else
  {
    auto goal = std::exchange(pending_fixed_, nullptr);
    startFixedPath(goal, std::exchange(pending_path_, nav_msgs::msg::Path()));
  }
  return true;
}

}  // namespace nav2_regulated_modules
