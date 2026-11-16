// fixed_path 的唯一底盘出口。默认回调组串行执行，mutex_ 保护遥控目标与出口状态。
// 交接／反馈失效发保护性零速；正常导航透传原平滑输出，遥控保留原 S 曲线及限幅。
#include "nav2_regulated_modules/chassis_control_subscriber.hpp"

#include <cmath>

#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

bool ChassisControlSubscriber::motionFresh(const MotionStateSnapshot & state) const
{
  return state.valid && std::chrono::duration<double>(std::chrono::steady_clock::now() - state.receive_time).count() <= config_.motion_state_timeout;
}

bool ChassisControlSubscriber::motionStopped(const MotionStateSnapshot & state) const
{
  return motionFresh(state) && std::abs(state.linear_velocity) < 1e-3 && std::abs(state.angular_velocity) < 1e-3;
}

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

void ChassisControlSubscriber::onNavigationVelocity(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z))
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_ && config_.navigation_active() && (authority_ == Authority::IDLE || authority_ == Authority::NAVIGATION))
  {
    navigation_velocity_ = *message;
    navigation_receive_time_ = std::chrono::steady_clock::now();
    has_navigation_velocity_ = true;
    authority_ = Authority::NAVIGATION;
  }
}

// 调用方已持 mutex_。已取消路径的迟到 Twist 在订阅回调中丢弃，不作为后续任务的缓存。
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
  if (config_.navigation_active() && has_navigation_velocity_)
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
