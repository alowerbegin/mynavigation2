// 可选自适应制动的纯纵向计算，返回停车距离、速度包络与减速度，由上层控制器选择和执行对应分支。
#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_regulated_modules
{

// 纯纵向计算，不依赖 ROS；控制器和单元测试使用同一公式。
// deceleration 是本轮命令减速度上限 m/s²，stopping_distance 是含预留的估算停车距离 m，speed_limit 是包络上限 m/s。
// 返回全零表示三个运动输入校验失败，不是已经执行停车。
struct FixedPathSpeedProfile
{
  double deceleration;
  double stopping_distance;
  double speed_limit;
};

// 根据实测速度、剩余距离及制动约束计算速度包络；速度为 m/s，距离为 m，加速度为 m/s²，jerk 为 m/s³，约束合法性由调用方保证。
inline FixedPathSpeedProfile calculateFixedPathSpeedProfile(const double measured_speed, const double remaining_distance, const double nominal_speed, const double max_deceleration, const double jerk_limit, const double response_time, const double distance_margin, const double goal_tolerance, const double approach_speed)
{
  if (!std::isfinite(measured_speed) || !std::isfinite(remaining_distance) || !std::isfinite(nominal_speed) || measured_speed < 0.0 || remaining_distance < 0.0 || nominal_speed < 0.0)
  {
    return {0.0, 0.0, 0.0};
  }
  const double minimum_deceleration = std::min(0.25, max_deceleration);
  const double speed_fraction = std::clamp((measured_speed - 0.2) / 1.3, 0.0, 1.0);
  const double deceleration = minimum_deceleration + (max_deceleration - minimum_deceleration) * speed_fraction;
  const double planning_deceleration = std::min(deceleration, 0.4);
  const double jerk_distance = measured_speed * planning_deceleration / (2.0 * jerk_limit);
  const double guard_distance = goal_tolerance + distance_margin + measured_speed * response_time + jerk_distance;
  const double stopping_distance = measured_speed * measured_speed / (2.0 * planning_deceleration) + guard_distance;
  const double available_distance = std::max(0.0, remaining_distance - guard_distance);
  const double braking_speed = std::sqrt(2.0 * planning_deceleration * available_distance);
  const double speed_limit = std::min(nominal_speed, std::max(std::min(approach_speed, nominal_speed), braking_speed));
  return {deceleration, stopping_distance, speed_limit};
}

// 以正 dt 限制减速度每周期的变化幅值，积分后速度保持不低于 target_speed，next_deceleration 回传本轮减速度。
inline double advanceFixedPathBrakingSpeed(const double previous_speed, const double target_speed, const double previous_deceleration, const double max_deceleration, const double jerk_limit, const double dt, double & next_deceleration)
{
  const double requested_deceleration = std::clamp((previous_speed - target_speed) / dt, 0.0, max_deceleration);
  next_deceleration = std::clamp(requested_deceleration, std::max(0.0, previous_deceleration - jerk_limit * dt), std::min(max_deceleration, previous_deceleration + jerk_limit * dt));
  return std::max(target_speed, previous_speed - next_deceleration * dt);
}

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_SPEED_PROFILE_HPP_
