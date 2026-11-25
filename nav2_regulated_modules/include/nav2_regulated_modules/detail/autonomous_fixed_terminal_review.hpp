// 终点制动、位置复核、航向对齐和连续停稳的纯状态机。捕获裕量限定等待范围，成功仍使用独立的位置与角度容差。
#ifndef NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_TERMINAL_REVIEW_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_TERMINAL_REVIEW_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace nav2_regulated_modules::detail
{
// 各阶段区分制动、位置等待、航向调整、连续停稳及失败原因，供上层状态诊断。
enum class FixedReviewPhase
{
  BRAKE,
  WAIT_POSITION,
  TRIM_ALIGN,
  POSITION_TRIM,
  ALIGN_GOAL,
  SETTLE,
  READY,
  FAILED_CAPTURE,
  FAILED_POSITION_TIMEOUT
};

// 将终点复核枚举转换成稳定的状态名称，供诊断和状态消息使用。
inline const char * fixedReviewPhaseName(const FixedReviewPhase phase)
{
  switch (phase)
  {
    case FixedReviewPhase::BRAKE: return "BRAKE";
    case FixedReviewPhase::WAIT_POSITION: return "WAIT_POSITION";
    case FixedReviewPhase::TRIM_ALIGN: return "TRIM_ALIGN";
    case FixedReviewPhase::POSITION_TRIM: return "POSITION_TRIM";
    case FixedReviewPhase::ALIGN_GOAL: return "ALIGN_GOAL";
    case FixedReviewPhase::SETTLE: return "SETTLE";
    case FixedReviewPhase::READY: return "READY";
    case FixedReviewPhase::FAILED_CAPTURE: return "FAILED_CAPTURE";
    case FixedReviewPhase::FAILED_POSITION_TIMEOUT: return "FAILED_POSITION_TIMEOUT";
  }
  return "FAILED_CAPTURE";
}

struct FixedTerminalReview
{
  FixedReviewPhase phase = FixedReviewPhase::BRAKE;
  // 连续制动、位置等待与最终停稳使用独立计时，避免把前一阶段等待并入后一阶段。
  bool brake_complete = false;
  std::optional<double> brake_since;
  std::optional<double> position_since;
  std::optional<double> settled_since;
  std::optional<double> last_time;

  void reset()
  {
    *this = FixedTerminalReview();
  }

  bool failed() const
  {
    return phase == FixedReviewPhase::FAILED_CAPTURE || phase == FixedReviewPhase::FAILED_POSITION_TIMEOUT;
  }
};

// 按制动、位置等待、航向对齐和停稳窗口推进复核；捕获裕量限制等待范围，采样中断会重建连续稳定计时。
inline FixedReviewPhase reviewFixedTerminal(FixedTerminalReview & state, const double xy, const double yaw, const bool stationary, const double now, const double xy_tolerance, const double yaw_tolerance, const double brake_duration = 0.2, const double settle_duration = 0.5, const double review_timeout = 2.0, const double capture_margin = 0.02)
{
  if (!std::isfinite(xy) || xy < 0.0 || !std::isfinite(yaw) || !std::isfinite(now) || !std::isfinite(xy_tolerance) || xy_tolerance <= 0.0 || !std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0 || !std::isfinite(brake_duration) || brake_duration <= 0.0 || !std::isfinite(settle_duration) || settle_duration <= 0.0 || !std::isfinite(review_timeout) || review_timeout <= settle_duration || !std::isfinite(capture_margin) || capture_margin <= 0.0)
  {
    throw std::invalid_argument("Invalid autonomous fixed terminal review input");
  }
  if (state.failed())
  {
    return state.phase;
  }
  if (state.last_time && now < *state.last_time)
  {
    state.reset();
  }
  else if (state.last_time && now - *state.last_time > 0.2)
  {
    // 采样中断不能凑足连续停稳／精度时间；已开始的有限等待仍保留上限。
    state.brake_complete = false;
    state.brake_since.reset();
    state.settled_since.reset();
  }
  state.last_time = now;
  if (xy > xy_tolerance + capture_margin)
  {
    state.phase = FixedReviewPhase::FAILED_CAPTURE;
    return state.phase;
  }
  if (!state.brake_complete)
  {
    state.settled_since.reset();
    if (!stationary)
    {
      state.brake_since.reset();
    }
    else if (!state.brake_since)
    {
      state.brake_since = now;
    }
    if (!state.brake_since || now - *state.brake_since < brake_duration)
    {
      state.phase = FixedReviewPhase::BRAKE;
      return state.phase;
    }
    state.brake_complete = true;
  }
  if (xy > xy_tolerance)
  {
    state.settled_since.reset();
    if (!stationary)
    {
      state.brake_complete = false;
      state.brake_since.reset();
      state.phase = FixedReviewPhase::BRAKE;
      return state.phase;
    }
    if (!state.position_since)
    {
      state.position_since = now;
    }
    state.phase = now - *state.position_since >= review_timeout ? FixedReviewPhase::FAILED_POSITION_TIMEOUT : FixedReviewPhase::WAIT_POSITION;
    return state.phase;
  }
  if (std::abs(yaw) > yaw_tolerance)
  {
    state.settled_since.reset();
    // 位置恢复后允许正常航向对齐；其进展由上层监控，不让 2 秒位置窗口截断有效旋转。
    state.position_since.reset();
    state.phase = FixedReviewPhase::ALIGN_GOAL;
    return state.phase;
  }
  if (!stationary)
  {
    state.brake_complete = false;
    state.brake_since.reset();
    state.settled_since.reset();
    state.phase = FixedReviewPhase::BRAKE;
    return state.phase;
  }
  if (!state.settled_since)
  {
    state.settled_since = now;
  }
  if (now - *state.settled_since >= settle_duration)
  {
    state.position_since.reset();
    state.phase = FixedReviewPhase::READY;
  }
  else if (state.position_since && now - *state.position_since >= review_timeout)
  {
    state.phase = FixedReviewPhase::FAILED_POSITION_TIMEOUT;
  }
  else
  {
    state.phase = FixedReviewPhase::SETTLE;
  }
  return state.phase;
}
}
#endif
