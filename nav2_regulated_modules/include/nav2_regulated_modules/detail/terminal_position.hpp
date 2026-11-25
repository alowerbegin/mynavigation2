// 提供末端位置及越过终点平面的几何判断，供跟踪器与到点检查器采用一致口径。
#ifndef NAV2_REGULATED_MODULES__DETAIL__TERMINAL_POSITION_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__TERMINAL_POSITION_HPP_

namespace nav2_regulated_modules
{
namespace detail
{
struct TerminalPosition
{
  bool stopped;
  bool accurate;
};

// entry>0 时以入口半径触发停车，否则采用 tolerance；crossed 与既有停止状态也可锁停。
// accurate 按 tolerance 判断；entry=0 且此前已锁停并准确时，继续保持准确标志。
inline TerminalPosition terminalPosition(double distance, double tolerance, double entry, bool crossed, bool was_stopped, bool was_accurate)
{
  const double stop_radius = entry > 0.0 ? entry : tolerance;
  TerminalPosition result;
  result.stopped = was_stopped || distance <= stop_radius || crossed;
  result.accurate = result.stopped && (distance <= tolerance || (entry == 0.0 && was_stopped && was_accurate));
  return result;
}
}
}

#endif
