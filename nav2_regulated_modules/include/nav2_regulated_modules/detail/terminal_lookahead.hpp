// 在距终点 0.5～0.1 米的区间，将正常前视距离线性混合到终点前视距离，输入与输出均使用米。
#ifndef NAV2_REGULATED_MODULES__DETAIL__TERMINAL_LOOKAHEAD_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__TERMINAL_LOOKAHEAD_HPP_
#include <algorithm>
namespace nav2_regulated_modules
{
namespace detail
{
// 混合比例 = clamp((0.5−distance)÷0.4, 0, 1)。
// 输出前视 = normal + 混合比例 × (terminal−normal)，所有距离单位为 m。
inline double terminalLookahead(double normal, double terminal, double distance)
{
  const double blend = std::clamp((0.5 - distance) / 0.4, 0.0, 1.0);
  return normal + blend * (terminal - normal);
}
}
}
#endif
