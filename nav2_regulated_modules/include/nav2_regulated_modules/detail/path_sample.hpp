// 合并相邻二维距离≤1e-9 米的采样点，用新末点保留精确业务终点与姿态，其余位置按原顺序追加。
#ifndef NAV2_REGULATED_MODULES__DETAIL__PATH_SAMPLE_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__PATH_SAMPLE_HPP_

#include <cmath>
#include <utility>

namespace nav2_regulated_modules
{
namespace detail
{
// 与切线判定采用相同容差；末点取新样本，保留精确业务终点。
template<typename Sequence, typename Pose>
// 相邻二维位置距离≤1e-9 m 时，用新 Pose 替换末点而非追加。
// 新姿态／精确终点得以保留，避免重复点导致零长度切线；不删除非相邻重复点。
void appendPathSample(Sequence & samples, Pose pose)
{
  if (!samples.empty())
  {
    const auto & previous = samples.back().pose.position;
    const auto & next = pose.pose.position;
    if (std::hypot(next.x - previous.x, next.y - previous.y) <= 1e-9)
    {
      samples.back() = std::move(pose);
      return;
    }
  }
  samples.push_back(std::move(pose));
}
}
}

#endif
