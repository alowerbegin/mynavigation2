// 当前半自主通道的 Lattice 包装规划器。完成状态格搜索后校正端点与起点残差，重新检查足迹及扫掠采样，再返回一次规划结果。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <mutex>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_path.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{
class AutonomousLatticePlanner : public nav2_smac_planner::SmacPlannerLattice
{
public:
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override
  {
    const auto began = std::chrono::steady_clock::now();
    auto plan = nav2_smac_planner::SmacPlannerLattice::createPlan(start, goal);
    if (plan.poses.empty())
    {
      return plan;
    }
    // 平滑器会按首段切线重写首点朝向；起点对齐必须使用真实起点朝向。
    if (start.header.frame_id.empty() || plan.header.frame_id.empty() ||
      start.header.frame_id == plan.header.frame_id)
    {
      plan.poses.front().pose.orientation = start.pose.orientation;
    }
    std::lock_guard<std::mutex> planner_lock(_mutex);
    auto * costmap = _costmap_ros->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap->getMutex());
    if (!_costmap_ros->isCurrent())
    {
      throw nav2_core::PlannerException("Lattice correction requires a current costmap");
    }
    const double dx = goal.pose.position.x - plan.poses.back().pose.position.x;
    const double dy = goal.pose.position.y - plan.poses.back().pose.position.y;
    const double offset = std::hypot(dx, dy);
    const double resolution = costmap->getResolution();
    // 5 cm 状态格的尾点量化偏差上限是半格对角（sqrt(2)/2 * resolution ≈ 3.5 cm）
    const double limit = std::sqrt(0.5) * resolution + _tolerance + 1e-6;
    if (!std::isfinite(offset) || !std::isfinite(resolution) || resolution <= 0.0 || offset > limit)
    {
      throw nav2_core::PlannerException("Lattice grid-center correction exceeds its bounded offset");
    }
    if (plan.poses.size() == 1 && offset > 1e-6)
    {
      throw nav2_core::PlannerException("Same-cell lattice result has no translation segment for precise arrival");
    }
    // 此处先对整条路径作等量平移，保留这一阶段的相对几何，后续再处理起点残差和末端航向。
    for (auto & pose : plan.poses)
    {
      pose.pose.position.x += dx;
      pose.pose.position.y += dy;
    }
    plan.poses.back().pose.orientation = goal.pose.orientation;
    if (plan.poses.size() >= 2)
    {
      const double need_x = start.pose.position.x - plan.poses.front().pose.position.x;
      const double need_y = start.pose.position.y - plan.poses.front().pose.position.y;
      if (std::hypot(need_x, need_y) > 1e-9)
      {
        const std::size_t count = plan.poses.size();
        std::vector<double> arc(count, 0.0);
        for (std::size_t i = count - 1; i > 0; --i)
        {
          arc[i - 1] = arc[i] + std::hypot(
            plan.poses[i - 1].pose.position.x - plan.poses[i].pose.position.x,
            plan.poses[i - 1].pose.position.y - plan.poses[i].pose.position.y);
        }
        const double total = arc.front();
        if (total > 1e-6)
        {
          for (std::size_t i = 0; i < count; ++i)
          {
            const double weight = arc[i] / total;
            plan.poses[i].pose.position.x += need_x * weight;
            plan.poses[i].pose.position.y += need_y * weight;
          }
        }
        else
        {
          plan.poses.front().pose.position = start.pose.position;
        }
      }
    }
    // 换向点两侧统一为同一车头朝向，避免状态格分箱差在冻结后变成额外旋转段。
    alignCuspHeadings(plan);
    // 删除终点前近重合的旋转点，避免最终段退化成纯旋转而无法收敛位置。
    mergeTerminalRotation(plan);
    const auto footprint = _costmap_ros->getRobotFootprint();
    if (footprint.size() < 3)
    {
      throw nav2_core::PlannerException("Lattice correction requires a polygon footprint");
    }
    double radius = 0.0;
    for (const auto & vertex : footprint)
    {
      radius = std::max(radius, std::hypot(vertex.x, vertex.y));
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
    const auto validate = [&](const double x, const double y, const double yaw)
    {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      if (elapsed >= _max_planning_time)
      {
        throw nav2_core::PlannerException("Lattice search and correction exceeded the planning deadline");
      }
      unsigned int mx = 0;
      unsigned int my = 0;
      // 只检查是否落在地图内；中心点落在内切膨胀区并不代表实际矩形足迹碰撞，
      // Smac 自身也只按足迹判定，这里多一道中心代价检查会把可通行走廊误判为阻塞。
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !costmap->worldToMap(x, y, mx, my))
      {
        throw nav2_core::PlannerException("Lattice corrected pose is outside the costmap");
      }
      const double cost = checker.footprintCostAtPose(x, y, yaw, footprint);
      // NO_INFORMATION(255) 数值上大于 LETHAL_OBSTACLE(254)，必须先单独判别未知区域，
      // 否则 allow_unknown=1 时仍会被当作致命障碍拒绝。只有致命障碍（254）、足迹出图（-1）
      // 以及禁用未知通行时的未知栅格才拒绝。
      const bool unknown = cost == nav2_costmap_2d::NO_INFORMATION;
      const bool lethal = !unknown && cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
      if (cost < 0.0 || !std::isfinite(cost) || lethal || (unknown && !_allow_unknown))
      {
        RCLCPP_WARN(_logger, "Lattice correction rejected at x=%.3f y=%.3f yaw=%.3f cost=%.1f allow_unknown=%d",
          x, y, yaw, cost, static_cast<int>(_allow_unknown));
        throw nav2_core::PlannerException("Lattice corrected footprint is blocked or outside the costmap");
      }
    };
    // Validate the actual start and the swept alignment to the corrected first sample.
    auto previous = start;
    validate(previous.pose.position.x, previous.pose.position.y, tf2::getYaw(previous.pose.orientation));
    const double sample_step = std::min(0.005, resolution * 0.5);
    for (const auto & pose : plan.poses)
    {
      const double x0 = previous.pose.position.x;
      const double y0 = previous.pose.position.y;
      const double yaw0 = tf2::getYaw(previous.pose.orientation);
      const double yaw_delta = std::remainder(tf2::getYaw(pose.pose.orientation) - yaw0, 2.0 * M_PI);
      const double distance = std::hypot(pose.pose.position.x - x0, pose.pose.position.y - y0);
      const double swept_distance = distance + radius * std::abs(yaw_delta);
      if (!std::isfinite(swept_distance))
      {
        throw nav2_core::PlannerException("Lattice path contains a non-finite pose");
      }
      const auto steps = static_cast<std::size_t>(std::max(1.0, std::ceil(swept_distance / sample_step)));
      for (std::size_t i = 1; i <= steps; ++i)
      {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        validate(x0 + t * (pose.pose.position.x - x0), y0 + t * (pose.pose.position.y - y0), yaw0 + t * yaw_delta);
      }
      previous = pose;
    }
    RCLCPP_INFO(_logger, "Lattice goal correction before freeze: dx=%.6f dy=%.6f m, poses=%zu, searches=1", dx, dy, plan.poses.size());
    return plan;
  }

private:
  static constexpr double kCuspRotationDistance = 0.025;
  static constexpr double kCuspAlignMaxError = 1.0471975511965976;
  static constexpr double kCuspApproachLength = 0.12;
  static constexpr double kCuspApproachMaxLateral = 0.03;

  static double cuspPoseYaw(const geometry_msgs::msg::PoseStamped & pose)
  {
    return tf2::getYaw(pose.pose.orientation);
  }

  static void setCuspPoseYaw(geometry_msgs::msg::PoseStamped & pose, const double yaw)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    tf2::convert(quaternion, pose.pose.orientation);
  }

  static bool cuspLastMovementTangent(
    const nav_msgs::msg::Path & path, const detail::FixedSegment & segment,
    const double rotation_distance, double & tangent)
  {
    for (std::size_t index = segment.last; index > segment.first; --index)
    {
      if (detail::fixedDistance(path.poses[index - 1], path.poses[index]) > rotation_distance)
      {
        tangent = std::atan2(
          path.poses[index].pose.position.y - path.poses[index - 1].pose.position.y,
          path.poses[index].pose.position.x - path.poses[index - 1].pose.position.x);
        return true;
      }
    }
    return false;
  }

  static bool cuspFirstMovementTangent(
    const nav_msgs::msg::Path & path, const detail::FixedSegment & segment,
    const double rotation_distance, double & tangent)
  {
    for (std::size_t index = segment.first; index < segment.last; ++index)
    {
      if (detail::fixedDistance(path.poses[index], path.poses[index + 1]) > rotation_distance)
      {
        tangent = std::atan2(
          path.poses[index + 1].pose.position.y - path.poses[index].pose.position.y,
          path.poses[index + 1].pose.position.x - path.poses[index].pose.position.x);
        return true;
      }
    }
    return false;
  }

  static double cuspCarHeading(const double tangent, const int direction)
  {
    return detail::fixedAngle(tangent + (direction < 0 ? M_PI : 0.0));
  }

  static std::vector<int> cuspTranslationDirections(
    const std::vector<detail::FixedSegment> & segments)
  {
    std::vector<int> directions;
    for (const auto & segment : segments)
    {
      if (!segment.rotation)
      {
        directions.push_back(segment.direction);
      }
    }
    return directions;
  }

  static bool cuspCandidateValid(
    const nav_msgs::msg::Path & candidate,
    const std::vector<int> & original_directions,
    const double rotation_distance,
    const double endpoint_yaw,
    const std::size_t check_first,
    const std::size_t check_last)
  {
    std::vector<detail::FixedSegment> segments;
    try
    {
      segments = detail::splitFixedPath(candidate, rotation_distance);
    }
    catch (const std::exception &)
    {
      return false;
    }
    if (cuspTranslationDirections(segments) != original_directions)
    {
      return false;
    }
    for (const auto & segment : segments)
    {
      if (segment.rotation && segment.first <= check_last && segment.last >= check_first)
      {
        return false;
      }
    }
    return std::abs(detail::fixedAngle(cuspPoseYaw(candidate.poses.back()) - endpoint_yaw)) <= 1e-6;
  }

  void mergeTerminalRotation(nav_msgs::msg::Path & plan)
  {
    if (plan.poses.size() < 4)
    {
      return;
    }
    std::vector<detail::FixedSegment> segments;
    try
    {
      segments = detail::splitFixedPath(plan, kCuspRotationDistance);
    }
    catch (const std::exception &)
    {
      return;
    }
    if (segments.empty())
    {
      return;
    }
    const auto & tail = segments.back();
    if (!tail.rotation || tail.last + 1 != plan.poses.size() || tail.first <= 1 || tail.first >= tail.last)
    {
      return;
    }
    const auto original_directions = cuspTranslationDirections(segments);
    const double shortcut = detail::fixedDistance(plan.poses[tail.first - 1], plan.poses.back());
    if (shortcut > 0.15)
    {
      RCLCPP_WARN(_logger, "Lattice terminal rotation merge skipped: shortcut=%.3fm exceeds 0.15m", shortcut);
      return;
    }
    auto candidate = plan;
    candidate.poses.erase(candidate.poses.begin() + tail.first, candidate.poses.begin() + tail.last);
    if (candidate.poses.size() < 2)
    {
      return;
    }
    std::vector<detail::FixedSegment> merged;
    try
    {
      merged = detail::splitFixedPath(candidate, kCuspRotationDistance);
    }
    catch (const std::exception &)
    {
      return;
    }
    if (cuspTranslationDirections(merged) != original_directions ||
      merged.empty() || merged.back().rotation || merged.back().last + 1 != candidate.poses.size())
    {
      RCLCPP_WARN(_logger, "Lattice terminal rotation merge rejected: shortcut=%.3fm poses=%zu", shortcut, plan.poses.size());
      return;
    }
    plan = candidate;
    RCLCPP_INFO(_logger, "Lattice terminal rotation merge: removed=%zu shortcut=%.3fm poses=%zu",
      tail.last - tail.first, shortcut, plan.poses.size());
  }

  static bool cuspBoundaryPreserved(
    const nav_msgs::msg::Path & candidate,
    const double rotation_distance,
    const std::size_t previous_last,
    const std::size_t next_first,
    const int previous_direction,
    const int next_direction)
  {
    std::vector<detail::FixedSegment> segments;
    try
    {
      segments = detail::splitFixedPath(candidate, rotation_distance);
    }
    catch (const std::exception &)
    {
      return false;
    }
    bool previous_ok = false;
    bool next_ok = false;
    for (const auto & segment : segments)
    {
      if (!segment.rotation && segment.last == previous_last && segment.direction == previous_direction)
      {
        previous_ok = true;
      }
      if (!segment.rotation && segment.first == next_first && segment.direction == next_direction)
      {
        next_ok = true;
      }
    }
    return previous_ok && next_ok;
  }

  void alignCuspHeadings(nav_msgs::msg::Path & plan)
  {
    if (plan.poses.size() < 4)
    {
      return;
    }
    std::vector<detail::FixedSegment> segments;
    try
    {
      segments = detail::splitFixedPath(plan, kCuspRotationDistance);
    }
    catch (const std::exception & error)
    {
      RCLCPP_WARN(_logger, "Lattice cusp heading alignment unavailable: %s", error.what());
      return;
    }
    const auto original_directions = cuspTranslationDirections(segments);
    if (original_directions.size() < 2)
    {
      return;
    }
    const double endpoint_yaw = cuspPoseYaw(plan.poses.back());
    std::size_t applied = 0;
    std::size_t rotation_applied = 0;
    std::size_t rejected = 0;
    for (std::size_t index = 0; index < segments.size(); ++index)
    {
      const auto & current = segments[index];
      if (current.rotation)
      {
        if (index == 0 || index + 1 >= segments.size())
        {
          continue;
        }
        const auto & previous = segments[index - 1];
        const auto & next = segments[index + 1];
        if (previous.rotation || next.rotation || previous.direction * next.direction >= 0)
        {
          continue;
        }
        if (current.first <= 1 || current.last + 2 >= plan.poses.size())
        {
          continue;
        }
        double previous_tangent = 0.0;
        double next_tangent = 0.0;
        if (!cuspLastMovementTangent(plan, previous, kCuspRotationDistance, previous_tangent) ||
          !cuspFirstMovementTangent(plan, next, kCuspRotationDistance, next_tangent))
         {
          continue;
        }
        const double previous_heading = cuspCarHeading(previous_tangent, previous.direction);
        const double next_heading = cuspCarHeading(next_tangent, next.direction);
        const double mismatch = detail::fixedAngle(next_heading - previous_heading);
        if (std::abs(mismatch) > kCuspAlignMaxError)
        {
          RCLCPP_WARN(_logger, "Lattice cusp heading align skipped: type=rotation index=%zu mismatch=%.2fdeg limit=%.2fdeg",
            index, mismatch * 180.0 / M_PI, kCuspAlignMaxError * 180.0 / M_PI);
          ++rejected;
          continue;
        }
        const double common_heading = previous_heading;
        auto candidate = plan;
        for (std::size_t pose_index = current.first; pose_index <= current.last; ++pose_index)
        {
          setCuspPoseYaw(candidate.poses[pose_index], common_heading);
        }
        if (detail::fixedDistance(candidate.poses[current.first - 1], candidate.poses[current.first]) <= kCuspRotationDistance)
        {
          setCuspPoseYaw(candidate.poses[current.first - 1], common_heading);
        }
        if (detail::fixedDistance(candidate.poses[current.last], candidate.poses[current.last + 1]) <= kCuspRotationDistance)
        {
          setCuspPoseYaw(candidate.poses[current.last + 1], common_heading);
        }
        if (cuspCandidateValid(candidate, original_directions, kCuspRotationDistance, endpoint_yaw, current.first, current.last))
        {
          plan = candidate;
          ++applied;
          ++rotation_applied;
          RCLCPP_INFO(_logger, "Lattice cusp heading align: type=rotation index=%zu prev_dir=%d next_dir=%d prev_heading=%.3f next_heading=%.3f mismatch=%.2fdeg common=%.3f accepted=1",
            index, previous.direction, next.direction, previous_heading, next_heading, mismatch * 180.0 / M_PI, common_heading);
        }
        else
        {
          ++rejected;
          RCLCPP_WARN(_logger, "Lattice cusp heading align rejected: type=rotation index=%zu mismatch=%.2fdeg",
            index, mismatch * 180.0 / M_PI);
        }
        continue;
      }
      if (index + 1 >= segments.size() || segments[index + 1].rotation ||
        current.direction * segments[index + 1].direction >= 0)
      {
        continue;
      }
      const auto & next = segments[index + 1];
      // splitFixedPath 会跳过零位移点，前后段首尾索引可能相差 1 个点。
      const std::size_t gap_first = current.last;
      const std::size_t gap_last = next.first;
      if (gap_last < gap_first || gap_first <= 1 || gap_last + 1 >= plan.poses.size())
      {
        continue;
      }
      double previous_tangent = 0.0;
      double next_tangent = 0.0;
      if (!cuspLastMovementTangent(plan, current, kCuspRotationDistance, previous_tangent) ||
        !cuspFirstMovementTangent(plan, next, kCuspRotationDistance, next_tangent))
      {
        continue;
      }
      const double previous_heading = cuspCarHeading(previous_tangent, current.direction);
      const double next_heading = cuspCarHeading(next_tangent, next.direction);
      const double mismatch = detail::fixedAngle(next_heading - previous_heading);
      if (std::abs(mismatch) > kCuspAlignMaxError)
      {
        RCLCPP_WARN(_logger, "Lattice cusp heading align skipped: type=boundary index=%zu..%zu mismatch=%.2fdeg limit=%.2fdeg",
          gap_first, gap_last, mismatch * 180.0 / M_PI, kCuspAlignMaxError * 180.0 / M_PI);
        ++rejected;
        continue;
      }
      const double common_heading = previous_heading;
      auto candidate = plan;
      for (std::size_t pose_index = gap_first; pose_index <= gap_last; ++pose_index)
      {
        setCuspPoseYaw(candidate.poses[pose_index], common_heading);
      }
      if (gap_first > 1 &&
        detail::fixedDistance(candidate.poses[gap_first - 1], candidate.poses[gap_first]) <= kCuspRotationDistance)
      {
        setCuspPoseYaw(candidate.poses[gap_first - 1], common_heading);
      }
      if (gap_last + 2 < candidate.poses.size() &&
        detail::fixedDistance(candidate.poses[gap_last], candidate.poses[gap_last + 1]) <= kCuspRotationDistance)
      {
        setCuspPoseYaw(candidate.poses[gap_last + 1], common_heading);
      }
      auto oriented = candidate;
      bool straightened = false;
      double tail_length = 0.0;
      double lateral = 0.0;
      if (gap_first > current.first + 3)
      {
        const std::size_t start = gap_first - 3;
        std::vector<double> arc(gap_first - start + 1, 0.0);
        for (std::size_t pose_index = start + 1; pose_index <= gap_first; ++pose_index)
        {
          arc[pose_index - start] = arc[pose_index - start - 1] +
            detail::fixedDistance(candidate.poses[pose_index - 1], candidate.poses[pose_index]);
        }
        tail_length = arc.back();
        if (tail_length >= 0.05)
        {
          const double movement_heading = detail::fixedAngle(
            common_heading + (current.direction < 0 ? M_PI : 0.0));
          const auto & cusp = candidate.poses[gap_first].pose.position;
          const double target_x = cusp.x - std::cos(movement_heading) * tail_length;
          const double target_y = cusp.y - std::sin(movement_heading) * tail_length;
          lateral = std::hypot(
            candidate.poses[start].pose.position.x - target_x,
            candidate.poses[start].pose.position.y - target_y);
          if (lateral <= kCuspApproachMaxLateral)
          {
            for (std::size_t pose_index = start; pose_index <= gap_first; ++pose_index)
            {
              const double weight = arc[pose_index - start] / tail_length;
              candidate.poses[pose_index].pose.position.x = target_x + weight * (cusp.x - target_x);
              candidate.poses[pose_index].pose.position.y = target_y + weight * (cusp.y - target_y);
              setCuspPoseYaw(candidate.poses[pose_index], common_heading);
            }
            straightened = true;
          }
        }
      }
      bool valid = cuspCandidateValid(
        candidate, original_directions, kCuspRotationDistance, endpoint_yaw, gap_first, gap_last) &&
        cuspBoundaryPreserved(
        candidate, kCuspRotationDistance, gap_first, gap_last, current.direction, next.direction);
      if (!valid && straightened)
      {
        candidate = oriented;
        straightened = false;
        valid = cuspCandidateValid(
          candidate, original_directions, kCuspRotationDistance, endpoint_yaw, gap_first, gap_last) &&
          cuspBoundaryPreserved(
          candidate, kCuspRotationDistance, gap_first, gap_last, current.direction, next.direction);
      }
      if (valid)
      {
        plan = candidate;
        ++applied;
        RCLCPP_INFO(_logger, "Lattice cusp heading align: type=boundary index=%zu..%zu prev_dir=%d next_dir=%d prev_heading=%.3f next_heading=%.3f mismatch=%.2fdeg common=%.3f straighten=%d tail=%.3fm lateral=%.3fm accepted=1",
          gap_first, gap_last, current.direction, next.direction, previous_heading, next_heading,
          mismatch * 180.0 / M_PI, common_heading, straightened ? 1 : 0, tail_length, lateral);
      }
      else
      {
        ++rejected;
        RCLCPP_WARN(_logger, "Lattice cusp heading align rejected: type=boundary index=%zu..%zu mismatch=%.2fdeg",
          gap_first, gap_last, mismatch * 180.0 / M_PI);
      }
    }
    if (applied > 0 || rejected > 0)
    {
      RCLCPP_INFO(_logger, "Lattice cusp heading align summary: applied=%zu rotation_applied=%zu rejected=%zu poses=%zu",
        applied, rotation_applied, rejected, plan.poses.size());
    }
  }

};
}  // namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::AutonomousLatticePlanner, nav2_core::GlobalPlanner)
