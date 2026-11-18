// 仅 autonomous_lattice 使用：一次状态格搜索，冻结前校正格心偏差与分箱航向。
// autonomous_fixed 仍使用 autonomous_fixed_hybrid_planner.cpp，两者互不影响。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "pluginlib/class_list_macros.hpp"
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
    // Rigid translation preserves curvature, yaw, path length and every direction-change cusp.
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
};
}  // namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::AutonomousLatticePlanner, nav2_core::GlobalPlanner)
