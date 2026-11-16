# dwb_critics

## 本地源码接口（2026-11-16）

Critics 给 DWB 生成的候选轨迹评分，插件选择、权重和短路行为由 DWB 参数决定。它们不独立发布底盘速度，也不参与当前 fixed_path 的专用几何跟踪。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- 插件导出：[default_critics.xml](default_critics.xml) ；共 10 个 class 声明，包括 `dwb_critics::PreferForwardCritic`、`dwb_critics::GoalDistCritic`、`dwb_critics::PathAlignCritic`、`dwb_critics::GoalAlignCritic`。导出类与当前选中的插件 ID 分开核对。
- 实现定位：[alignment_util.cpp](src/alignment_util.cpp) 、[map_grid.cpp](src/map_grid.cpp) 、[goal_dist.cpp](src/goal_dist.cpp) 。

## 当前项目用法

提供障碍、路径、目标、朝向及振荡等评分器，由 DWB Controller 装载。

包类型：轨迹评分插件。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[src/](src/) 、[include/](include/) 、[test/](test/) 。

直接依赖（清单节选，完整约束见 manifest）：`angles`、`costmap_queue`、`dwb_core`、`geometry_msgs`、`nav2_common`、`nav2_costmap_2d`、`nav2_util`、`nav_2d_msgs`、`nav_2d_utils`、`pluginlib`、`rclcpp`、`sensor_msgs`。

插件声明：[default_critics.xml](default_critics.xml) 。插件由对应宿主加载，插件类名以声明 XML 为准。

使用方式：在宿主服务器的插件列表和参数块中配置导出的类；仅构建本包不会自动切换当前运行控制器或规划器。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
