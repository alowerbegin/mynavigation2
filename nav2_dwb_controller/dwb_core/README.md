# dwb_core

## 本地源码接口（2026-11-16）

DWBLocalPlanner 由 Controller Server 装载，调用轨迹生成器和 Critics 评分。整车 YAML 注册 DWB 不代表当前正在使用 DWB；fixed_path 明确选择 FixedPathController，场景 autonomous 默认选择项目 MPPI 包装。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- 插件导出：[local_planner_plugin.xml](local_planner_plugin.xml) ；共 1 个 class 声明，类型为 `dwb_core::DWBLocalPlanner`。导出类与当前选中的插件 ID 分开核对。
- 实现定位：[dwb_local_planner.cpp](src/dwb_local_planner.cpp) 、[publisher.cpp](src/publisher.cpp) 、[illegal_trajectory_tracker.cpp](src/illegal_trajectory_tracker.cpp) 。

## 当前项目用法

实现 DWB 局部控制框架，组合轨迹生成器与评分器选择速度指令。

包类型：控制插件与核心库。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[src/](src/) 、[include/](include/) 、[test/](test/) 。

直接依赖（清单节选，完整约束见 manifest）：`dwb_msgs`、`geometry_msgs`、`nav2_common`、`nav2_core`、`nav2_costmap_2d`、`nav2_util`、`nav_2d_msgs`、`nav_2d_utils`、`nav_msgs`、`pluginlib`、`rclcpp`、`sensor_msgs` 等。

插件声明：[local_planner_plugin.xml](local_planner_plugin.xml) 。插件由对应宿主加载，插件类名以声明 XML 为准。

使用方式：在宿主服务器的插件列表和参数块中配置导出的类；仅构建本包不会自动切换当前运行控制器或规划器。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
