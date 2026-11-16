# dwb_plugins

## 本地源码接口（2026-11-16）

本包导出 DWB 轨迹生成和相关检查插件，实际类名与基类见 XML。轨迹生成器受 DWB 采样与运动约束控制，不与 fixed_path 的后台遥控 S 曲线混用。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- 插件导出：[plugins.xml](plugins.xml) ；共 4 个 class 声明，类型为 `dwb_plugins::SimpleGoalChecker`、`dwb_plugins::StandardTrajectoryGenerator`、`dwb_plugins::LimitedAccelGenerator`、`dwb_plugins::StoppedGoalChecker`。导出类与当前选中的插件 ID 分开核对。
- 实现定位：[standard_traj_generator.cpp](src/standard_traj_generator.cpp) 、[limited_accel_generator.cpp](src/limited_accel_generator.cpp) 、[kinematic_parameters.cpp](src/kinematic_parameters.cpp) 。

## 当前项目用法

提供 DWB 标准及受加速度约束的轨迹生成器，供 dwb_core 采样使用。

包类型：轨迹生成插件。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[src/](src/) 、[include/](include/) 、[test/](test/) 。

直接依赖（清单节选，完整约束见 manifest）：`angles`、`dwb_core`、`nav2_common`、`nav2_util`、`nav_2d_msgs`、`nav_2d_utils`、`pluginlib`、`rclcpp`。

插件声明：[plugins.xml](plugins.xml) 。插件由对应宿主加载，插件类名以声明 XML 为准。

使用方式：在宿主服务器的插件列表和参数块中配置导出的类；仅构建本包不会自动切换当前运行控制器或规划器。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
