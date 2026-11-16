# nav2_rviz_plugins

## 本地源码接口（2026-11-16）

插件提供 Nav2 面板、目标工具和代价地图显示。Nav2 Goal 发送标准导航 Action，fixed_path 的业务路径仍经 NavigationService；独立 RViz 关闭不会自行取消外部客户端持有的任务。面板可显示不等于导航任务或精度验收成功。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- 插件导出：[plugins_description.xml](plugins_description.xml) ；共 3 个 class 声明，类型为 `nav2_rviz_plugins::GoalTool`、`nav2_rviz_plugins::Nav2Panel`、`nav2_rviz_plugins::ParticleCloudDisplay`。导出类与当前选中的插件 ID 分开核对。
- 实现定位：[goal_tool.cpp](src/goal_tool.cpp) 、[nav2_panel.cpp](src/nav2_panel.cpp) 、[flat_weighted_arrows_array.cpp](src/particle_cloud_display/flat_weighted_arrows_array.cpp) 。

## 当前项目用法

提供导航目标工具、Nav2 面板与粒子云显示，供 RViz 加载。

包类型：RViz 插件。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[src/](src/) 、[include/](include/) 。

直接依赖（清单节选，完整约束见 manifest）：`geometry_msgs`、`libqt5-core`、`libqt5-gui`、`libqt5-opengl`、`libqt5-widgets`、`nav2_lifecycle_manager`、`nav2_msgs`、`nav2_util`、`nav_msgs`、`pluginlib`、`qtbase5-dev`、`rclcpp` 等。

插件声明：[plugins_description.xml](plugins_description.xml) 。插件由对应宿主加载，插件类名以声明 XML 为准。

### 本项目接入说明

在 RViz 中装载 `nav2_rviz_plugins/GoalTool`、`nav2_rviz_plugins/Navigation 2` 或 `nav2_rviz_plugins/ParticleCloud`。目标工具发送导航请求，不能替代外部定位或生命周期激活。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
