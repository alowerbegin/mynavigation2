# nav_2d_utils

## 本地源码接口（2026-11-16）

提供二维导航的几何、消息转换、TF 和里程计工具；MotionState／Odometry 反馈适配要与当前消费者配置匹配，不能只凭话题名称相似就认为消息类型相同。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- 实现定位：[conversions.cpp](src/conversions.cpp) 、[path_ops.cpp](src/path_ops.cpp) 、[tf_help.cpp](src/tf_help.cpp) 。

## 当前项目用法

提供二维导航消息转换、路径处理和 TF 辅助函数。

包类型：工具库。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[src/](src/) 、[include/](include/) 、[test/](test/) 。

直接依赖（清单节选，完整约束见 manifest）：`geometry_msgs`、`nav2_common`、`nav2_msgs`、`nav2_util`、`nav_2d_msgs`、`nav_msgs`、`std_msgs`、`tf2`、`tf2_geometry_msgs`。

使用方式：由依赖方通过构建依赖、头文件或 Python 导入使用，不提供通用的独立导航启动命令。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
