# dwb_msgs

## 本地源码接口（2026-11-16）

这是 DWB 评估／轨迹消息与服务定义。消息存在不构成 DWB 已被当前启动模式选中的证明；业务固定路径接口位于 byd_custom_msgs。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- 接口目录：[msg/](msg)（4 项）、[srv/](srv)（5 项） 。消息字段以定义文件为准。

## 当前项目用法

定义 DWB 轨迹、评分与局部规划评估相关消息和服务。

包类型：接口包。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[msg/](msg/) 、[srv/](srv/) 。

直接依赖（清单节选，完整约束见 manifest）：`builtin_interfaces`、`geometry_msgs`、`nav_2d_msgs`、`nav_msgs`、`rosidl_default_runtime`、`std_msgs`。

### 接口文件

- [CriticScore.msg](msg/CriticScore.msg) ：`dwb_msgs/msg/CriticScore`。
- [LocalPlanEvaluation.msg](msg/LocalPlanEvaluation.msg) ：`dwb_msgs/msg/LocalPlanEvaluation`。
- [Trajectory2D.msg](msg/Trajectory2D.msg) ：`dwb_msgs/msg/Trajectory2D`。
- [TrajectoryScore.msg](msg/TrajectoryScore.msg) ：`dwb_msgs/msg/TrajectoryScore`。
- [DebugLocalPlan.srv](srv/DebugLocalPlan.srv) ：`dwb_msgs/srv/DebugLocalPlan`。
- [GenerateTrajectory.srv](srv/GenerateTrajectory.srv) ：`dwb_msgs/srv/GenerateTrajectory`。
- [GenerateTwists.srv](srv/GenerateTwists.srv) ：`dwb_msgs/srv/GenerateTwists`。
- [GetCriticScore.srv](srv/GetCriticScore.srv) ：`dwb_msgs/srv/GetCriticScore`。
- [ScoreTrajectory.srv](srv/ScoreTrajectory.srv) ：`dwb_msgs/srv/ScoreTrajectory`。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
