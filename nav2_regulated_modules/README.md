# nav2_regulated_modules

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py
```

遥控订阅 `/downstream/chassis_control`，固定路径使用 `/navigation_service`，半自主单次 Lattice 规划使用 `/navigate_to_pose`。两个 Action 服务端后台待命，协调器负责抢占、取消、确认停稳及底盘输出。启动前提、消息示例和成本地图检查见 [工作空间说明](../README.md) 。

## 固定路径与终点行为

- [NavigationService](../byd_custom_msgs/action/NavigationService.action) 接收 `task_id` 和 [NaviSegment[]](../byd_custom_msgs/msg/NaviSegment.msg) 。支持直线与三次贝塞尔；相邻段连续，同一 Action 所有段同方向。
- `max_speed` 为有限正数，所有段取最小值，再受 `fixed_path_max_speed=1.5` 限制。方向 1／2 分别生成前进／倒车车体姿态和正／负速度，不修改初始定位或 Gazebo 位姿。
- 起点总距离上限 1.2 m；首次横向误差决定直接跟踪或严格对齐策略。横向误差较小时仍需满足航向条件才进入直线跟踪，大航向误差可能先原地转向；收到 Action 不代表立即达速。
- 每条路径起步保护距离默认 0.5 m，速度上限 0.3 m/s；横向和航向误差满足条件并稳定后解除，当前 YAML 的释放航向门限为 20°、稳定周期 10。
- 当前项目 YAML 的停车入口 0.008 m 与目标检查器位置容差 0.01 m 分开；位置／终点平面及停稳参与终点判断，终点航向作诊断。内部成功不等于独立地图精度或实车精度通过。

启动与 Action 几何例子见[仿真入口说明](../myworld_bringup/README.md) ；控制实现见 [fixed_path_controller.cpp](src/fixed_path_controller.cpp) 与 [fixed_path_goal_checker.cpp](src/fixed_path_goal_checker.cpp) 。

## collision 状态与 Collision Monitor

碰撞检测状态链与速度门独立。`use_collision_monitor=true` 启动动态区域和激光状态翻译器，输出 `/collision_state`（UInt8）及 `/obstacle_status_int`（Int32）；翻译器不发布 `/cmd_vel`。固定路径链按状态执行减速停车。`use_collision_velocity_gate=true` 才把 Collision Monitor 插入速度链。

| 项目 | 直接规控入口 | `myworld_bringup/entry.launch.py` |
|---|---|---|
| `enable_collision_state` | 默认 true，fixed_path／autonomous_fixed 生效 | 相同 |
| `use_collision_monitor` | 检测状态链，默认 false | 检测状态链，默认 false |
| `use_collision_velocity_gate` | 速度门，默认 false | 速度门，默认 false |
| `use_obstacle_status` | 默认 true，总开关开启时启动翻译器 | 相同 |
| `use_car_op_status_sim` | 默认 true；演示状态源，外部发布者出现时让位 | 相同 |
| `use_collision_visualization` | 默认 true，导航模式 | 仅 fixed_path 默认 true，其他模式 false |

`enable_collision_state` 由 Launch 转成三节点的 `collision_state_enabled`。项目 YAML 为 true；通用 C++ 节点独立运行的内置回退仍为 false，不能混写这两层默认值。自主和遥控模式不启用外部状态门控。

旧命令若用 `use_collision_monitor:=true` 开启速度过滤，现在应改用 `use_collision_velocity_gate:=true`。仅启用状态链时保持速度门 false；两个开关均 false 时保留原速度路由。整车配置显式使用 `enable_stamped_cmd_vel=false`，与 `controlpub` 的裸 Twist 输入匹配。

```bash
# 实车：状态检测链；真实上层持续提供 /car_op_status，关闭演示状态源。
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  use_sim_time:=false \
  use_collision_monitor:=true use_collision_velocity_gate:=false \
  use_car_op_status_sim:=false scan_topic:=/c200_lidar_node/scan

# 仿真：使用 /scan、sim 尺寸档和默认演示状态源。
ros2 launch myworld_bringup entry.launch.py \
  use_collision_monitor:=true \
  use_collision_velocity_gate:=false
```

实车雷达话题须按实际部署设置。状态链仍需有效雷达、TF 和 `/motion_state`；生产环境由上层发布正确的 `/car_op_status`，不能把默认状态 0 当作自动判断方向／载货。当前本地 autonomous_fixed 的 footprint 保持 1.2×0.9 m、膨胀半径 0.4 m；检测脚本 real 档车体为 1.2×0.8 m，自车点滤除默认外扩 0.05 m。车体尺寸、导航安全外廓及滤除范围用途不同，部署前分别核对。完整说明见[碰撞状态链](../nav2_collision_monitor/doc/OBSTACLE_STATUS.md) 。

| 有效状态 | 含义 | 相对任务速度上限 |
|---|---|---:|
| 4 | 正常 | 100% |
| 3 | 一级减速 | 65% |
| 2 | 二级减速 | 35% |
| 1、0 | 停止 | 0 |

启动、configure 或 reset 后从未收到消息时为 4，保留 `received=false`、`age=-1`、`reason=initial`，不因没有首条消息而超时。首次收到后按实际值处理，非法值或接收间隔达到 0.5 s 按 0 平滑停车；超时使用单调时钟。订阅为 BEST_EFFORT／VOLATILE／KeepLast(1)。

状态 4 只解除限制，不创建任务或速度。停止期间暂停对应无进展计时；恢复时仅在原任务及速度输入仍有效时继续，不重建路径或恢复已取消任务。门控在 S 曲线目标生成前，要求 `jerk_limited_smoothing=true`；参数修改需要重启或生命周期重新配置。

已有有效任务时，可在同环境终端持续发布状态；下面命令可能使车辆按原任务运动：

```bash
ros2 topic pub --rate 50 /collision_state std_msgs/msg/UInt8 '{data: 4}'
```

周期例子见 [COLLISION_TEST.md](tools/COLLISION_TEST.md) ：50 Hz 发布 `4→3→2→1→0`，默认保持 4／3／3／3／3 秒；请求 1.5 m/s，支持去程前进、返程倒车。有限运行示例（先核对地图端点与起步位姿）：

```bash
ros2 run nav2_regulated_modules collision_action_test.py \
  --schedule periodic --return-direction backward --max-legs 4 \
  --output-dir "$PWD/collision-runs/$(date +%Y%m%d-%H%M%S)"
```

该周期工具退出时会发布停止并取消自身任务，不能用它的 Ctrl+C 代替“独立状态源断流、原 Action 保持”的验收。配置 50 Hz 与实测发布调用频率、独立订阅接收频率应分别报告。

## 日志、工具与证据

统一速度诊断默认开启：自主模式文件快照 50 Hz、固定路径／remote 100 Hz，终端摘要 1 Hz；默认日志目录 `/tmp/nav2_logs`，可通过 `log_dir` 改到数据盘。`ROS_LOG_DIR` 管理 ROS Launch 日志，两者独立。`enable_legacy_velocity_logs=false` 默认抑制重复摘要，关闭统一诊断后回退旧记录。

速度快照仍在首次实际收到 ChassisControl 后才开始，导航任务执行本身不触发这个门控。fixed_path 中发一帧零速也会请求遥控接管并取消旧路径，不能在任务执行中仅为触发日志而随意发布该消息；TF 诊断和任务事件日志采用各自的原开关。

- [速度日志](doc/velocity_logging.md) ：字段、异步滚动、S 曲线及缺失／过期状态。
- [固定路径 TF 诊断](doc/tf_diagnostics.md) ：map→base_link 默认 100 Hz 采样打印、直接边逐条接收打印、控制器消费记录及逐边／时钟摘要。
- [固定路径架构](doc/fixed_path_architecture.md) ：既有模块与控制流程分析，其中旧的 Collision Monitor 默认开启描述以本 README 的当前默认表为准。
- [分开验收](doc/action_only_acceptance.md) ：速度停稳与定位采样质量分开评价。
- [测试工具](tools/README.md) ：工具运行前提及会修改环境的实验边界。

本文为当前源码静态说明。本轮文档更新不执行构建或运动。已有隔离真实节点＋合成反馈回归不等同于实车验收；现有短路线周期测试只取得部分减速／停止证据，独立状态源启停验收因起步转向占满 5 秒基准窗而 UNAVAILABLE，不能声称两程全部通过。动态验证记录由外层项目 ROADMAP 与报告维护。
