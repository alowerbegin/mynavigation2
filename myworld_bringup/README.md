# myworld_bringup

```bash
ros2 launch myworld_bringup entry.launch.py
```

场景启动 Gazebo、传感器桥接、底盘反馈和统一规控。外部定位提供 `map → base_link`。遥控、固定路径和单次 Lattice 半自主导航共用同一套实车 Action 接口。启动顺序、地图、综合 RViz 及可用话题见 [工作空间说明](../README.md) 。

## 仅启动仿真和传感器

使用 [simulation_only.launch.py](launch/simulation_only.launch.py) 加载 `myworld2/world_only.sdf` 仓库环境和差速车模型；默认打开 Gazebo 和 RViz，并通过静态地图服务发布对应的占据栅格，不启动规划、控制、Cartographer 或 AMCL：

```bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch myworld_bringup simulation_only.launch.py
```

Gazebo 可通过 `headless:=true` 在后台运行；不需要 RViz 时可传入 `use_rviz:=false`。该入口提供以下仿真接口：

| 接口 | 类型与用途 |
|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan`，雷达扫描，坐标系 `base_scan` |
| `/scan/points` | `sensor_msgs/msg/PointCloud2`，同一仿真雷达的点云 |
| `/imu`、`/joint_states` | 仿真 IMU 和左右轮关节状态 |
| `/odometry` | Gazebo 原始 `nav_msgs/msg/Odometry`，保留为转换输入 |
| `/motion_state` | `byd_custom_msgs/msg/MotionState`，与实车一致的速度反馈；升降及货架字段为零 |
| `/map` | 来自 `models/myworld2/myworld2.yaml` 的静态占据栅格 |
| `/cmd_vel` | 可选的 `geometry_msgs/msg/Twist` 速度输入，由 Gazebo 差速底盘接收 |

默认桥接 Gazebo 的 `odom → base_footprint`，并发布仿真用的单位变换 `map → odom`；机器人模型继续发布 `base_footprint → base_link → base_scan` 等内部 TF。静态地图由 Map Server 加载、启动文件管理其生命周期，不执行定位或建图。默认 RViz 使用 [simulation_only.rviz](rviz/simulation_only.rviz) 显示地图、车体和 `/scan`；点云 `/scan/points` 可在显示列表中启用。默认 Fixed Frame 为 `map`；关闭地图时改为 `odom`，关闭仿真里程计 TF 时改为 `base_link`，也可用 `rviz_fixed_frame` 指定其他已有坐标系。关闭仿真里程计 TF 后，地图和车体之间还需要外部定位 TF 才能同时显示。

默认 `use_sim_time:=false`，桥接消息使用系统时间；设置 `use_sim_time:=true` 时桥接 `/clock` 并使用 Gazebo 时间，其他消费节点须使用相同时间基准。`world_file` 和 `map` 可分别覆盖世界及地图文件，二者应使用一致的世界坐标；`publish_map:=false` 关闭内置地图及 `map → odom`，`ign_partition` 可覆盖默认的独立 Gazebo 分区。当前车辆 SDF 没有相机传感器，保留的相机桥接接口不会自行产生图像。

若外部定位直接发布 `map → base_link`，可设置 `publish_odom_tf:=false`，关闭 Gazebo 里程计 TF 并采用 `base_link` 为模型根节点；外部系统提供 `/map` 时另设 `publish_map:=false`，同时检查外部模型不要重复发布同名内部 TF。独立仿真与 `entry.launch.py` 不应在同一 ROS 域重复运行，以免多个发布者使用相同话题和坐标系。

## 固定路径任务

`/navigation_service` 类型为 `byd_custom_msgs/action/NavigationService`。直线与贝塞尔的 `max_speed` 均为有限正数；同任务所有段方向一致，1 前进、2 倒车，速度符号由方向决定。任务不重置机器人位姿。

起点距离、航向与起步保护可能导致先转向或低速接近，不能假定 Action 接受后立即达到 1.5 m/s。当前 fixed_path 使用专用目标检查器，终点航向只作诊断；停车入口、目标容差与独立定位精度分开评价。具体参数以传入 YAML 与 Launch 重写为准，见[规控包](../nav2_regulated_modules/README.md) 。

当前地图的参数化直线命令见[规控包固定路径 Action](../nav2_regulated_modules/README.md#固定路径-action仅-fixed_path) 。以下保留历史几何例子，**坐标来自历史地图**，发送前须核对并替换当前地图、通行区域、定位和起点，不作为当前出生位姿或默认路线：

```bash
ros2 action send_goal --feedback /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: 'myworld_fixed_path_001', navi_segment: [{segment_type: 1, segment_name: 'diagonal_corridor', segment_id: 'segment_001', node1: {x: -2.8, y: -1.7, z: 0.0}, node2: {x: 1.91, y: -4.83, z: 0.0}, control_pos1: {x: -0.933333, y: -2.933333, z: 0.0}, control_pos2: {x: 0.933333, y: -4.166667, z: 0.0}, max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0}]}"
```

### 三段连续路径：直线—曲线—直线

下面的独立 Action 沿用上例的仿真起点 `(-2.8, -1.7)`、终点 `(1.91, -4.83)`、
`0.20 m/s` 限速与后退方向。第一段直线在 `(-1.4, -2.4)` 接入三次贝塞尔曲线，
曲线在 `(0.5, -3.8)` 接入末段直线；相邻段端点相同，两个曲线控制点分别沿
前后直线的切线方向布置，避免连接处出现几何折角。先启动上述 `fixed_path` 模式，
确认机器人仍在首段 `node1` 附近且相关 Lifecycle 节点已进入 `active`，再在另一终端发送：

```bash
ros2 action send_goal --feedback \
  /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: 'myworld_line_curve_line_001', navi_segment: [
    {segment_type: 1, segment_name: 'line_start', segment_id: 'segment_001',
     node1: {x: -2.8, y: -1.7, z: 0.0}, node2: {x: -1.4, y: -2.4, z: 0.0},
     control_pos1: {x: 0.0, y: 0.0, z: 0.0}, control_pos2: {x: 0.0, y: 0.0, z: 0.0},
     max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0},
    {segment_type: 2, segment_name: 'bezier_middle', segment_id: 'segment_002',
     node1: {x: -1.4, y: -2.4, z: 0.0}, node2: {x: 0.5, y: -3.8, z: 0.0},
     control_pos1: {x: -0.7, y: -2.75, z: 0.0}, control_pos2: {x: -0.205, y: -3.285, z: 0.0},
     max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0},
    {segment_type: 1, segment_name: 'line_end', segment_id: 'segment_003',
     node1: {x: 0.5, y: -3.8, z: 0.0}, node2: {x: 1.91, y: -4.83, z: 0.0},
     control_pos1: {x: 0.0, y: 0.0, z: 0.0}, control_pos2: {x: 0.0, y: 0.0, z: 0.0},
     max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0}
  ]}"
```

`segment_type: 1` 的控制点由路径生成代码根据直线起终点重新计算，因此这里填零；
`segment_type: 2` 按 `node1 → control_pos1 → control_pos2 → node2` 生成曲线。
该几何来自旧实验，不对应当前默认地图和出生位置；仅在核对地图与起始位姿后使用，不能通过重启假定机器人会回到这些坐标。
此命令仅提供协议与几何连续的输入示例，未据此验证地图通行或最终到点精度。
