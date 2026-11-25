# mynavigation2

本目录是 nav2_demo 的统一控制实验副本源码，位于 `nav2change_ws/src/navigation2/`，基于 ROS 2 Humble。默认入口同时提供后台遥控、外部固定路径和单次 Lattice 半自主导航，包含标准 Nav2、自定义底盘接口和仿真场景。

## 入口选择

| 用途 | 入口 | 所需环境 |
|---|---|---|
| 实车统一规控 | `ros2 launch nav2_regulated_modules regulated_modules.launch.py` | 外部定位、传感器、运动反馈和底盘驱动 |
| 综合 RViz | `ros2 launch nav2_regulated_modules unified_rviz.launch.py` | 与规控相同 ROS 域、DDS 和系统时间 |
| 当前场景仿真 | `ros2 launch myworld_bringup entry.launch.py` | Gazebo、桥接及统一规控，定位另行启动 |
| TurtleBot3 仿真 | [mytb3_bringup](mytb3_bringup/README.md) | 独立场景与时钟配置 |
| 另一套 myworld 仿真 | [myworld_bringup_sim](myworld_bringup_sim/README.md) | 兼容场景 |
| 键盘直接控制底盘 | [myagv_keyboard_control](myagv_keyboard_control/README.md) | 交互终端直接输出 ControlRes |

两个入口均启动遥控、固定路径与半自主 Lattice。导航服务后台待命，收到对应 Action Goal 后开始执行。

## 当前源码契约（2026-11-24）

| 对外入口 | 类型 | 行为 |
|---|---|---|
| `/downstream/chassis_control` | `byd_custom_msgs/msg/ChassisControl` | 后台遥控，优先接管导航 |
| `/navigation_service` | `byd_custom_msgs/action/NavigationService` | 跟踪业务分段路径 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 一次 Lattice 规划，冻结后分段跟踪 |
| `/control_to_uart` | `byd_custom_msgs/msg/ControlRes` | 唯一底盘指令出口 |

- 两类导航共用任务管理器，每次执行一个任务。新的合法 Goal 可以请求抢占，交接等待旧下游终态和新鲜停稳反馈，交接期间的其他请求可能被拒绝。
- 有效遥控取消当前导航，等待下游结束和停稳后接管。遥控断流后减速停车、释放控制权，继续导航需重新发送 Goal。
- 固定路径和半自主单点导航分别通过各自的 Action 接收任务。
- 统一入口强制选择 SmacLattice，覆盖共享 YAML 的 `autonomous_fixed_profile.planner: hybrid`。暂停恢复继续使用原冻结路径。
- `use_collision_monitor` 控制状态检测链，`use_collision_velocity_gate` 控制最终速度门，两者默认 false。`enable_collision_state` 默认 true，控制下游消费状态。
- 统一入口使用独立进程承载双执行通道，当前 `use_composition` 对此分支不起作用。

实现见 [启动装配](nav2_regulated_modules/launch/regulated_modules.launch.py) 、[任务管理](nav2_regulated_modules/src/navigator_unified.cpp) 和 [底盘仲裁](nav2_regulated_modules/src/chassis_control_authority.cpp) 。

## 安装依赖

以下命令适用于 Ubuntu 22.04／ROS 2 Humble，执行前需已安装 ROS 2 Humble 并配置 ROS 2 官方 apt 软件源。

安装 Nav2 运行依赖：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

本仓库的 TEB Local Planner 还依赖 g2o；全量构建包含该包时先安装：

```bash
sudo apt install ros-humble-libg2o
```

碰撞监视器的激光／点云半径离群点过滤还使用 PCL 的 `common`、`filters` 模块，依赖已在 [nav2_collision_monitor/package.xml](nav2_collision_monitor/package.xml) 和 [CMakeLists.txt](nav2_collision_monitor/CMakeLists.txt) 声明；按下节通过 rosdep 安装对应依赖。

源码其余依赖按下节的 `rosdep install` 命令安装，依赖声明见各包的 `package.xml`。仿真另需 Ignition Gazebo 与 ros_gz 组件，入口说明见 [myworld_bringup](myworld_bringup/README.md) 和 [TurtleBot3 仿真](navigation2_ignition_gazebo_turtlebot3/README.md) 。完成源码构建后，按下节加载本工作空间的 `install/local_setup.bash`。

## 目录与构建

源码含 54 个 ROS 包；53 个由当前 colcon 发现，nav2_system_tests 因 COLCON_IGNORE 默认跳过。嵌套 DWB 子包和 turtlebot3 按各自 package.xml 的名称构建，不能用仓库目录名代替包名。

命令从外层 nav2_demo 项目根目录开始；已在 nav2change_ws 时跳过 cd。不在本源码仓库或包目录中运行 colcon，也不同时叠加 learningnav2_ws 环境。

```bash
cd /mnt/nvme/codex/nav2_demo/nav2change_ws
source /opt/ros/humble/setup.bash
# 首次准备依赖；需要已配置 ROS 2 / rosdep，实际安装由使用者执行。
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

仅构建规控及其工作空间依赖：

```bash
# 在 nav2change_ws，先加载 /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to nav2_regulated_modules \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

依赖基线为 Ubuntu 22.04／ROS 2 Humble；仿真另需 Ignition Gazebo 与 ros_gz 组件，TEB 另依赖 g2o。具体依赖看各包 manifest 和构建声明。构建、定向测试和运行验证的证据见后文验证记录。构建临时文件和 ROS 日志可通过 TMPDIR、ROS_LOG_DIR 指向可写数据盘，避免系统盘不足。

## 默认实车启动与综合 RViz

各终端加载同一工作空间，使用相同 ROS 域、DDS 和系统时间。底盘驱动、激光与定位按实际部署独立启动。

```bash
source /opt/ros/humble/setup.bash
source /mnt/nvme/codex/nav2_demo/nav2change_ws/install/local_setup.bash
ros2 pkg prefix nav2_regulated_modules
ros2 launch nav2_regulated_modules regulated_modules.launch.py
```

包前缀应指向 `nav2change_ws/install/nav2_regulated_modules`。默认系统时间，地图 `/userdata/map/pbstream/out.yaml`，参数为包内 `params/regulated_modules.yaml`，内置 RViz 默认关闭。其他地图用 `map:=实际路径` 指定。

另一个同环境终端运行综合视图。

```bash
ros2 launch nav2_regulated_modules unified_rviz.launch.py
```

配置与配套入口见 [unified.rviz](nav2_regulated_modules/rviz/unified.rviz) 和 [unified_rviz.launch.py](nav2_regulated_modules/launch/unified_rviz.launch.py) 。直接导入配置可适配显示话题，面板还需配套启动文件提供生命周期服务重映射。

面板 Navigation 对应协调节点管理器，Localization 标签对应地图服务管理器。Pause／Reset 操作这两个组，两条执行通道有独立管理器。通用面板保留多点模式按钮，统一入口使用单点 Nav2 Goal Action。

### 输入与准入条件

| 输入或状态 | 默认要求 |
|---|---|
| `map → base_link` | 外部定位提供可查询的动态 TF，半自主执行阶段检查时效 |
| `/motion_state` | MotionState，`v_car/w_car` 为有限数值，接收年龄 ≤0.2 s |
| 首次任务停稳 | 线速度与角速度绝对值均 <0.001，收到交接开始后的新反馈 |
| `/control_to_uart` | 唯一发布者为当前导航器 |
| 所选执行通道 | 下游 Action 可用，受监控节点 Active，生命周期响应在最近 1 s 内 |
| 地图和激光 | 地图加载完成，成本地图接收 `/c200_lidar_node/scan` 与传感器 TF |
| `/collision_state` | 首次未收到按 4，收到后超时或非法值按停车处理 |

`map2base_tf/map2baseTF` 周期发布固定位置 TF，只提供坐标变换，运动反馈和传感器输入需要另外准备。成本地图初始化和面板 Active 各有自己的状态，Goal 还要通过底盘准入检查。

### 双执行通道与话题

```text
/navigation_service → /fixed_path_exec/follow_path
  → /fixed_path_exec/controller_server → cmd_vel_nav
  → /fixed_path_exec/velocity_smoother → /fixed_path_exec/cmd_vel
                                                               ↘
                                                   控制权仲裁 → /control_to_uart
                                                               ↗
/navigate_to_pose → /lattice_exec/compute_path_to_pose → 冻结并分段
  → /lattice_exec/follow_path → /lattice_exec/controller_server → cmd_vel_nav
  → /lattice_exec/velocity_smoother → /lattice_exec/cmd_vel
```

图中 cmd_vel_nav 分别位于对应执行通道命名空间。启用速度门后，每个通道的平滑输出先经过该通道 Collision Monitor，再进入仲裁。后台遥控接入仲裁。仿真由 `control_res_to_twist` 把统一出口转换为 Gazebo 使用的 `/cmd_vel`。

| 显示内容 | 统一模式话题 |
|---|---|
| 地图 | `/map` |
| 半自主全局成本地图 | `/lattice_exec/global_costmap/costmap` |
| 半自主局部成本地图 | `/lattice_exec/local_costmap/costmap` |
| 固定路径局部成本地图 | `/fixed_path_exec/local_costmap/costmap` |
| 足迹与增量 | 对应成本地图命名空间下的 `published_footprint`、`costmap_updates` |
| 固定路径 | `/fixed_path_plan` |
| 半自主冻结路径 | `/autonomous_fixed/plan` |

原根目录 `/controller_server`、`/global_costmap`、`/local_costmap` 的监控脚本需要适配上述执行通道前缀。

## Gazebo 仿真

加载工作空间后运行场景，默认启动 unified、Gazebo、桥接和规控。

```bash
ros2 launch myworld_bringup entry.launch.py
```

外部定位在其实际安装环境中独立启动，本机既有路径如下。

```bash
source /home/forlinx/lidar_slam/colcon_ws/install/setup.bash
ros2 launch cartographer_ros cartographer.launch.py
```

各终端使用相同 ROS 域、DDS 和时间设置，定位 pbstream 与导航地图保持对应。场景已包含规控，运行前检查现有后台。场景默认 `headless=true`、`use_rviz=true`。使用本页综合视图时，可用 `ros2 launch myworld_bringup entry.launch.py use_rviz:=false` 启动场景，再单独启动 `unified_rviz.launch.py`。

## 隔离本机虚拟定位验证

以下合成输入用于独立本机测试。所有测试终端先加载 nav2change_ws 环境，再使用同一隔离域。

```bash
export ROS_DOMAIN_ID=219
export ROS_LOCALHOST_ONLY=1
```

启动固定虚拟定位。坐标来自本机测试地图的自由区域，换地图后重新检查位置与车辆足迹通行性。

```bash
ros2 run map2base_tf map2baseTF --ros-args -p x:=10.314983 -p y:=-3.075084 -p yaw:=0.0 -p publish_rate:=100.0
```

另一个同环境、同隔离域终端持续发布静止运动反馈。

```bash
ros2 topic pub -r 50 /motion_state byd_custom_msgs/msg/MotionState '{header: auto, v_car: 0.0, w_car: 0.0}'
```

模拟反馈持续更新 Header 时间戳，底盘准入按本机接收时刻校验新鲜度。完整初始化测试还需要激光输入，可以在同隔离域用下面的测试程序提供空激光和通行状态。

```bash
python3 - <<'FIXTURE'
import math
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8

rclpy.init()
node = rclpy.create_node('local_sensor_inputs')
scan = node.create_publisher(LaserScan, '/c200_lidar_node/scan', 10)
collision = node.create_publisher(UInt8, '/collision_state', 10)

def publish():
    laser = LaserScan()
    laser.header.stamp = node.get_clock().now().to_msg()
    laser.header.frame_id = 'base_link'
    laser.angle_min = -math.pi
    laser.angle_max = math.pi
    laser.angle_increment = math.pi / 180
    laser.range_min = 0.1
    laser.range_max = 12.0
    laser.scan_time = 0.05
    laser.ranges = [float('inf')] * 361
    scan.publish(laser)
    collision.publish(UInt8(data=4))

timer = node.create_timer(0.05, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
FIXTURE
```

再分别运行默认规控和综合 RViz。等待执行节点 Active、三张成本地图收数后发送 Goal。虚拟位置保持固定，本步骤验证初始化、目标接入与取消，运动到点由 Gazebo 或实际底盘另行验证。退出模拟输入程序后再次测试，需要重新准备这些输入。

## 底盘 ChassisControl 命令行发布

话题为 `/downstream/chassis_control`，类型为 [`byd_custom_msgs/msg/ChassisControl`](byd_custom_msgs/msg/ChassisControl.msg) 。在新终端加载同一工作空间，并与接收端使用相同的 ROS_DOMAIN_ID／DDS 设置：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source install/local_setup.bash
ros2 topic info /downstream/chassis_control --verbose
```

当前统一入口的 `regulated_navigator` 接收该话题，遥控优先接管正在执行的固定路径或半自主任务，等待旧下游终态与实测停稳后输出遥控目标。接收条件包括节点 active 和新鲜 `/motion_state`，`/control_to_uart` 应只有导航器一个发布者。

| `op` | 操作 | 使用的速度字段 |
|---|---|---|
| `0`（OP_FORWARD） | 前进 | `linear_velocity`，单位 m/s |
| `1`（OP_BACKWARD） | 后退 | `linear_velocity`，单位 m/s |
| `2`（OP_TURN_LEFT） | 原地左转 | `angular_velocity`，单位 rad/s |
| `3`（OP_TURN_RIGHT） | 原地右转 | `angular_velocity`，单位 rad/s |

速度和 `acceleration` 均填写有限、非负幅值，正负方向由 `op` 决定；前进／后退会将角速度置零，左转／右转会将线速度置零。`acceleration` 是对应线／角轴的加速度请求，填 0 使用接收端默认值，实际输出还受限速及 S 曲线约束。下面每次只运行一条运动命令，换方向前先 `Ctrl+C` 停止原发布进程。

```bash
# 前进：0.1 m/s，50 Hz 持续发送。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 0, linear_velocity: 0.1, angular_velocity: 0.0, acceleration: 0.3}'
```

```bash
# 后退：幅值仍为正数，op=1 决定倒车。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 1, linear_velocity: 0.1, angular_velocity: 0.0, acceleration: 0.3}'
```

```bash
# 原地左转：0.2 rad/s。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 2, linear_velocity: 0.0, angular_velocity: 0.2, acceleration: 0.3}'
```

```bash
# 原地右转：幅值仍为正数，op=3 决定右转。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 3, linear_velocity: 0.0, angular_velocity: 0.2, acceleration: 0.3}'
```

停止原运动发布进程后，用下面命令以 50 Hz 连续发送 20 帧零目标，然后观察 `/motion_state` 的 `v_car/w_car` 确认停稳；没有单独的 STOP 操作码：

```bash
ros2 topic pub --rate 50 --times 20 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 0, linear_velocity: 0.0, angular_velocity: 0.0, acceleration: 0.3}'
```

只需发送一帧零速消息以检查消息接收或触发速度诊断时，可用：

```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 0, linear_velocity: 0.0, angular_velocity: 0.0, acceleration: 0.0}'
```

`ros2 topic pub` 默认只有 1 Hz，而导航器底盘支路默认 `chassis_command_timeout=0.15` s，持续运动应显式使用上面的 50 Hz。超时会触发接收端停车处理；`Ctrl+C` 只结束发布进程，不会自动补发零速。固定路径交还导航前须结束全部遥控发布者，再等遥控超时减速与实测停稳后下发新路径；单帧或持续零速也会触发接管，不能仅把它当成无副作用的诊断开关。示例 Header 仅设置 `frame_id`，时间戳使用消息默认零值；当前两个消费者按本地接收时间计算命令超时。底盘实际执行与停稳通过运动反馈核对。

## 固定路径 Action 示例

以下数字取自 2026-11-25 本机后台仿真的实际请求，坐标保留六位小数，使用 `map` 坐标系。先结束上面的遥控发布进程，等待控制权释放与 `/motion_state` 停稳，再发送 Action。固定路径示例对应车辆位于起点附近、车头朝向约 −2.04835 rad 的状态。

| 项目 | 实际数值 |
|---|---|
| 起点 | x＝9.004632 m，y＝−4.608506 m |
| 终点 | x＝8.751848 m，y＝−5.096974 m |
| 路段长度 | 约 0.55 m |
| 方向与速度 | `motion_direction=1`，前进 0.15 m/s |

```bash
ros2 action send_goal --feedback /navigation_service byd_custom_msgs/action/NavigationService \
'{
  task_id: fixed_line_demo,
  navi_segment: [{
    segment_type: 1,
    segment_id: line_01,
    motion_direction: 1,
    max_speed: 0.15,
    node1: {x: 9.004632, y: -4.608506, z: 0.0},
    node2: {x: 8.751848, y: -5.096974, z: 0.0}
  }]
}'
```

`segment_type=1` 表示直线，路径点的车头航向由路径切线与运动方向生成。收到 `SUCCEEDED` 且结果 `finish=true` 后，再通过运动反馈核对停稳。消息字段见 [NavigationService.action](byd_custom_msgs/action/NavigationService.action) 和 [NaviSegment.msg](byd_custom_msgs/msg/NaviSegment.msg) 。

## 半自主 Lattice Action 示例

下面的目标取自同次仿真的半自主到点用例，可在上述固定路径完成并停稳后发送。实测请求时车辆约位于 `(8.754544, -5.091547)`，目标在该车头朝向的后方约 0.55 m。Lattice 根据地图搜索含倒车能力的路径，保留目标车头航向。

| 项目 | 实际数值 |
|---|---|
| 目标位置 | x＝9.007079 m，y＝−4.602951 m，z＝0 m |
| 目标航向 | yaw≈−2.04784 rad |
| 目标四元数 | x＝0，y＝0，z＝−0.854152901，w＝0.520021943 |
| Action | `/navigate_to_pose`，`nav2_msgs/action/NavigateToPose` |

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 9.007079, y: -4.602951, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: -0.854152901, w: 0.520021943}
    }
  },
  behavior_tree: ""
}'
```

此 Goal 提供目标位姿，速度采用当前控制器与平滑器配置，`behavior_tree` 保持为空。协调器请求一次 Lattice 规划，冻结结果后按分段路径跟踪；完整路径可在 `/autonomous_fixed/plan` 观察。使用其他地图或其他车辆起点时，需要据实际定位和可通行区域调整这些坐标。

### 结果判断与取消任务

- `Goal accepted with ID: ...` 仅表示目标被接收，保存该 UUID 以便精确取消。`Goal finished with status: SUCCEEDED` 才表示任务成功；`ABORTED` 表示失败，`CANCELED` 表示取消。
- 反馈包括 `current_pose`、`distance_remaining`、`navigation_time` 和 `number_of_recoveries`。自主剩余距离取下游路径估计与当前请求目标直线距离的较大值，避免最近路径点已到末点时提前归零；这仍不是独立物理真值。规划或换路径后首次控制反馈尚未到达时，使用目标直线距离作为下界。
- `estimated_time_remaining` 包含平移、终点转向及默认 0.5 秒停稳的估计，原地转向或低速时不直接置零；估计值不参与成功判定。Duration 要同时看 `sec` 和 `nanosec`，`nanosec: 0` 也可以表示合法的整秒。
- 自主 `number_of_recoveries` 为控制恢复与定位丢失后恢复执行次数之和，受字段范围限制时饱和显示；控制恢复的次数上限仍单独生效。
- 默认场景的最终位置／航向／速度需满足配置阈值并连续稳定 0.5 秒；Action 成功与独立实车精度验收分开评价。失败原因可查看 `/tmp/nav2_logs/nav2_regulated_modules/regulated_navigator.log`，若设置了 `log_dir` 则使用对应目录。
- 本机已核对的 Humble `ros2 action send_goal` 在目标被接收后支持原终端 `Ctrl+C` 请求取消该目标。等待取消结果，并确认 `/motion_state` 的 `v_car/w_car` 停稳；关闭终端或客户端断开不能作为停车确认。

也可在另一终端按 UUID 取消单点任务。将 `GOAL_UUID` 替换成发送命令打印的 32 位目标 ID；转换成功后才发送取消请求，时间戳保持零，仅匹配该 UUID。

```bash
GOAL_UUID='替换为本轮目标的32位UUID'
GOAL_UUID_BYTES=$(python3 -c 'import sys, uuid; u = uuid.UUID(sys.argv[1]); assert u.int != 0, "目标 UUID 不能全为零"; print(list(u.bytes))' "$GOAL_UUID") && \
ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: $GOAL_UUID_BYTES}, stamp: {sec: 0, nanosec: 0}}}"
```


## 固定路径、collision 与测试

NavigationService 支持直线／三次贝塞尔；各段同向，方向 1 前进、2 倒车，max_speed 始终为正。任务限速取各段最小值，再受 fixed_path_max_speed=1.5 限制。起步航向和速度保护可能先转向或低速跟踪；Action 成功与独立速度／定位验收分别记录。

固定路径直接使用业务分段生成的 Path，不调用全局规划或路径平滑 Action，也不自动搜索绕障路线。`motion_direction=2` 时按路径切线反向编码车头姿态，并以负线速度执行；`max_load_speed`、`dwell_time` 当前未在这条固定路径执行链中实现对应的限速／停留调度，不能仅凭消息字段存在就认为已生效。

当前直接规控参数的起步上限为 `start_offset_speed_limit=0.30 m/s`，仍受更低任务限速约束。保护距离按沿路径投影进度计算：至少从首次有效投影位置再前进 `start_speed_limit_distance=0.50 m`，并且横向／运动方向航向误差连续 10 个周期达标后才解除；这不是车辆到起点的直线距离。起步保护在同一路径解除后不重复开启。

终点参数与成功条件按当前源码分开理解：

- 直接规控默认 `goal_stop_entry_tolerance=0.008 m`；进入停车入口或满足末段越过终点平面条件后，控制器锁存零速并停止继续转向。位置精度仍按 `xy_goal_tolerance=0.01 m` 检查，实际轮速由下游执行链决定。
- `terminal_convergence_enabled=true` 时，优先要求当前位置精度、定位／速度反馈时效及停稳条件连续通过 1 秒复核；已锁停且新鲜反馈持续停稳 2 秒、定位仍新鲜时，也可经兜底返回成功并记录 WARN。因此必须同时检查误差、超限标志和复核日志。
- 直接规控默认终点减速度 `0.20 m/s²`、末段接近速度 `0.015 m/s`，动态制动裕量为低速 `0.010 m`／高速 `0.050 m`，过渡速度 `0.50 m/s`。仿真包装有独立参数与覆盖逻辑，不能把两套配置的调参结果直接混用。
- 直接入口 YAML 的终点基础前视为 `goal_terminal_lookahead_dist=0.30 m`，参考速度 `0.30 m/s`，速度增益 `0.20 s`，距离限制为 `0.20～0.40 m`。场景 YAML 与通用 C++ 回退仍分别为 `0.20 m`、`0.75 m/s`、`0.10 s`、`0.15～0.25 m`；不能用回退值替代直接入口配置。正常前视到末段前视的混合区间为距终点 `0.50～0.10 m`。
- 剩余距离已使用相邻有效线段的连续投影弧长，避免最近采样点切换时的计距台阶；实现见 [fixed_path_progress.hpp](nav2_regulated_modules/include/nav2_regulated_modules/detail/fixed_path_progress.hpp) 。`selectCarrot()` 仍从最近离散路径点累计长度选点，路径不足前视距离时沿终点切线外推虚拟跟踪点；该选点过程没有一并改为段内连续插值，不改变业务终点。

实现入口见 [固定路径控制器](nav2_regulated_modules/src/fixed_path_controller.cpp) 、[末端前视混合](nav2_regulated_modules/include/nav2_regulated_modules/detail/terminal_lookahead.hpp) 和 [Controller Server 到点复核](nav2_controller/src/controller_server.cpp) 。

固定路径和半自主通道默认消费外部 collision 状态。状态 4／3／2 分别为任务速度上限的 100%／65%／35%，1／0 停止；首次从未收到消息按 4，收到后非法值或断流达到 0.5 秒停车。恢复 4 只解除限制，不创建任务或恢复已取消任务。后台遥控使用独立的原遥控 S 曲线，不经过导航碰撞速度门；交接／反馈失效使用保护性零速，不能据正常导航的平滑限制承诺该保护阶段的制动舒适性。

直接规控和 myworld 入口的 `use_collision_monitor`（检测状态链）、`use_collision_velocity_gate`（速度门）均默认 false，二者独立；区域可视化本身不产生碰撞状态。启用检测链后，翻译器从雷达和动态区域生成 `/collision_state`，unified／fixed_path／autonomous_fixed 可按状态减速停车。Launch 的 enable_collision_state 默认 true 与独立 C++ 节点 collision_state_enabled 的回退 false 属于不同配置层。旧速度门命令须改用 `use_collision_velocity_gate:=true`；用法及雷达参数见[碰撞状态链说明](nav2_regulated_modules/README.md#collision-状态与-collision-monitor) 。

- [规控完整说明](nav2_regulated_modules/README.md) ：模式、反馈、默认配置与门控状态表。
- [50 Hz collision Action 例子](nav2_regulated_modules/tools/COLLISION_TEST.md) ：周期 4→3→2→1→0、1.5 m/s、往返倒车及结果分类。
- [测试工具](nav2_regulated_modules/tools/README.md) ：现有环境观察与隔离仿真实验的区别。
- [分开验收](nav2_regulated_modules/doc/action_only_acceptance.md) ：速度停稳、定位覆盖与精度独立评价。
- [固定路径架构](nav2_regulated_modules/doc/fixed_path_architecture.md) ：既有模块职责与流程；其中旧的 Collision Monitor 默认开启描述以本文当前配置为准。

`collision_action_test.py` 已区分实车与仿真：两者默认均使用 `/motion_state`／MotionState；`--runtime sim` 默认巡航触发、往返均前进并持续运行，`--runtime real` 默认周期调度、去程前进返程倒车、四程。实车必须显式提供 `--a X Y --b X Y`，工具不替现场选择路线。仿真显式选其他末级名称的反馈话题时保留 Odometry 兼容，`--odom-topic` 只供仿真使用且与 `--feedback-topic` 互斥。

`--schedule periodic` 选择独立周期调度，发布调用目标频率为 50 Hz；五态保持时间默认仿真为 4／3／3／3／3 秒，实车为 12／4／4／3／3 秒，可用 `--state-durations` 覆盖。实际时序、各方向／状态响应和同一 Action 停后恢复分别记录，未覆盖的观察项标为 UNAVAILABLE。

截至本次文档核对，隔离真实节点配合合成反馈的测试、当前地图仿真测试、实车验证是不同证据层级。周期仿真仅取得部分减速／停止有效证据；独立状态源启停验收因 5 秒起步窗内一直转向而 UNAVAILABLE，断流停车和同任务恢复的完整两程验收仍未完成。历史原始报告由外层项目维护，本文不替代报告。

## 日志与排查

运行日志默认在 `/tmp/nav2_logs`，可用 log_dir 指向数据盘，ROS Launch 日志由 ROS_LOG_DIR 管理。统一入口当前跳过独立 velocity_diagnostics 节点，导航器 TfDiagnostics 仅在显式 fixed_path 分支创建，因此 enable_velocity_diagnostics=true 仍需结合实际启动节点判断诊断覆盖。

```bash
ros2 lifecycle get /regulated_navigator
ros2 lifecycle get /map_server
ros2 lifecycle get /fixed_path_exec/controller_server
ros2 lifecycle get /fixed_path_exec/velocity_smoother
ros2 lifecycle get /lattice_exec/controller_server
ros2 lifecycle get /lattice_exec/velocity_smoother
ros2 lifecycle get /lattice_exec/planner_server
ros2 topic info /control_to_uart --verbose
ros2 topic info /motion_state --verbose
ros2 topic hz /motion_state
ros2 run tf2_ros tf2_echo map base_link
```


### 单点 Goal 被拒绝

当前 handlePoseGoal 把统一准入失败、节点未激活、目标数据格式无效和非空 behavior_tree 合并打印为“拒绝单点目标：节点未激活、位姿无效或请求了行为树 XML”。按以下条件排查。

1. 协调节点和所选执行通道 Active，生命周期服务及时响应。
2. `/control_to_uart` 只有一个发布者，当前无遥控占用或待交接任务。
3. `/motion_state` 持续更新且满足新鲜度和停稳条件，虚拟 TF 需配合测试运动反馈。
4. `map → base_link` 可查询，目标坐标系、坐标和四元数有效，behavior_tree 留空。

validPose() 检查数据结构和数值，地图上的起终点碰撞在后续规划阶段判断。面板 Active 与成本地图初始化后，任务仍需通过上述准入检查。

### 已有验证与边界

- 副本 53 包构建通过，四项定向 CTest 通过。见 [补齐剩余包的构建日志](../../log/resume-build.log) 和 [定向回归](../../log/final-regression.log) 。
- Gazebo 覆盖遥控、固定路径前进／倒车／取消、半自主单次规划到点和遥控打断两类任务。一次半自主到点用例平面误差约 4.3 mm。见 [仿真记录](../../log/simulation3/acceptance.log) 。
- 综合 RViz 的独立域 219 测试使用本机地图、map2baseTF、静止反馈、空激光和碰撞状态 4。七节点 Active、三成本地图收数，鼠标 Goal 接受并生成冻结路径，取消返回 CANCELED。见 [检查结果](../../log/unified-rviz-local/report.json) 、[初始化截图](../../log/unified-rviz-local/initialized.png) 和 [箭头截图](../../log/unified-rviz-local/arrow.png) 。
- 实车运动效果及两类导航执行中直接互相抢占的端到端验证仍待完成。实车归档曾检测到额外底盘出口发布者，部署时需现场核对。上述日志和图片位于工作空间本地 log，Git 克隆需要另行获取验收产物。

## 全部包索引

以下按 package.xml 的包名列出，嵌套目录保持真实结构。“colcon 可发现”只描述包发现，不表示当前启动图会使用它。当前实车／myworld 仿真链以启动节点、服务器实际配置的插件列表及必要公共库为范围；AMCL、BT Navigator、Route、Waypoint Follower、TEB 和独立旧场景不因参数文件中留有配置块就自动接入。

| 包 | 类型与职责 | 构建状态 |
|---|---|---|
| [aws_robomaker_small_warehouse_world](aws_robomaker_small_warehouse_world/README.md) | 资源包：提供 AWS 小型仓库世界、模型与地图资源，用于仿真场景搭建；不实现导航控制器。 | colcon 可发现 |
| [byd_custom_msgs](byd_custom_msgs/README.md) | 接口包：定义底盘命令、运动反馈、分段路径和固定路径 Action，是规控与底盘之间的消息契约。 | colcon 可发现 |
| [controlpub](controlpub/README.md) | 适配节点：将 Twist 速度输入转换为 ControlRes，供外部底盘通信链路使用；本包不实现串口驱动。 | colcon 可发现 |
| [costmap_queue](nav2_dwb_controller/costmap_queue/README.md) | 算法库：提供代价地图栅格按距离展开的队列工具，服务于 DWB 距离场计算。 | colcon 可发现 |
| [dwb_core](nav2_dwb_controller/dwb_core/README.md) | 控制插件与核心库：实现 DWB 局部控制框架，组合轨迹生成器与评分器选择速度指令。 | colcon 可发现 |
| [dwb_critics](nav2_dwb_controller/dwb_critics/README.md) | 轨迹评分插件：提供障碍、路径、目标、朝向及振荡等评分器，由 DWB Controller 装载。 | colcon 可发现 |
| [dwb_msgs](nav2_dwb_controller/dwb_msgs/README.md) | 接口包：定义 DWB 轨迹、评分与局部规划评估相关消息和服务。 | colcon 可发现 |
| [dwb_plugins](nav2_dwb_controller/dwb_plugins/README.md) | 轨迹生成插件：提供 DWB 标准及受加速度约束的轨迹生成器，供 dwb_core 采样使用。 | colcon 可发现 |
| [laserpub](laserpub/README.md) | 测试节点：发布可配置的合成二维 LaserScan，供隔离功能验证使用；不能替代真实激光观测。 | colcon 可发现 |
| [map2base_tf](map2base_tf/README.md) | 测试节点：周期发布固定位置的 map→base_link 动态 TF，供隔离接口验证；不实现定位算法。 | colcon 可发现 |
| [myagv_keyboard_control](myagv_keyboard_control/README.md) | 遥控节点：读取终端按键，按速度、加速度和 jerk 约束生成 ControlRes 底盘命令。 | colcon 可发现 |
| [mytb3_bringup](mytb3_bringup/README.md) | 仿真启动包：组织 TurtleBot3 的 Ignition Gazebo、机器人描述、传感器桥接、定位与导航场景。 | colcon 可发现 |
| [myworld_bringup](myworld_bringup/README.md) | 仿真启动包：组织仓库场景仿真与统一规控装配；当前 entry 使用外部定位和系统时间适配。 | colcon 可发现 |
| [myworld_bringup_sim](myworld_bringup_sim/README.md) | 仿真启动包：保留另一套 myworld 场景启动、定位与导航配置；入口行为独立于 myworld_bringup。 | colcon 可发现 |
| [nav2_amcl](nav2_amcl/README.md) | 定位节点：使用粒子滤波、运动模型与激光地图匹配估计二维位姿，向导航提供定位变换。 | colcon 可发现 |
| [nav2_behavior_tree](nav2_behavior_tree/README.md) | 行为树插件库：提供规划、控制、恢复、条件与流程控制 BT 节点，供 BT Navigator 装载。 | colcon 可发现 |
| [nav2_behaviors](nav2_behaviors/README.md) | 行为服务器：通过行为插件提供旋转、后退、等待、按航向行驶及辅助遥控等动作。 | colcon 可发现 |
| [nav2_bringup](nav2_bringup/README.md) | 标准启动包：提供标准 Nav2 的导航、定位、组合节点和示例仿真启动资源。 | colcon 可发现 |
| [nav2_bt_navigator](nav2_bt_navigator/README.md) | 导航协调节点：执行行为树，将导航 Action 编排为规划、控制与恢复任务；与本项目无 BT 协调器是不同入口。 | colcon 可发现 |
| [nav2_collision_monitor](nav2_collision_monitor/README.md) | 速度过滤节点：依据 LaserScan、PointCloud2、Range 等传感器与配置区域，对输入速度实施停车、减速或接近保护。 | colcon 可发现 |
| [nav2_common](nav2_common/README.md) | 构建与启动工具库：提供 Nav2 通用 CMake 宏与 Python Launch 参数重写工具，包括 RewrittenYaml。 | colcon 可发现 |
| [nav2_constrained_smoother](nav2_constrained_smoother/README.md) | 平滑插件：通过 Ceres 优化路径的平滑性、曲率和障碍物约束，由 Smoother Server 加载。 | colcon 可发现 |
| [nav2_controller](nav2_controller/README.md) | 控制服务器：实现 FollowPath Action，加载控制器、进度检查器和目标检查器，计算并发布速度。 | colcon 可发现 |
| [nav2_core](nav2_core/README.md) | 插件接口库：定义 GlobalPlanner、Controller、Smoother、Behavior、GoalChecker 等 Nav2 插件基类。 | colcon 可发现 |
| [nav2_costmap_2d](nav2_costmap_2d/README.md) | 代价地图库与节点：组合静态、障碍、体素、膨胀等层及代价过滤器，为规划与控制提供环境代价。 | colcon 可发现 |
| [nav2_dwb_controller](nav2_dwb_controller/nav2_dwb_controller/README.md) | 元包：聚合 DWB 控制器相关包的依赖，不提供独立运行节点。 | colcon 可发现 |
| [nav2_graceful_controller](nav2_graceful_controller/README.md) | 控制插件：通过平滑控制律跟踪路径，由 Controller Server 加载并生成速度。 | colcon 可发现 |
| [nav2_lifecycle_manager](nav2_lifecycle_manager/README.md) | 生命周期管理节点：按配置顺序启动、激活、停用和监视生命周期节点，并通过 bond 检测连接状态。 | colcon 可发现 |
| [nav2_map_server](nav2_map_server/README.md) | 地图服务节点：提供地图加载、发布、保存及 Costmap Filter 信息服务。 | colcon 可发现 |
| [nav2_mppi_controller](nav2_mppi_controller/README.md) | 控制插件：通过模型预测路径积分采样控制序列，并按 critics 评分选择路径跟踪动作。 | colcon 可发现 |
| [nav2_msgs](nav2_msgs/README.md) | 接口包：定义标准 Nav2 导航、规划、路径跟踪、平滑、行为及生命周期相关消息、服务和 Action。 | colcon 可发现 |
| [nav2_navfn_planner](nav2_navfn_planner/README.md) | 规划插件：提供栅格 Dijkstra／A* 路径搜索，由 Planner Server 加载。 | colcon 可发现 |
| [nav2_planner](nav2_planner/README.md) | 规划服务器：实现 ComputePathToPose／ComputePathThroughPoses Action，并管理全局规划插件与全局代价地图。 | colcon 可发现 |
| [nav2_regulated_modules](nav2_regulated_modules/README.md) | 规控协调与插件包：提供双 Action 和后台遥控入口、无 BT 任务协调、固定路径插件和诊断工具。 | colcon 可发现 |
| [nav2_regulated_pure_pursuit_controller](nav2_regulated_pure_pursuit_controller/README.md) | 控制插件：提供受曲率、代价与目标接近约束的 Pure Pursuit 路径跟踪控制器。 | colcon 可发现 |
| [nav2_rotation_shim_controller](nav2_rotation_shim_controller/README.md) | 控制插件：在需要时先对齐路径朝向，再把控制交给配置的内部控制器。 | colcon 可发现 |
| [nav2_route](nav2_route/README.md) | 路网服务器与插件：从路网图搜索路线，并通过边评分和路线操作插件处理路线任务；不同于栅格 Planner Server。 | colcon 可发现 |
| [nav2_rviz_plugins](nav2_rviz_plugins/README.md) | RViz 插件：提供导航目标工具、Nav2 面板与粒子云显示，供 RViz 加载。 | colcon 可发现 |
| [nav2_simple_commander](nav2_simple_commander/README.md) | Python 客户端库：提供 BasicNavigator 及导航、路径跟踪、路点任务的 Python 示例与应用演示。 | colcon 可发现 |
| [nav2_smac_planner](nav2_smac_planner/README.md) | 规划插件：提供二维 A*、Hybrid-A* 和 State Lattice 搜索，支持不同车辆运动约束。 | colcon 可发现 |
| [nav2_smoother](nav2_smoother/README.md) | 平滑服务器与插件：实现 SmoothPath Action，加载简单平滑与 Savitzky–Golay 等路径平滑插件。 | colcon 可发现 |
| [nav2_system_tests](nav2_system_tests/README.md) | 系统测试资源：包含导航系统级测试脚本、场景和模型；当前 COLCON_IGNORE 使其不参加默认 colcon 构建。 | 默认忽略 |
| [nav2_theta_star_planner](nav2_theta_star_planner/README.md) | 规划插件：提供 Theta* 任意角路径搜索，通过栅格视线检查减少路径方向约束。 | colcon 可发现 |
| [nav2_util](nav2_util/README.md) | 公共运行库：提供生命周期节点、Action Server、TF、里程计、几何、速度及 collision 状态等公共工具。 | colcon 可发现 |
| [nav2_velocity_smoother](nav2_velocity_smoother/README.md) | 速度平滑节点：处理控制速度输入，执行开环／闭环平滑及本项目可选 jerk 限制与 collision 目标门控。 | colcon 可发现 |
| [nav2_voxel_grid](nav2_voxel_grid/README.md) | 体素算法库：提供按位压缩的三维体素标记、清除与射线追踪结构，服务于 VoxelLayer。 | colcon 可发现 |
| [nav2_waypoint_follower](nav2_waypoint_follower/README.md) | 路点服务器：把路点列表编排为导航任务，并在路点执行等待、拍照或输入触发等插件。 | colcon 可发现 |
| [nav_2d_msgs](nav2_dwb_controller/nav_2d_msgs/README.md) | 接口包：提供二维位姿、路径、速度等公共消息类型，供 DWB 系列使用。 | colcon 可发现 |
| [nav_2d_utils](nav2_dwb_controller/nav_2d_utils/README.md) | 工具库：提供二维导航消息转换、路径处理和 TF 辅助函数。 | colcon 可发现 |
| [navigation2](navigation2/README.md) | 元包：聚合 Nav2 栈依赖；源码仓库名称 navigation2 与本目录中的同名 ROS 元包应区分。 | colcon 可发现 |
| [spdlog_wrapper](spdlog_wrapper/README.md) | 日志库：封装 spdlog、YAML 配置与 ROS 日志转接，供规控与底盘适配等模块复用。 | colcon 可发现 |
| [teb_local_planner](teb_local_planner/README.md) | 控制插件：通过 Timed Elastic Band 优化轨迹时间、障碍距离与运动学约束，作为 Nav2 控制插件使用。 | colcon 可发现 |
| [teb_msgs](teb_msgs/README.md) | 接口包：定义 TEB 优化轨迹、反馈和障碍物消息，供 teb_local_planner 使用。 | colcon 可发现 |
| [turtlebot3](navigation2_ignition_gazebo_turtlebot3/turtlebot3/README.md) | 仿真资源包：安装 Ignition Gazebo TurtleBot3 的 Launch、地图、模型和参数；ROS 包名为 turtlebot3。 | colcon 可发现 |

## 外部贡献：Fork＋Pull Request

本仓库公开地址：

```text
https://github.com/zpy560/mynavigation2
```

普通外部贡献者不需要本仓库的写权限。推荐使用“Fork 到个人账号、在个人 Fork 开发、向本仓库
`main` 分支提交 Pull Request”的方式贡献代码。

### Fork仓库

贡献者登录 GitHub 后打开：

```text
https://github.com/zpy560/mynavigation2/fork
```

选择自己的个人账号并创建 Fork。完成后，贡献者会得到：

```text
https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2
```

其中 `CONTRIBUTOR_ACCOUNT` 需要替换为贡献者自己的 GitHub 用户名。

### 克隆个人Fork并添加上游仓库

```bash
git clone https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2.git
cd mynavigation2

git remote add upstream https://github.com/zpy560/mynavigation2.git
git remote -v
```

远端职责：

| 远端 | 仓库 | 用途 |
| --- | --- | --- |
| `origin` | 贡献者自己的 Fork | 推送贡献者的开发分支 |
| `upstream` | `zpy560/mynavigation2` | 获取本仓库最新代码 |

### 创建开发分支

不要直接在个人 Fork 的 `main` 分支开发。先创建能够表达改动目的的分支：

```bash
git switch -c fix/map-server-lifecycle
```

其他分支名示例：

```text
feat/keyboard-dev-tty
docs/update-bringup-guide
fix/dual-lidar-tf
```

### 修改、验证并提交

完成修改后，先检查变更范围和验证结果：

```bash
git status
git diff --check
git diff
```

只暂存本次贡献相关的文件：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 修复具体问题并说明影响范围"
```

推荐的提交前缀：

| 前缀 | 用途 |
| --- | --- |
| `feat:` | 新增功能 |
| `fix:` | 修复问题 |
| `docs:` | 更新文档 |
| `refactor:` | 不改变功能的结构调整 |
| `chore:` | 构建、依赖或工程维护 |

### 推送个人分支

```bash
git push -u origin fix/map-server-lifecycle
```

贡献者只向自己的 `origin` 推送，不需要也不应直接向 `zpy560/mynavigation2` 的 `main` 分支推送。

### 创建Pull Request

推送后，在 GitHub 页面点击 `Compare & pull request`，并确认目标关系：

```text
base repository: zpy560/mynavigation2
base branch:     main

head repository: CONTRIBUTOR_ACCOUNT/mynavigation2
compare branch:  fix/map-server-lifecycle
```

PR 描述至少应包含：

- 修改了什么。
- 为什么需要修改。
- 影响哪些包、节点、Topic、Action、Service 或参数。
- 使用了哪些验证命令。
- 哪些运行环境尚未验证。

提交后的 PR 会显示在：

```text
https://github.com/zpy560/mynavigation2/pulls
```

### 根据审查意见更新PR

如果维护者提出修改意见，贡献者继续在同一个开发分支修改并推送即可：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 根据审查意见修正具体问题"
git push
```

新的提交会自动追加到现有 PR，不需要重新创建 PR。

### 同步上游main分支

当 PR 开发期间上游 `main` 有新提交时，可以把上游更新合并到当前开发分支：

```bash
git fetch upstream
git switch fix/map-server-lifecycle
git merge upstream/main
git push
```

如果出现冲突，应在本地解决冲突、重新验证后再推送，不要在不了解影响时覆盖上游文件。

### 权限边界

- 外部贡献者可以 Fork 公开仓库并提交 PR。
- 外部贡献者默认不能直接推送本仓库，也不能自行合并 PR。
- 仓库维护者负责审查、要求修改、批准、关闭或合并 PR。
- 长期可信任的协作者可以单独授予仓库写权限，但普通外部贡献优先使用 Fork＋PR。
- PR 被合并前，变更不会进入本仓库的 `main` 分支。
