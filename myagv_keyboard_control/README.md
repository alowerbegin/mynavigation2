# myagv_keyboard_control

键盘节点向 `/downstream/chassis_control` 发布 `byd_custom_msgs/msg/ChassisControl` 目标，由统一导航器进行控制权交接、加减速及 jerk 规划，最终由导航器唯一发布 `/control_to_uart`。本节点不再直接发布 ControlRes，也不在本地重复平滑。

## 启动

两个终端均加载同一工作空间，使用相同 ROS_DOMAIN_ID／DDS 设置。

```bash
source /opt/ros/humble/setup.bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source install/local_setup.bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py
```

在另一个可交互终端加载同样环境后启动键盘：

```bash
ros2 launch myagv_keyboard_control keyboard_control.launch.py
```

键盘 Launch 自动传入当前终端设备，可用 `input_device:=/dev/pts/实际编号` 覆盖；自定义配置使用 `params_file:=/绝对路径/keyboard_control.yaml`。旧 `operation_mode:=remote` 不再启用键盘。主入口不自动启动键盘，也不自动提供定位或实车反馈。

## 按键及停车

| 按键 | op | 目标 |
|---|---:|---|
| W／↑ | 0 | 前进，线速度幅值为正 |
| S／↓ | 1 | 后退，线速度幅值仍为正 |
| A／← | 2 | 左转，角速度幅值为正 |
| D／→ | 3 | 右转，角速度幅值仍为正 |
| X／空格 | 当前合法 op | 三帧零速度请求，随后停止发布 |
| Q | 当前合法 op | 三帧零速度请求后退出 |

启动时、完成停车请求后保持静默，不持续占用遥控。方向键有效期间按100 Hz重复发送目标；超时按与X相同的方式请求停车。终端无法直接识别松键，最后一次方向键后超过0.5秒才判定超时。换向和轴间切换沿用下游现有行为，不在键盘侧另加规划。

零速请求交给下游平滑减速，Q不代表机械急停。Ctrl+C、进程异常或通信断流依靠下游0.15秒指令超时触发停车；该时间不包含实际减速过程。结束遥控后，下游在指令过期且实测停稳后释放控制权，旧导航任务不自动恢复。

## 键盘参数

默认配置见 [keyboard_control.yaml](config/keyboard_control.yaml) ，参数在启动时读取，修改后重启节点。

| 参数 | 默认值 | 作用 |
|---|---|---|
| input_device | /dev/tty | Launch自动覆盖为当前交互终端 |
| output_topic | /downstream/chassis_control | ChassisControl目标输入话题 |
| publish_rate | 100.0 | 目标发送频率，Hz |
| linear_speed／max_linear_speed | 0.3／0.3 | 目标线速度／键盘限幅，m/s |
| angular_speed／max_angular_speed | 0.3／0.3 | 目标角速度／键盘限幅，rad/s |
| command_timeout | 0.5 | 按键有效时间，s；0关闭按键超时 |

实际目标取速度参数与键盘限幅的较小值。消息 `acceleration=0`，加减速度和jerk统一使用导航器参数。原键盘 `*_accel_limit`、`*_decel_limit`、`*_accel_jerk_limit`、`*_decel_jerk_limit` 已移除，不能再用这些名称调速。

## 下游加减速配置

修改 [regulated_modules.yaml](../nav2_regulated_modules/params/regulated_modules.yaml) 中 `regulated_navigator.ros__parameters` 的以下参数，重新构建安装并重启统一规控入口。键盘的 params_file 只配置键盘节点，不能替代导航器配置。

| 约束 | 线速度轴参数／当前值 | 角速度轴参数／当前值 |
|---|---|---|
| 默认加速度 | chassis_default_linear_accel_max／1.5 m/s² | chassis_default_angular_accel_max／1.0 rad/s² |
| 加速度硬上限 | chassis_linear_accel_max／2.0 m/s² | chassis_angular_accel_max／1.5 rad/s² |
| 减速度上限 | chassis_linear_decel_max／2.0 m/s² | chassis_angular_decel_max／1.5 rad/s² |
| 加速jerk | chassis_linear_accel_jerk_max／3.0 m/s³ | chassis_angular_accel_jerk_max／3.0 rad/s³ |
| 减速jerk | chassis_linear_decel_jerk_max／3.0 m/s³ | chassis_angular_decel_jerk_max／3.0 rad/s³ |

本轮保持这些值与下游S曲线算法不变。键盘始终发送acceleration=0，不涉及非零加速度请求的生效行为。

导航器需Active且收到新鲜 `/motion_state`，默认反馈超时0.2秒。`/control_to_uart` 应只有导航器一个发布者；不要同时启动旧键盘直控、controlpub等直接底盘输出进程。

## 手动核对

```bash
ros2 topic info /downstream/chassis_control --verbose
ros2 topic info /control_to_uart --verbose
ros2 topic echo /downstream/chassis_control
ros2 topic echo /control_to_uart
```

检查启动静默、四方向、重复按键、X／空格／Q、输入超时和Action接管。最终速度的平滑及实际停稳需要同时观察底盘反馈；编译和静态检查不能证明实车效果。
