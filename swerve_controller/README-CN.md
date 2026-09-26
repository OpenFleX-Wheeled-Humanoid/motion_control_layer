# swerve_controller

[English](./README.md) | 中文

---

四转四驱舵轮底盘 ros2_control 控制器插件（运动学 + 里程计）。

## 简介

本包实现了一个 `controller_interface::ControllerInterface` 插件，功能如下：

- 订阅速度指令（`cmd_vel`）
- 计算逆运动学，输出 4 个舵轮模块的转向角度和驱动轮速指令
- 计算正运动学并积分里程计
- 发布里程计及（可选的）`odom -> base_link` TF 变换

插件以 `swerve_controller/SwerveDriveController` 名称加载。

## 主要功能

- 完整 4WS4WD 逆/正运动学（Moore-Penrose 伪逆）
- 转向角优化（>90° 时翻转角度并反转速度）
- 轮速去饱和（按比例缩放）
- 轮速变化率限制（加速度限制）
- 转向同步：根据转向误差按比例缩放各轮驱动
- 全车对齐阈值，未对齐前暂停驱动
- 低速转向和模块速度的可配置死区
- cmd_vel 超时安全保护（无指令时停止驱动）
- 调试状态发布，便于调参

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `cmd_vel`（可配置） | `geometry_msgs/msg/Twist` | 底盘目标速度（vx、vy、omega） |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间（如有） |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/swerve_drive_controller/odom` | `nav_msgs/msg/Odometry` | 轮式里程计 |
| `/tf` | `tf2_msgs/msg/TFMessage` | odom -> base_link 变换（`enable_odom_tf: true` 时） |
| `/swerve_drive_controller/debug_states` | `std_msgs/msg/Float64MultiArray` | 调试输出，用于调参 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wheel_radius` | `0.075` | 车轮半径（m） |
| `fl_pos_x/y`、`fr_pos_x/y` 等 | 由 YAML 注入 | 模块相对中心位置 |
| `steering_joint_names` | `[fl/fr/bl/br]_steering_joint` | 转向关节名 |
| `wheel_joint_names` | `[fl/fr/bl/br]_wheel_joint` | 驱动关节名 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `base_frame_id` | `base_link` | 机器人底座坐标系 |
| `cmd_vel_topic` | `cmd_vel` | 速度指令话题 |
| `cmd_vel_timeout` | `0.5` | 指令超时时间（s） |
| `publish_rate` | `50.0` | 里程计发布频率（Hz） |
| `max_wheel_speed` | `2.0` | 最大轮速（m/s） |
| `wheel_accel_limit` | `1.0` | 轮速变化率上限（m/s^2） |
| `enable_odom_tf` | `true` | 是否发布 odom TF |
| `steering_align_threshold` | `0.087` | 转向对齐精度（rad） |
| `steering_stop_threshold` | `1.22` | 单轮停止驱动的误差阈值（rad） |

## 编译

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select swerve_controller
source install/setup.bash
```

## 依赖

- `controller_interface`
- `geometry_msgs`
- `hardware_interface`
- `nav_msgs`
- `pluginlib`
- `rclcpp` / `rclcpp_lifecycle`
- `realtime_tools`
- `tf2` / `tf2_msgs`
- `std_msgs`

## 说明

- 本插件不直接启动，由 `controller_manager` 生成。
- 导航模式下，`enable_odom_tf` 通常为 `false`（TF 由 FAST-LIO2 等定位模块发布）。
- 建图模式下，`enable_odom_tf` 设为 `true`。

## 许可证

Apache-2.0
