# swerve_controller

English | [中文](./README-CN.md)

---

ros2_control controller plugin for the 4WS4WD swerve drive chassis (kinematics + odometry).

## Description

This package implements a `controller_interface::ControllerInterface` plugin that:

- Subscribes to velocity commands (`cmd_vel`)
- Computes inverse kinematics to produce steering angle and wheel speed commands for 4 swerve modules
- Computes forward kinematics and integrates odometry
- Publishes odometry and (optionally) the `odom -> base_link` TF transform

The plugin is loaded as `swerve_controller/SwerveDriveController`.

## Key Features

- Full 4WS4WD inverse/forward kinematics with Moore-Penrose pseudo-inverse
- Steering angle optimization (>90 deg flip with speed reversal)
- Wheel speed desaturation (proportional scaling)
- Wheel speed slew rate limiting (acceleration limit)
- Steering synchronization: per-wheel drive scaling based on steering error
- Global steering alignment threshold before driving
- Configurable deadbands for low-speed steering and module speed
- cmd_vel timeout safety (stops driving if no command received)
- Debug state publishing for tuning

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` (configurable) | `geometry_msgs/msg/Twist` | Target chassis velocity (vx, vy, omega) |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time (if available) |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/swerve_drive_controller/odom` | `nav_msgs/msg/Odometry` | Wheel odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | odom -> base_link transform (if `enable_odom_tf: true`) |
| `/swerve_drive_controller/debug_states` | `std_msgs/msg/Float64MultiArray` | Debug output for tuning |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | `0.075` | Wheel radius (m) |
| `fl_pos_x/y`, `fr_pos_x/y`, etc. | from YAML | Module positions relative to center |
| `steering_joint_names` | `[fl/fr/bl/br]_steering_joint` | Steering joint names |
| `wheel_joint_names` | `[fl/fr/bl/br]_wheel_joint` | Wheel joint names |
| `odom_frame_id` | `odom` | Odometry frame |
| `base_frame_id` | `base_link` | Robot base frame |
| `cmd_vel_topic` | `cmd_vel` | Velocity command topic |
| `cmd_vel_timeout` | `0.5` | Command timeout (s) |
| `publish_rate` | `50.0` | Odometry publish rate (Hz) |
| `max_wheel_speed` | `2.0` | Maximum wheel speed (m/s) |
| `wheel_accel_limit` | `1.0` | Wheel speed change rate limit (m/s^2) |
| `enable_odom_tf` | `true` | Whether to publish odom TF |
| `steering_align_threshold` | `0.087` | Steering alignment precision (rad) |
| `steering_stop_threshold` | `1.22` | Error threshold to stop individual wheel (rad) |

## Build

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select swerve_controller
source install/setup.bash
```

## Dependencies

- `controller_interface`
- `geometry_msgs`
- `hardware_interface`
- `nav_msgs`
- `pluginlib`
- `rclcpp` / `rclcpp_lifecycle`
- `realtime_tools`
- `tf2` / `tf2_msgs`
- `std_msgs`

## Notes

- This plugin is not launched directly; it is spawned by the `controller_manager`.
- In navigation mode, `enable_odom_tf` is typically set to `false` (TF published by FAST-LIO2 or other localization).
- In mapping mode, `enable_odom_tf` is set to `true`.

## License

Apache-2.0
