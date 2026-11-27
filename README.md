# Motion Control Layer

English | [中文](./README.zh-CN.md)

---

This layer converts chassis velocity commands into steering angles and wheel speeds for the four swerve modules, and publishes wheel-based odometry.

## Package

- `swerve_controller`: ros2_control controller plugin with swerve inverse/forward kinematics, steering-angle optimization, wheel-speed limiting, synchronized slowdown, and second-order midpoint odometry.

## Interfaces

- Input: `geometry_msgs/Twist`, default topic `/cmd_vel`.
- Output: steering joint position commands and wheel joint velocity commands.
- Output: `/odom` and optional `odom -> base_link` TF.
- Debug: `/swerve_drive_controller/debug_states`.

The layer does not implement navigation planning, SLAM, sensor drivers, or hardware protocols.

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
