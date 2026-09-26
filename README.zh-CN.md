# 运控层

[English](./README.md) | 中文

---

本层负责把上层 `/cmd_vel` 底盘速度命令转换成四个舵轮模块的转向角和轮速，并发布底盘里程计。

## 包清单

- `swerve_controller`: ros2_control 控制器插件，包含四转四驱舵轮逆/正运动学、转向角优化、轮速限幅、同步减速和二阶中点法里程计。

## 输入输出

- 输入：`geometry_msgs/Twist`，默认 `/cmd_vel`。
- 输出：转向关节位置命令、驱动轮速度命令。
- 输出：`/odom` 里程计和可选 `odom -> base_link` TF。
- 调试：`/swerve_drive_controller/debug_states`。

## 职责边界

运控层只处理底盘级运动控制，不负责导航路径规划、地图定位、传感器驱动或硬件协议细节。

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
