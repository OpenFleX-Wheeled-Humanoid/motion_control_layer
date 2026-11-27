/**
 * @File    swerve_drive_odometry.hpp
 * @Time    2026/02/24
 * @Author  OpenArmX
 * @Version 1.0
 * @Desc    舵轮底盘里程计（基于正运动学 + 中点法积分）
 */

#ifndef SWERVE_CONTROLLER__SWERVE_DRIVE_ODOMETRY_HPP_
#define SWERVE_CONTROLLER__SWERVE_DRIVE_ODOMETRY_HPP_

#include "swerve_controller/swerve_drive_kinematics.hpp"

namespace swerve_controller
{

/// 舵轮底盘里程计
class SwerveDriveOdometry
{
public:
  SwerveDriveOdometry() = default;

  /// 设置运动学指针
  void setKinematics(const SwerveDriveKinematics * kinematics);

  /// 重置里程计
  void reset();

  /// 更新里程计（二阶中点法积分）
  /// @param states  当前 4 个模块状态
  /// @param dt      时间步长 (s)
  void update(const std::array<SwerveModuleState, SwerveDriveKinematics::NUM_MODULES> & states,
              double dt);

  // 位姿获取
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }

  // 速度获取
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngularZ() const { return angular_z_; }

private:
  const SwerveDriveKinematics * kinematics_ = nullptr;

  // 位姿
  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;

  // 速度
  double linear_x_ = 0.0;
  double linear_y_ = 0.0;
  double angular_z_ = 0.0;
};

}  // namespace swerve_controller

#endif  // SWERVE_CONTROLLER__SWERVE_DRIVE_ODOMETRY_HPP_
