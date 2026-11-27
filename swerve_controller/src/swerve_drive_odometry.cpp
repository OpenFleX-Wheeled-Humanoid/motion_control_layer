/**
 * @File    swerve_drive_odometry.cpp
 * @Time    2026/02/24
 * @Author  OpenArmX
 * @Version 1.0
 * @Desc    舵轮底盘里程计实现（二阶中点法积分）
 */

#include "swerve_controller/swerve_drive_odometry.hpp"

#include <cmath>

namespace swerve_controller
{

void SwerveDriveOdometry::setKinematics(const SwerveDriveKinematics * kinematics)
{
  kinematics_ = kinematics;
}

void SwerveDriveOdometry::reset()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  linear_x_ = 0.0;
  linear_y_ = 0.0;
  angular_z_ = 0.0;
}

void SwerveDriveOdometry::update(
  const std::array<SwerveModuleState, SwerveDriveKinematics::NUM_MODULES> & states,
  double dt)
{
  if (!kinematics_ || dt <= 0.0) {
    return;
  }

  // 正运动学: 模块状态 → 底盘速度
  ChassisSpeeds speeds = kinematics_->toChassisSpeeds(states);

  // 速度死区：过滤电机编码器静止时的微小噪声，防止里程计漂移
  constexpr double kLinearDeadband  = 0.005;  // m/s
  constexpr double kAngularDeadband = 0.01;   // rad/s
  if (std::abs(speeds.vx)    < kLinearDeadband)  speeds.vx    = 0.0;
  if (std::abs(speeds.vy)    < kLinearDeadband)  speeds.vy    = 0.0;
  if (std::abs(speeds.omega) < kAngularDeadband) speeds.omega = 0.0;

  linear_x_ = speeds.vx;
  linear_y_ = speeds.vy;
  angular_z_ = speeds.omega;

  // 二阶中点法积分
  double delta_heading = speeds.omega * dt;
  double mid_heading = heading_ + delta_heading / 2.0;

  double cos_mid = std::cos(mid_heading);
  double sin_mid = std::sin(mid_heading);

  x_ += (speeds.vx * cos_mid - speeds.vy * sin_mid) * dt;
  y_ += (speeds.vx * sin_mid + speeds.vy * cos_mid) * dt;
  heading_ += delta_heading;

  // 归一化航向角到 [-π, π]
  while (heading_ > M_PI) heading_ -= 2.0 * M_PI;
  while (heading_ < -M_PI) heading_ += 2.0 * M_PI;
}

}  // namespace swerve_controller
