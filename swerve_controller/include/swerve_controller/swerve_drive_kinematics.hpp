/**
 * @File    swerve_drive_kinematics.hpp
 * @Time    2026/02/24
 * @Author  OpenArmX
 * @Version 1.0
 * @Desc    四转四驱舵轮底盘运动学（逆运动学 + 正运动学 + 转向优化）
 */

#ifndef SWERVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
#define SWERVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_

#include <array>
#include <cstddef>

namespace swerve_controller
{

/// 底盘速度（机器人坐标系）
struct ChassisSpeeds
{
  double vx = 0.0;     // 前进速度 (m/s)
  double vy = 0.0;     // 左移速度 (m/s)
  double omega = 0.0;  // 逆时针角速度 (rad/s)
};

/// 单个舵轮模块状态
struct SwerveModuleState
{
  double speed = 0.0;        // 轮速 (m/s)
  double angle = 0.0;        // 转向角 (rad)
  double clamp_error = 0.0;  // 限位 clamp 造成的角度偏差 (rad)
};

/// 模块位置（相对于机器人中心）
struct ModulePosition
{
  double x = 0.0;  // 前方为正 (m)
  double y = 0.0;  // 左方为正 (m)
};

/// 四转四驱舵轮运动学
class SwerveDriveKinematics
{
public:
  static constexpr size_t NUM_MODULES = 4;

  SwerveDriveKinematics() = default;

  /// 配置模块位置和转向限位
  void configure(
    const std::array<ModulePosition, NUM_MODULES> & positions,
    const std::array<double, NUM_MODULES> & steering_min,
    const std::array<double, NUM_MODULES> & steering_max);

  /// 逆运动学: 底盘速度 → 4 个模块状态
  std::array<SwerveModuleState, NUM_MODULES> toModuleStates(
    const ChassisSpeeds & speeds,
    const std::array<double, NUM_MODULES> & current_angles) const;

  /// 正运动学: 4 个模块状态 → 底盘速度（Moore-Penrose 伪逆）
  ChassisSpeeds toChassisSpeeds(
    const std::array<SwerveModuleState, NUM_MODULES> & states) const;

  /// 速度去饱和: 按比例缩放使最大轮速不超限
  static void desaturateWheelSpeeds(
    std::array<SwerveModuleState, NUM_MODULES> & states,
    double max_speed);

private:
  /// 转向优化: 最小化旋转角度（>90° 时反转速度 + 翻转角度 180°）
  SwerveModuleState optimize(
    const SwerveModuleState & desired,
    double current_angle,
    double min_angle,
    double max_angle) const;

  /// 计算正运动学矩阵（伪逆）
  void computeForwardKinematicsMatrix();

  bool configured_ = false;

  // 模块位置
  std::array<ModulePosition, NUM_MODULES> positions_;

  // 转向限位
  std::array<double, NUM_MODULES> steering_min_;
  std::array<double, NUM_MODULES> steering_max_;

  // 逆运动学矩阵 8×3 (行优先)
  std::array<double, 8 * 3> ik_matrix_ = {};

  // 正运动学矩阵 3×8 (行优先, Moore-Penrose 伪逆)
  std::array<double, 3 * 8> fk_matrix_ = {};
};

}  // namespace swerve_controller

#endif  // SWERVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
