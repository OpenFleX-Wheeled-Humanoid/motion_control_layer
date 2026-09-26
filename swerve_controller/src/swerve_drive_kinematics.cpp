/**
 * @File    swerve_drive_kinematics.cpp
 * @Time    2026/02/24
 * @Author  OpenArmX
 * @Version 1.0
 * @Desc    四转四驱舵轮运动学实现（IK/FK/转向优化/速度去饱和）
 */

#include "swerve_controller/swerve_drive_kinematics.hpp"

#include <cmath>
#include <algorithm>

namespace swerve_controller
{

/// 角度归一化到 [-π, π]
static double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void SwerveDriveKinematics::configure(
  const std::array<ModulePosition, NUM_MODULES> & positions,
  const std::array<double, NUM_MODULES> & steering_min,
  const std::array<double, NUM_MODULES> & steering_max)
{
  positions_ = positions;
  steering_min_ = steering_min;
  steering_max_ = steering_max;

  // 构建逆运动学矩阵 (8×3)
  // 对于每个模块 i:
  //   vx_i = vx - omega * yi   → [1, 0, -yi]
  //   vy_i = vy + omega * xi   → [0, 1,  xi]
  for (size_t i = 0; i < NUM_MODULES; ++i) {
    double xi = positions_[i].x;
    double yi = positions_[i].y;

    ik_matrix_[(i * 2) * 3 + 0] = 1.0;
    ik_matrix_[(i * 2) * 3 + 1] = 0.0;
    ik_matrix_[(i * 2) * 3 + 2] = -yi;

    ik_matrix_[(i * 2 + 1) * 3 + 0] = 0.0;
    ik_matrix_[(i * 2 + 1) * 3 + 1] = 1.0;
    ik_matrix_[(i * 2 + 1) * 3 + 2] = xi;
  }

  computeForwardKinematicsMatrix();
  configured_ = true;
}

std::array<SwerveModuleState, SwerveDriveKinematics::NUM_MODULES>
SwerveDriveKinematics::toModuleStates(
  const ChassisSpeeds & speeds,
  const std::array<double, NUM_MODULES> & current_angles) const
{
  std::array<SwerveModuleState, NUM_MODULES> states;

  // 逆运动学: 计算每个模块的速度向量
  double chassis[3] = {speeds.vx, speeds.vy, speeds.omega};

  for (size_t i = 0; i < NUM_MODULES; ++i) {
    double vx_i = 0.0, vy_i = 0.0;
    for (int j = 0; j < 3; ++j) {
      vx_i += ik_matrix_[i * 2 * 3 + j] * chassis[j];
      vy_i += ik_matrix_[(i * 2 + 1) * 3 + j] * chassis[j];
    }

    // 转换为速度和角度
    double speed = std::hypot(vx_i, vy_i);

    if (speed < 1e-4) {
      // 速度接近零时保持当前转向角，不回零
      states[i].speed = 0.0;
      states[i].angle = current_angles[i];
    } else {
      double angle = std::atan2(vy_i, vx_i);
      states[i].speed = speed;
      states[i].angle = angle;

      // 转向优化: 最小化旋转角度
      states[i] = optimize(states[i], current_angles[i],
                           steering_min_[i], steering_max_[i]);
    }
  }

  return states;
}

ChassisSpeeds SwerveDriveKinematics::toChassisSpeeds(
  const std::array<SwerveModuleState, NUM_MODULES> & states) const
{
  // 将模块状态转换为速度向量
  double module_vels[8];
  for (size_t i = 0; i < NUM_MODULES; ++i) {
    module_vels[i * 2]     = states[i].speed * std::cos(states[i].angle);
    module_vels[i * 2 + 1] = states[i].speed * std::sin(states[i].angle);
  }

  // 正运动学: fk_matrix_ (3×8) × module_vels (8×1)
  ChassisSpeeds speeds;
  double result[3] = {0.0, 0.0, 0.0};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 8; ++c) {
      result[r] += fk_matrix_[r * 8 + c] * module_vels[c];
    }
  }
  speeds.vx = result[0];
  speeds.vy = result[1];
  speeds.omega = result[2];

  return speeds;
}

void SwerveDriveKinematics::desaturateWheelSpeeds(
  std::array<SwerveModuleState, NUM_MODULES> & states,
  double max_speed)
{
  double highest = 0.0;
  for (const auto & s : states) {
    highest = std::max(highest, std::abs(s.speed));
  }

  if (highest > max_speed && highest > 0.0) {
    double scale = max_speed / highest;
    for (auto & s : states) {
      s.speed *= scale;
    }
  }
}

SwerveModuleState SwerveDriveKinematics::optimize(
  const SwerveModuleState & desired,
  double current_angle,
  double min_angle,
  double max_angle) const
{
  // 候选1: 正向（保持原始角度和速度）
  double fwd_delta = normalizeAngle(desired.angle - current_angle);
  double fwd_angle = current_angle + fwd_delta;
  double fwd_speed = desired.speed;

  // 候选2: 反向（翻转180°，反转速度）
  double rev_target = normalizeAngle(desired.angle + M_PI);
  double rev_delta = normalizeAngle(rev_target - current_angle);
  double rev_angle = current_angle + rev_delta;
  double rev_speed = -desired.speed;

  // 对两个候选分别做限位钳制，计算 clamp 造成的角度偏差
  double fwd_clamped = std::clamp(fwd_angle, min_angle, max_angle);
  double fwd_error = std::abs(fwd_angle - fwd_clamped);

  double rev_clamped = std::clamp(rev_angle, min_angle, max_angle);
  double rev_error = std::abs(rev_angle - rev_clamped);

  // 选择：优先选旋转量小的那个，但如果它被 clamp 截断了很多，则选另一个
  // 综合考虑旋转量和 clamp 误差
  double fwd_cost = std::abs(fwd_delta) + fwd_error * 2.0;
  double rev_cost = std::abs(rev_delta) + rev_error * 2.0;

  SwerveModuleState result;
  double chosen_error;
  if (fwd_cost <= rev_cost) {
    result.angle = fwd_clamped;
    result.speed = fwd_speed;
    chosen_error = fwd_error;
  } else {
    result.angle = rev_clamped;
    result.speed = rev_speed;
    chosen_error = rev_error;
  }

  // 将 clamp 误差输出给调用方，由控制器层统一做同步衰减
  result.clamp_error = chosen_error;

  return result;
}

void SwerveDriveKinematics::computeForwardKinematicsMatrix()
{
  // Moore-Penrose 伪逆: (A^T * A)^{-1} * A^T
  // A = ik_matrix_ (8×3), A^T (3×8)
  // A^T * A (3×3), 然后求逆

  // 计算 A^T * A (3×3)
  double ata[9] = {};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double sum = 0.0;
      for (int k = 0; k < 8; ++k) {
        sum += ik_matrix_[k * 3 + r] * ik_matrix_[k * 3 + c];
      }
      ata[r * 3 + c] = sum;
    }
  }

  // 3×3 矩阵求逆（解析法）
  double det = ata[0] * (ata[4] * ata[8] - ata[5] * ata[7])
             - ata[1] * (ata[3] * ata[8] - ata[5] * ata[6])
             + ata[2] * (ata[3] * ata[7] - ata[4] * ata[6]);

  if (std::abs(det) < 1e-10) {
    // 退化情况，使用单位矩阵
    fk_matrix_.fill(0.0);
    return;
  }

  double inv_det = 1.0 / det;
  double ata_inv[9];
  ata_inv[0] = (ata[4] * ata[8] - ata[5] * ata[7]) * inv_det;
  ata_inv[1] = (ata[2] * ata[7] - ata[1] * ata[8]) * inv_det;
  ata_inv[2] = (ata[1] * ata[5] - ata[2] * ata[4]) * inv_det;
  ata_inv[3] = (ata[5] * ata[6] - ata[3] * ata[8]) * inv_det;
  ata_inv[4] = (ata[0] * ata[8] - ata[2] * ata[6]) * inv_det;
  ata_inv[5] = (ata[2] * ata[3] - ata[0] * ata[5]) * inv_det;
  ata_inv[6] = (ata[3] * ata[7] - ata[4] * ata[6]) * inv_det;
  ata_inv[7] = (ata[1] * ata[6] - ata[0] * ata[7]) * inv_det;
  ata_inv[8] = (ata[0] * ata[4] - ata[1] * ata[3]) * inv_det;

  // fk = (A^T * A)^{-1} * A^T  → (3×3) × (3×8) = (3×8)
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 8; ++c) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += ata_inv[r * 3 + k] * ik_matrix_[c * 3 + k];
      }
      fk_matrix_[r * 8 + c] = sum;
    }
  }
}

}  // namespace swerve_controller
