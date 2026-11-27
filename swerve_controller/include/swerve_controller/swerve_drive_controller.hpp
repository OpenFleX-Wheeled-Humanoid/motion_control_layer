/**
 * @File    swerve_drive_controller.hpp
 * @Time    2026/02/24
 * @Author  OpenArmX
 * @Version 1.0
 * @Desc    ros2_control 舵轮底盘控制器插件
 */

#ifndef SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "swerve_controller/swerve_drive_kinematics.hpp"
#include "swerve_controller/swerve_drive_odometry.hpp"

namespace swerve_controller
{

class SwerveDriveController : public controller_interface::ControllerInterface
{
public:
  SwerveDriveController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  static builtin_interfaces::msg::Time to_builtin_time(int64_t time_ns);
  int64_t current_time_ns(const rclcpp::Time & fallback_time) const;

  // 关节名称
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;

  // 参数
  double wheel_radius_ = 0.075;
  std::array<ModulePosition, 4> module_positions_;
  std::array<double, 4> steering_min_;
  std::array<double, 4> steering_max_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string cmd_vel_topic_;
  std::string debug_topic_;
  double cmd_vel_timeout_ = 0.5;
  double publish_rate_ = 50.0;
  double debug_publish_rate_ = 20.0;
  double max_wheel_speed_ = 2.0;

  // 运动学和里程计
  SwerveDriveKinematics kinematics_;
  SwerveDriveOdometry odometry_;

  // cmd_vel 与 /clock 订阅
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buffer_;
  std::atomic<int64_t> latest_sim_time_ns_{-1};
  std::atomic<int64_t> last_cmd_vel_time_ns_{0};

  // 转向角度锁定（防止零速时追踪反馈位置导致振荡）
  std::array<double, 4> last_commanded_angles_{};
  bool first_update_ = true;

  // 驱动轮速度 slew rate 限制
  double wheel_accel_limit_ = 1.0;  // m/s²
  std::array<double, 4> last_wheel_speeds_{};  // 上一周期实际输出的轮速 (m/s)

  // 转向同步：保留整体对齐状态，但驱动按每个轮子的角度误差连续缩放
  double steering_align_threshold_ = 0.087;  // ~5°, 同步对齐精度
  double steering_stop_threshold_ = 1.22173;  // ~70°, 误差过大时暂停该轮驱动
  double steering_slowdown_exponent_ = 2.0;  // 误差越大，轮速衰减越快
  double steering_global_stop_threshold_ = 0.45;  // 全车最差转向误差过大时暂停驱动
  double steering_global_slowdown_exponent_ = 2.5;  // 全车对齐前整体降速
  double module_speed_deadband_ = 0.03;  // m/s, 小速度命令直接置零避免抖动
  double steering_command_deadband_ = 0.02;  // rad, 小角度命令保持不动避免来回摆
  double low_speed_steering_command_deadband_ = 0.10;  // rad, 低速时更强过滤舵角微摆
  double low_speed_steering_deadband_speed_ = 0.16;  // m/s, 低于该模块速度使用低速舵角死区
  bool steering_aligned_ = false;  // 所有轮子是否已对齐

  // 里程计发布
  bool enable_odom_tf_ = true;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> debug_pub_;
  std::atomic<int64_t> last_publish_time_ns_{0};
  std::atomic<int64_t> last_debug_publish_time_ns_{0};
};

}  // namespace swerve_controller

#endif  // SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
