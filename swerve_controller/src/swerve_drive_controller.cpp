/**
 * @File    swerve_drive_controller.cpp
 * @Time    2026/02/24
 * @Author  OpenArmX
 * @Version 1.0
 * @Desc    ros2_control 舵轮底盘控制器插件实现
 */

#include "swerve_controller/swerve_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace swerve_controller
{

SwerveDriveController::SwerveDriveController()
: controller_interface::ControllerInterface()
{
}

void SwerveDriveController::on_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  const int64_t sim_time_ns =
    static_cast<int64_t>(msg->clock.sec) * 1000000000LL +
    static_cast<int64_t>(msg->clock.nanosec);

  latest_sim_time_ns_.store(sim_time_ns, std::memory_order_relaxed);

  if (last_cmd_vel_time_ns_.load(std::memory_order_relaxed) <= 0) {
    last_cmd_vel_time_ns_.store(sim_time_ns, std::memory_order_relaxed);
  }
  if (last_publish_time_ns_.load(std::memory_order_relaxed) <= 0) {
    last_publish_time_ns_.store(sim_time_ns, std::memory_order_relaxed);
  }
}

builtin_interfaces::msg::Time SwerveDriveController::to_builtin_time(const int64_t time_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(time_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000LL);
  return stamp;
}

int64_t SwerveDriveController::current_time_ns(const rclcpp::Time & fallback_time) const
{
  const int64_t sim_time_ns = latest_sim_time_ns_.load(std::memory_order_relaxed);
  if (sim_time_ns >= 0) {
    return sim_time_ns;
  }

  const int64_t fallback_ns = fallback_time.nanoseconds();
  if (fallback_ns > 0) {
    return fallback_ns;
  }

  return get_node()->now().nanoseconds();
}

controller_interface::CallbackReturn SwerveDriveController::on_init()
{
  auto_declare<std::vector<std::string>>("steering_joint_names",
    std::vector<std::string>{"fl_steering_joint", "fr_steering_joint",
                             "bl_steering_joint", "br_steering_joint"});
  auto_declare<std::vector<std::string>>("wheel_joint_names",
    std::vector<std::string>{"fl_wheel_joint", "fr_wheel_joint",
                             "bl_wheel_joint", "br_wheel_joint"});

  auto_declare<double>("wheel_radius", 0.075);

  // 每模块转向限位 (rad)
  auto_declare<double>("fl_steering_min", -1.5708);
  auto_declare<double>("fl_steering_max",  1.5708);
  auto_declare<double>("fr_steering_min", -1.5708);
  auto_declare<double>("fr_steering_max",  1.5708);
  auto_declare<double>("bl_steering_min", -1.5708);
  auto_declare<double>("bl_steering_max",  1.5708);
  auto_declare<double>("br_steering_min", -1.5708);
  auto_declare<double>("br_steering_max",  1.5708);

  // 模块位置 [x_forward, y_left] (m)
  auto_declare<double>("fl_pos_x",  0.21);
  auto_declare<double>("fl_pos_y",  0.26);
  auto_declare<double>("fr_pos_x",  0.21);
  auto_declare<double>("fr_pos_y", -0.26);
  auto_declare<double>("bl_pos_x", -0.21);
  auto_declare<double>("bl_pos_y",  0.26);
  auto_declare<double>("br_pos_x", -0.21);
  auto_declare<double>("br_pos_y", -0.26);

  auto_declare<std::string>("odom_frame_id", "odom");
  auto_declare<std::string>("base_frame_id", "base_link");
  auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
  auto_declare<std::string>("debug_topic", "/swerve_drive_controller/debug_states");
  auto_declare<double>("cmd_vel_timeout", 0.5);
  auto_declare<double>("publish_rate", 50.0);
  auto_declare<double>("debug_publish_rate", 20.0);
  auto_declare<double>("max_wheel_speed", 2.0);
  auto_declare<double>("wheel_accel_limit", 2.0);
  auto_declare<bool>("enable_odom_tf", true);
  auto_declare<double>("steering_align_threshold", 0.087);  // ~5°, 转向同步对齐精度
  auto_declare<double>("steering_stop_threshold", 1.22173);  // ~70°, 误差过大时暂停该轮驱动
  auto_declare<double>("steering_slowdown_exponent", 2.0);
  auto_declare<double>("steering_global_stop_threshold", 0.45);
  auto_declare<double>("steering_global_slowdown_exponent", 2.5);
  auto_declare<double>("module_speed_deadband", 0.03);
  auto_declare<double>("steering_command_deadband", 0.02);
  auto_declare<double>("low_speed_steering_command_deadband", 0.10);
  auto_declare<double>("low_speed_steering_deadband_speed", 0.16);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  steering_joint_names_ = get_node()->get_parameter("steering_joint_names")
    .as_string_array();
  wheel_joint_names_ = get_node()->get_parameter("wheel_joint_names")
    .as_string_array();

  if (steering_joint_names_.size() != 4 || wheel_joint_names_.size() != 4) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Need exactly 4 steering joints and 4 wheel joints");
    return controller_interface::CallbackReturn::ERROR;
  }

  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  if (wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "wheel_radius must be > 0, got %.6f", wheel_radius_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // 读取转向限位
  steering_min_[0] = get_node()->get_parameter("fl_steering_min").as_double();
  steering_max_[0] = get_node()->get_parameter("fl_steering_max").as_double();
  steering_min_[1] = get_node()->get_parameter("fr_steering_min").as_double();
  steering_max_[1] = get_node()->get_parameter("fr_steering_max").as_double();
  steering_min_[2] = get_node()->get_parameter("bl_steering_min").as_double();
  steering_max_[2] = get_node()->get_parameter("bl_steering_max").as_double();
  steering_min_[3] = get_node()->get_parameter("br_steering_min").as_double();
  steering_max_[3] = get_node()->get_parameter("br_steering_max").as_double();

  // 读取模块位置
  module_positions_[0] = {
    get_node()->get_parameter("fl_pos_x").as_double(),
    get_node()->get_parameter("fl_pos_y").as_double()
  };
  module_positions_[1] = {
    get_node()->get_parameter("fr_pos_x").as_double(),
    get_node()->get_parameter("fr_pos_y").as_double()
  };
  module_positions_[2] = {
    get_node()->get_parameter("bl_pos_x").as_double(),
    get_node()->get_parameter("bl_pos_y").as_double()
  };
  module_positions_[3] = {
    get_node()->get_parameter("br_pos_x").as_double(),
    get_node()->get_parameter("br_pos_y").as_double()
  };

  kinematics_.configure(module_positions_, steering_min_, steering_max_);
  odometry_.setKinematics(&kinematics_);
  odometry_.reset();

  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
  debug_topic_ = get_node()->get_parameter("debug_topic").as_string();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  debug_publish_rate_ = get_node()->get_parameter("debug_publish_rate").as_double();
  max_wheel_speed_ = get_node()->get_parameter("max_wheel_speed").as_double();
  wheel_accel_limit_ = get_node()->get_parameter("wheel_accel_limit").as_double();
  steering_align_threshold_ = get_node()->get_parameter("steering_align_threshold").as_double();
  steering_stop_threshold_ = get_node()->get_parameter("steering_stop_threshold").as_double();
  steering_slowdown_exponent_ = get_node()->get_parameter("steering_slowdown_exponent").as_double();
  steering_global_stop_threshold_ =
    get_node()->get_parameter("steering_global_stop_threshold").as_double();
  steering_global_slowdown_exponent_ =
    get_node()->get_parameter("steering_global_slowdown_exponent").as_double();
  module_speed_deadband_ = get_node()->get_parameter("module_speed_deadband").as_double();
  steering_command_deadband_ = get_node()->get_parameter("steering_command_deadband").as_double();
  low_speed_steering_command_deadband_ =
    get_node()->get_parameter("low_speed_steering_command_deadband").as_double();
  low_speed_steering_deadband_speed_ =
    get_node()->get_parameter("low_speed_steering_deadband_speed").as_double();

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "publish_rate must be > 0, got %.6f", publish_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (debug_publish_rate_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "debug_publish_rate must be >= 0, got %.6f", debug_publish_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (max_wheel_speed_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_wheel_speed must be > 0, got %.6f", max_wheel_speed_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (wheel_accel_limit_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "wheel_accel_limit must be > 0, got %.6f", wheel_accel_limit_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (steering_slowdown_exponent_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "steering_slowdown_exponent must be > 0, got %.6f", steering_slowdown_exponent_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (steering_global_slowdown_exponent_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "steering_global_slowdown_exponent must be > 0, got %.6f",
      steering_global_slowdown_exponent_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (module_speed_deadband_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "module_speed_deadband must be >= 0, got %.6f", module_speed_deadband_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (steering_command_deadband_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "steering_command_deadband must be >= 0, got %.6f", steering_command_deadband_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (low_speed_steering_command_deadband_ < steering_command_deadband_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "low_speed_steering_command_deadband (%.6f) must be >= steering_command_deadband (%.6f)",
      low_speed_steering_command_deadband_, steering_command_deadband_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (low_speed_steering_deadband_speed_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "low_speed_steering_deadband_speed must be >= 0, got %.6f",
      low_speed_steering_deadband_speed_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (steering_stop_threshold_ < steering_align_threshold_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "steering_stop_threshold (%.6f) must be >= steering_align_threshold (%.6f)",
      steering_stop_threshold_, steering_align_threshold_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (steering_global_stop_threshold_ < steering_align_threshold_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "steering_global_stop_threshold (%.6f) must be >= steering_align_threshold (%.6f)",
      steering_global_stop_threshold_, steering_align_threshold_);
    return controller_interface::CallbackReturn::ERROR;
  }

  enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();

  clock_sub_ = get_node()->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(10).best_effort(),
    [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
      on_clock(msg);
    });

  // 订阅 cmd_vel
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_.writeFromNonRT(*msg);
      const int64_t time_ns =
        current_time_ns(rclcpp::Time(0, 0, RCL_ROS_TIME));
      if (time_ns > 0) {
        last_cmd_vel_time_ns_.store(time_ns, std::memory_order_relaxed);
      }
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Swerve controller listening on %s with global steering stop threshold %.3f rad",
    cmd_vel_topic_.c_str(), steering_global_stop_threshold_);

  // 创建里程计发布者
  auto odom_publisher = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    odom_publisher);

  auto tf_publisher = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::SystemDefaultsQoS());
  tf_pub_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
    tf_publisher);

  auto debug_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    debug_topic_, rclcpp::SystemDefaultsQoS());
  debug_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      debug_publisher);

  // 初始化消息
  auto & odom_msg = odom_pub_->msg_;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = base_frame_id_;

  auto & tf_msg = tf_pub_->msg_;
  tf_msg.transforms.resize(1);
  tf_msg.transforms[0].header.frame_id = odom_frame_id_;
  tf_msg.transforms[0].child_frame_id = base_frame_id_;

  auto & debug_msg = debug_pub_->msg_;
  debug_msg.layout.dim.resize(1);
  debug_msg.layout.dim[0].label =
    "cmd_vx,cmd_vy,cmd_wz,global_drive_scale,worst_angle_error,"
    "per_module(fl,fr,bl,br):requested_angle,commanded_angle,actual_angle,"
    "commanded_minus_actual,requested_speed,target_speed,commanded_speed,"
    "actual_speed,active_deadband,steering_scale";
  debug_msg.layout.dim[0].size = 45;
  debug_msg.layout.dim[0].stride = 45;
  debug_msg.data.resize(45, 0.0);

  last_cmd_vel_time_ns_.store(0, std::memory_order_relaxed);
  last_publish_time_ns_.store(0, std::memory_order_relaxed);
  last_debug_publish_time_ns_.store(0, std::memory_order_relaxed);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Swerve debug publisher: topic=%s rate=%.1fHz data=[cmd_vx,cmd_vy,cmd_wz,"
    "global_drive_scale,worst_angle_error,"
    "per module fl/fr/bl/br: requested_angle,commanded_angle,actual_angle,"
    "commanded_minus_actual,requested_speed,target_speed,commanded_speed,"
    "actual_speed,active_deadband,steering_scale]",
    debug_topic_.c_str(), debug_publish_rate_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SwerveDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & name : steering_joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & name : wheel_joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::InterfaceConfiguration
SwerveDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & name : steering_joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & name : wheel_joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 初始化 cmd_vel 为零
  geometry_msgs::msg::Twist zero;
  cmd_vel_buffer_.writeFromNonRT(zero);
  last_cmd_vel_time_ns_.store(0, std::memory_order_relaxed);
  last_publish_time_ns_.store(0, std::memory_order_relaxed);
  last_debug_publish_time_ns_.store(0, std::memory_order_relaxed);

  // 锁定角度：由 update() 首次调用时从电机反馈初始化
  last_commanded_angles_.fill(0.0);
  first_update_ = true;
  steering_aligned_ = false;

  // 初始化上一周期轮速为 0
  last_wheel_speeds_.fill(0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停止所有电机
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(0.0);       // 转向归零
    command_interfaces_[4 + i].set_value(0.0);   // 驱动归零
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  // 1. 读取 cmd_vel
  auto cmd = *cmd_vel_buffer_.readFromRT();

  const int64_t time_ns = current_time_ns(time);

  // 2. 超时检查：仿真优先使用 /clock，实机回退到控制循环时间
  if (time_ns > 0) {
    int64_t last_cmd_vel_time_ns = last_cmd_vel_time_ns_.load(std::memory_order_relaxed);
    if (last_cmd_vel_time_ns <= 0 || time_ns < last_cmd_vel_time_ns) {
      last_cmd_vel_time_ns = time_ns;
      last_cmd_vel_time_ns_.store(time_ns, std::memory_order_relaxed);
    }

    const double elapsed = static_cast<double>(time_ns - last_cmd_vel_time_ns) / 1e9;
    if (elapsed > cmd_vel_timeout_) {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }
  }

  // 3. 读取当前转向角度
  std::array<double, 4> current_angles;
  for (size_t i = 0; i < 4; ++i) {
    // state_interfaces 顺序: 4×steering(pos,vel) + 4×wheel(pos,vel)
    current_angles[i] = state_interfaces_[i * 2].get_value();
  }

  // 首次更新时从电机实际位置初始化锁定角度
  if (first_update_) {
    last_commanded_angles_ = current_angles;
    first_update_ = false;
  }

  // 4. 逆运动学（以实际转向反馈为参考，避免轮子在重规划时反复翻角）
  ChassisSpeeds speeds;
  speeds.vx = cmd.linear.x;
  speeds.vy = cmd.linear.y;
  speeds.omega = cmd.angular.z;

  auto module_states = kinematics_.toModuleStates(speeds, current_angles);

  // 5. 速度去饱和（最大轮速可配置）
  SwerveDriveKinematics::desaturateWheelSpeeds(module_states, max_wheel_speed_);

  // 小速度命令保持当前转向，避免探索/局部避障阶段的无意义抖动。
  for (size_t i = 0; i < 4; ++i) {
    if (std::abs(module_states[i].speed) < module_speed_deadband_) {
      module_states[i].speed = 0.0;
      module_states[i].angle = current_angles[i];
    }
  }

  // 6. 转向同步机制：记录整体对齐状态，但按每个轮子的误差连续衰减轮速。
  double worst_angle_error = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    double angle_diff = module_states[i].angle - current_angles[i];
    double angle_error = std::abs(std::atan2(std::sin(angle_diff), std::cos(angle_diff)));
    worst_angle_error = std::max(worst_angle_error, angle_error);
  }

  // 带迟滞的同步判断：防止在阈值边缘反复切换
  // 进入对齐状态需要误差 < threshold，退出对齐状态需要误差 > threshold * 2
  if (steering_aligned_) {
    if (worst_angle_error > steering_align_threshold_ * 2.0) {
      steering_aligned_ = false;
    }
  } else {
    if (worst_angle_error <= steering_align_threshold_) {
      steering_aligned_ = true;
    }
  }

  double global_drive_scale = 1.0;
  const bool chassis_motion_requested = std::any_of(
    module_states.begin(), module_states.end(),
    [this](const SwerveModuleState & state) {
      return std::abs(state.speed) >= module_speed_deadband_;
    });
  if (chassis_motion_requested) {
    if (worst_angle_error >= steering_global_stop_threshold_) {
      global_drive_scale = 0.0;
    } else if (worst_angle_error > steering_align_threshold_) {
      global_drive_scale = std::pow(
        std::max(0.0, std::cos(worst_angle_error)), steering_global_slowdown_exponent_);
    }
  }

  double max_delta = wheel_accel_limit_ * dt;  // 本周期最大速度变化 (m/s)

  std::array<double, 4> debug_requested_angles{};
  std::array<double, 4> debug_commanded_angles{};
  std::array<double, 4> debug_commanded_angle_errors{};
  std::array<double, 4> debug_requested_speeds{};
  std::array<double, 4> debug_target_speeds{};
  std::array<double, 4> debug_commanded_speeds{};
  std::array<double, 4> debug_actual_speeds{};
  std::array<double, 4> debug_active_deadbands{};
  std::array<double, 4> debug_steering_scales{};

  for (size_t i = 0; i < 4; ++i) {
    debug_requested_angles[i] = module_states[i].angle;
    debug_requested_speeds[i] = module_states[i].speed;

    double angle_diff = module_states[i].angle - current_angles[i];
    double angle_error = std::abs(std::atan2(std::sin(angle_diff), std::cos(angle_diff)));

    // 速度为零时保持上次转向角，避免停车瞬间轮子跳转
    double steering_scale = 1.0;
    if (module_states[i].speed == 0.0) {
      steering_scale = 0.0;
      module_states[i].angle = last_commanded_angles_[i];
    } else if (angle_error >= steering_stop_threshold_) {
      steering_scale = 0.0;
    } else if (angle_error > steering_align_threshold_) {
      steering_scale = std::pow(
        std::max(0.0, std::cos(angle_error)), steering_slowdown_exponent_);
    }

    // 低速时限制转向角变化率，防止终点附近轮子突然摆动
    // 最大转向角速度 = 基础值 + 速度成比例放大，低速时转向慢，高速时转向快
    constexpr double kSteeringRateBase = 0.8;   // rad/s 最低转向速率
    constexpr double kSteeringRatePerSpeed = 6.0; // rad/s per m/s 轮速
    double max_steer_delta = (kSteeringRateBase +
      kSteeringRatePerSpeed * std::abs(module_states[i].speed)) * dt;
    double steer_diff = module_states[i].angle - last_commanded_angles_[i];
    steer_diff = std::atan2(std::sin(steer_diff), std::cos(steer_diff));  // normalize
    const double active_steering_deadband =
      std::abs(module_states[i].speed) < low_speed_steering_deadband_speed_
        ? low_speed_steering_command_deadband_
        : steering_command_deadband_;
    debug_active_deadbands[i] = active_steering_deadband;
    if (std::abs(steer_diff) < active_steering_deadband) {
      steer_diff = 0.0;
    }
    if (steer_diff > max_steer_delta) steer_diff = max_steer_delta;
    if (steer_diff < -max_steer_delta) steer_diff = -max_steer_delta;
    module_states[i].angle = last_commanded_angles_[i] + steer_diff;

    command_interfaces_[i].set_value(module_states[i].angle);
    last_commanded_angles_[i] = module_states[i].angle;
    debug_commanded_angles[i] = module_states[i].angle;
    debug_commanded_angle_errors[i] = std::atan2(
      std::sin(module_states[i].angle - current_angles[i]),
      std::cos(module_states[i].angle - current_angles[i]));
    debug_steering_scales[i] = steering_scale;

    // 目标轮速 (m/s)：先按单轮误差衰减，再按全车最差误差统一缩放，
    // 避免单个车轮还在翻角时底盘整体出现“拧着走”的诡异动作。
    double target_speed = module_states[i].speed * steering_scale * global_drive_scale;
    debug_target_speeds[i] = target_speed;

    // Slew rate 限制：限制每周期速度变化量
    double delta = target_speed - last_wheel_speeds_[i];
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    double output_speed = last_wheel_speeds_[i] + delta;
    last_wheel_speeds_[i] = output_speed;
    debug_commanded_speeds[i] = output_speed;

    // 轮速 m/s → rad/s
    command_interfaces_[4 + i].set_value(output_speed / wheel_radius_);
  }

  // 7. 读取状态接口，构建模块状态用于里程计
  std::array<SwerveModuleState, 4> odom_states;
  for (size_t i = 0; i < 4; ++i) {
    odom_states[i].angle = state_interfaces_[i * 2].get_value();
    // 轮速 rad/s → m/s
    double wheel_vel = state_interfaces_[8 + i * 2 + 1].get_value();
    odom_states[i].speed = wheel_vel * wheel_radius_;
    debug_actual_speeds[i] = odom_states[i].speed;
  }

  if (debug_publish_rate_ > 0.0 && time_ns > 0) {
    int64_t last_debug_publish_time_ns = last_debug_publish_time_ns_.load(std::memory_order_relaxed);
    if (last_debug_publish_time_ns <= 0 || time_ns < last_debug_publish_time_ns) {
      last_debug_publish_time_ns = time_ns;
      last_debug_publish_time_ns_.store(time_ns, std::memory_order_relaxed);
    }

    const double debug_publish_dt =
      static_cast<double>(time_ns - last_debug_publish_time_ns) / 1e9;
    if (debug_publish_dt >= 1.0 / debug_publish_rate_ && debug_pub_->trylock()) {
      auto & msg = debug_pub_->msg_;
      if (msg.data.size() != 45) {
        msg.data.resize(45, 0.0);
      }
      msg.data[0] = cmd.linear.x;
      msg.data[1] = cmd.linear.y;
      msg.data[2] = cmd.angular.z;
      msg.data[3] = global_drive_scale;
      msg.data[4] = worst_angle_error;
      for (size_t i = 0; i < 4; ++i) {
        const size_t base = 5 + i * 10;
        msg.data[base + 0] = debug_requested_angles[i];
        msg.data[base + 1] = debug_commanded_angles[i];
        msg.data[base + 2] = current_angles[i];
        msg.data[base + 3] = debug_commanded_angle_errors[i];
        msg.data[base + 4] = debug_requested_speeds[i];
        msg.data[base + 5] = debug_target_speeds[i];
        msg.data[base + 6] = debug_commanded_speeds[i];
        msg.data[base + 7] = debug_actual_speeds[i];
        msg.data[base + 8] = debug_active_deadbands[i];
        msg.data[base + 9] = debug_steering_scales[i];
      }
      last_debug_publish_time_ns_.store(time_ns, std::memory_order_relaxed);
      debug_pub_->unlockAndPublish();
    }
  }

  // 8. 更新里程计
  odometry_.update(odom_states, dt);

  // 9. 发布里程计和 TF：仿真用 /clock，实机用 ROS 时间
  if (time_ns > 0) {
    int64_t last_publish_time_ns = last_publish_time_ns_.load(std::memory_order_relaxed);
    if (last_publish_time_ns <= 0 || time_ns < last_publish_time_ns) {
      last_publish_time_ns = time_ns;
      last_publish_time_ns_.store(time_ns, std::memory_order_relaxed);
    }

    const double publish_dt = static_cast<double>(time_ns - last_publish_time_ns) / 1e9;
    if (publish_dt >= 1.0 / publish_rate_) {
      last_publish_time_ns_.store(time_ns, std::memory_order_relaxed);
      const builtin_interfaces::msg::Time stamp = to_builtin_time(time_ns);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, odometry_.getHeading());

      if (odom_pub_->trylock()) {
        auto & msg = odom_pub_->msg_;
        msg.header.stamp = stamp;
        msg.pose.pose.position.x = odometry_.getX();
        msg.pose.pose.position.y = odometry_.getY();
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.twist.twist.linear.x = odometry_.getLinearX();
        msg.twist.twist.linear.y = odometry_.getLinearY();
        msg.twist.twist.angular.z = odometry_.getAngularZ();
        odom_pub_->unlockAndPublish();
      }

      if (enable_odom_tf_ && tf_pub_->trylock()) {
        auto & tf = tf_pub_->msg_.transforms[0];
        tf.header.stamp = stamp;
        tf.transform.translation.x = odometry_.getX();
        tf.transform.translation.y = odometry_.getY();
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_pub_->unlockAndPublish();
      }
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace swerve_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  swerve_controller::SwerveDriveController,
  controller_interface::ControllerInterface)
