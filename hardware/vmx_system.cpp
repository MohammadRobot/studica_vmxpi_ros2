// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "studica_vmxpi_ros2/vmx_system.hpp"
#include "studica_vmxpi_ros2/velocity_pid_safety.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRpmToRadPerSec = 2.0 * kPi / 60.0;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kGToMetersPerSecondSquared = 9.80665;
}  // namespace

namespace studica_vmxpi_ros2
{
hardware_interface::CallbackReturn VmxSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.Titan"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  auto get_int_param = [&](const std::string & name, int & out) -> bool {
    auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(get_logger(), "Missing hardware parameter '%s'", name.c_str());
      return false;
    }
    try {
      out = std::stoi(it->second);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Invalid int for '%s': %s", name.c_str(), ex.what());
      return false;
    }
    return true;
  };

  auto get_double_param = [&](const std::string & name, double & out, bool required) -> bool {
    auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end()) {
      if (required) {
        RCLCPP_ERROR(get_logger(), "Missing hardware parameter '%s'", name.c_str());
        return false;
      }
      return true;
    }
    try {
      out = hardware_interface::stod(it->second);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Invalid double for '%s': %s", name.c_str(), ex.what());
      return false;
    }
    return true;
  };

  auto get_bool_param = [&](const std::string & name, bool & out, bool required) -> bool {
    auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end()) {
      if (required) {
        RCLCPP_ERROR(get_logger(), "Missing hardware parameter '%s'", name.c_str());
        return false;
      }
      return true;
    }
    std::string value = it->second;
    for (auto & c : value) {
      c = static_cast<char>(std::tolower(c));
    }
    if (value == "true" || value == "1") {
      out = true;
      return true;
    }
    if (value == "false" || value == "0") {
      out = false;
      return true;
    }
    RCLCPP_ERROR(get_logger(), "Invalid bool for '%s': %s", name.c_str(), it->second.c_str());
    return false;
  };

  auto get_string_param = [&](const std::string & name, std::string & out, bool required) -> bool {
    auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end()) {
      if (required) {
        RCLCPP_ERROR(get_logger(), "Missing hardware parameter '%s'", name.c_str());
        return false;
      }
      return true;
    }
    out = it->second;
    return true;
  };

  int can_id = 0;
  int motor_freq = 0;
  int ticks_per_rotation = 0;
  double wheel_radius = 0.0;

  if (
    !get_int_param("can_id", can_id) ||
    !get_int_param("motor_freq", motor_freq) ||
    !get_int_param("ticks_per_rotation", ticks_per_rotation) ||
    !get_double_param("wheel_radius", wheel_radius, true))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (can_id < 0 || can_id > static_cast<int>(std::numeric_limits<uint8_t>::max())) {
    RCLCPP_ERROR(
      get_logger(), "can_id=%d is out of range [0, %u].", can_id,
      static_cast<unsigned>(std::numeric_limits<uint8_t>::max()));
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (motor_freq <= 0 || motor_freq > static_cast<int>(std::numeric_limits<uint16_t>::max())) {
    RCLCPP_ERROR(
      get_logger(), "motor_freq=%d is out of range [1, %u].", motor_freq,
      static_cast<unsigned>(std::numeric_limits<uint16_t>::max()));
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_id_ = static_cast<uint8_t>(can_id);
  motor_freq_ = static_cast<uint16_t>(motor_freq);
  ticks_per_rotation_ = ticks_per_rotation;
  wheel_radius_ = wheel_radius;

  if (!get_double_param("speed_scale", speed_scale_, false)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!get_double_param("max_wheel_angular_velocity_rad_s", max_wheel_angular_velocity_rad_s_, false)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  double feedback_warn_timeout_ms = 100.0;
  double feedback_error_timeout_ms = 250.0;
  double controller_temp_error_timeout_ms = 3000.0;
  if (!get_string_param("control_mode", control_mode_name_, false) ||
      !get_bool_param("pid_require_supported", pid_require_supported_, false) ||
      !get_bool_param("wheel_radius_calibrated", wheel_radius_calibrated_, false) ||
      !get_double_param("feedback_warn_timeout_ms", feedback_warn_timeout_ms, false) ||
      !get_double_param("feedback_error_timeout_ms", feedback_error_timeout_ms, false) ||
      !get_double_param("controller_temp_error_c", controller_temp_error_c_, false) ||
      !get_double_param(
        "controller_temp_error_timeout_ms", controller_temp_error_timeout_ms, false))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  int pid_sensitivity = static_cast<int>(pid_sensitivity_);
  if (!get_int_param("pid_sensitivity", pid_sensitivity)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!velocity_pid_safety::valid_pid_config("mcv2", pid_sensitivity)) {
    RCLCPP_ERROR(get_logger(), "pid_sensitivity must be in [0, 10].");
    return hardware_interface::CallbackReturn::ERROR;
  }
  pid_sensitivity_ = static_cast<uint8_t>(pid_sensitivity);

  std::string pid_type_name{"mcv2"};
  if (!get_string_param("pid_type", pid_type_name, false)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::transform(
    control_mode_name_.begin(), control_mode_name_.end(), control_mode_name_.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::transform(
    pid_type_name.begin(), pid_type_name.end(), pid_type_name.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (control_mode_name_ == "open_loop") {
    control_mode_ = MotorControlMode::OPEN_LOOP;
  } else if (control_mode_name_ == "velocity_pid") {
    control_mode_ = MotorControlMode::VELOCITY_PID;
  } else {
    RCLCPP_ERROR(
      get_logger(), "Unsupported control_mode='%s' (expected open_loop or velocity_pid).",
      control_mode_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!velocity_pid_safety::valid_pid_config(pid_type_name, pid_sensitivity)) {
    RCLCPP_ERROR(get_logger(), "Only pid_type='mcv2' is supported by velocity_pid mode.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  pid_type_ = TITAN_PID_TYPE_MCV2;

  // Hardware parameters are expressed in milliseconds for operator-friendly profiles.
  feedback_warn_timeout_sec_ = feedback_warn_timeout_ms / 1000.0;
  feedback_error_timeout_sec_ = feedback_error_timeout_ms / 1000.0;
  controller_temp_error_timeout_sec_ = controller_temp_error_timeout_ms / 1000.0;
  if (speed_scale_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "speed_scale must be >= 0.0.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (max_wheel_angular_velocity_rad_s_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "max_wheel_angular_velocity_rad_s must be > 0.0.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (feedback_warn_timeout_sec_ <= 0.0 ||
      feedback_error_timeout_sec_ <= feedback_warn_timeout_sec_)
  {
    RCLCPP_ERROR(
      get_logger(), "Feedback timeouts must satisfy 0 < warn < error (received %.3f s, %.3f s).",
      feedback_warn_timeout_sec_, feedback_error_timeout_sec_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(controller_temp_error_c_) || controller_temp_error_c_ <= 0.0 ||
      !std::isfinite(controller_temp_error_timeout_sec_) ||
      controller_temp_error_timeout_sec_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Titan temperature limit and timeout must be finite and positive "
      "(received %.1f C, %.3f s).",
      controller_temp_error_c_, controller_temp_error_timeout_sec_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (control_mode_ == MotorControlMode::VELOCITY_PID && !wheel_radius_calibrated_) {
    RCLCPP_ERROR(
      get_logger(),
      "velocity_pid hardware motion is blocked: measure the wheel radius and set "
      "hardware.wheel_radius_calibrated=true in the selected robot profile.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_int_param("left_front_motor", left_front_motor_) ||
      !get_int_param("left_rear_motor", left_rear_motor_) ||
      !get_int_param("right_front_motor", right_front_motor_) ||
      !get_int_param("right_rear_motor", right_rear_motor_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_bool_param("invert_left_front_motor", invert_left_front_motor_, false) ||
      !get_bool_param("invert_left_rear_motor", invert_left_rear_motor_, false) ||
      !get_bool_param("invert_right_front_motor", invert_right_front_motor_, false) ||
      !get_bool_param("invert_right_rear_motor", invert_right_rear_motor_, false) ||
      !get_bool_param("invert_left_front_encoder", invert_left_front_encoder_, false) ||
      !get_bool_param("invert_left_rear_encoder", invert_left_rear_encoder_, false) ||
      !get_bool_param("invert_right_front_encoder", invert_right_front_encoder_, false) ||
      !get_bool_param("invert_right_rear_encoder", invert_right_rear_encoder_, false))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto validate_motor_index = [&](int motor, const std::string & name) -> bool {
    if (motor < -1 || motor > 3) {
      RCLCPP_ERROR(get_logger(), "Invalid %s index %d (valid: 0-3 or -1)", name.c_str(), motor);
      return false;
    }
    return true;
  };

  if (!validate_motor_index(left_front_motor_, "left_front_motor") ||
      !validate_motor_index(left_rear_motor_, "left_rear_motor") ||
      !validate_motor_index(right_front_motor_, "right_front_motor") ||
      !validate_motor_index(right_rear_motor_, "right_rear_motor"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (left_front_motor_ < 0 && left_rear_motor_ < 0) {
    RCLCPP_ERROR(get_logger(), "At least one left motor index must be set.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (right_front_motor_ < 0 && right_rear_motor_ < 0) {
    RCLCPP_ERROR(get_logger(), "At least one right motor index must be set.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (ticks_per_rotation_ <= 0 || wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "ticks_per_rotation and wheel_radius must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  dist_per_tick_ = 2.0 * kPi * wheel_radius_ / static_cast<double>(ticks_per_rotation_);

  const auto imu_sensor_it = std::find_if(
    info_.sensors.begin(), info_.sensors.end(),
    [](const auto & sensor) { return sensor.name == "imu_sensor"; });
  if (imu_sensor_it != info_.sensors.end()) {
    const auto & imu_sensor = *imu_sensor_it;
    imu_enabled_ = true;
    imu_sensor_name_ = imu_sensor.name;

    const std::vector<std::string> required_imu_interfaces = {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

    for (const auto & interface_name : required_imu_interfaces) {
      const bool has_interface = std::any_of(
        imu_sensor.state_interfaces.begin(),
        imu_sensor.state_interfaces.end(),
        [&](const auto & state_interface) {
          return state_interface.name == interface_name;
        });
      if (!has_interface) {
        RCLCPP_ERROR(
          get_logger(),
          "Sensor '%s' is missing required IMU state interface '%s'.",
          imu_sensor.name.c_str(),
          interface_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  } else {
    imu_enabled_ = false;
    RCLCPP_WARN(
      get_logger(),
      "No sensor interfaces configured for VmxSystemHardware; IMU publisher will stay disabled.");
  }

  const auto titan_sensor_it = std::find_if(
    info_.sensors.begin(), info_.sensors.end(),
    [](const auto & sensor) { return sensor.name == "titan_controller"; });
  if (titan_sensor_it != info_.sensors.end()) {
    titan_sensor_name_ = titan_sensor_it->name;
  }

  try {
    vmx_ = std::make_shared<VMXPi>(true, 50);
    if (!vmx_ || !vmx_->IsOpen()) {
      RCLCPP_ERROR(get_logger(), "Unable to open VMXPi device for VmxSystemHardware.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    titan_driver_ = std::make_unique<studica_driver::Titan>(
      can_id_, motor_freq_, static_cast<float>(dist_per_tick_), vmx_);
    uint8_t titan_info[8] = {0};
    if (titan_driver_->GetTitanInfo(titan_info)) {
      titan_firmware_major_ = static_cast<double>(titan_info[1]);
      titan_firmware_minor_ = static_cast<double>(titan_info[2]);
      titan_firmware_patch_ = static_cast<double>(titan_info[3]);
      RCLCPP_INFO(
        get_logger(), "Titan firmware %.0f.%.0f.%.0f detected on CAN ID %u.",
        titan_firmware_major_, titan_firmware_minor_, titan_firmware_patch_,
        static_cast<unsigned>(can_id_));
    }
    titan_pid_supported_ = titan_driver_->SupportsPIDType(pid_type_) ? 1.0 : 0.0;
    titan_pid_type_ = control_mode_ == MotorControlMode::VELOCITY_PID ?
      static_cast<double>(pid_type_) : static_cast<double>(TITAN_PID_TYPE_OFF);
    if (
      control_mode_ == MotorControlMode::VELOCITY_PID &&
      pid_require_supported_ && titan_pid_supported_ < 0.5)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Titan firmware does not report MCV2 support (firmware %.0f.%.0f); refusing velocity_pid mode.",
        titan_firmware_major_, titan_firmware_minor_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (imu_enabled_) {
      imu_driver_ = std::make_unique<studica_driver::Imu>(vmx_);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error initializing VmxSystemHardware drivers: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "VmxSystemHardware drivers initialized in on_init");

  // Initialize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commanded_velocities_.assign(info_.joints.size(), 0.0);
  hw_feedback_ages_.assign(info_.joints.size(), std::numeric_limits<double>::infinity());
  hw_encoder_fresh_.assign(info_.joints.size(), 0.0);
  hw_command_saturated_.assign(info_.joints.size(), 0.0);
  joint_motor_indices_.assign(info_.joints.size(), -1);
  last_rpm_feedback_times_.resize(info_.joints.size());
  last_encoder_feedback_times_.resize(info_.joints.size());
  rpm_feedback_seen_.assign(info_.joints.size(), false);
  encoder_feedback_seen_.assign(info_.joints.size(), false);
  is_independent_motor_layout_ = info_.joints.size() == 4;

  const int left_primary_motor =
    (left_front_motor_ >= 0) ? left_front_motor_ : left_rear_motor_;
  const int right_primary_motor =
    (right_front_motor_ >= 0) ? right_front_motor_ : right_rear_motor_;

  auto map_joint_to_motor = [&](const std::string & joint_name) -> int {
    if (joint_name.find("front_left") != std::string::npos) {
      return left_front_motor_;
    }
    if (joint_name.find("rear_left") != std::string::npos) {
      return left_rear_motor_;
    }
    if (joint_name.find("front_right") != std::string::npos) {
      return right_front_motor_;
    }
    if (joint_name.find("rear_right") != std::string::npos) {
      return right_rear_motor_;
    }
    if (joint_name.find("left_wheel") != std::string::npos) {
      return left_primary_motor;
    }
    if (joint_name.find("right_wheel") != std::string::npos) {
      return right_primary_motor;
    }
    return -1;
  };

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto has_state_interface = [&](const std::string & interface_name) {
      return std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [&](const auto & state_interface) { return state_interface.name == interface_name; });
    };
    for (const auto & required_interface : {
        std::string(hardware_interface::HW_IF_POSITION),
        std::string(hardware_interface::HW_IF_VELOCITY),
        std::string("commanded_velocity"), std::string("feedback_age"),
        std::string("encoder_fresh"), std::string("command_saturated")})
    {
      if (!has_state_interface(required_interface)) {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' is missing required state interface '%s'.",
          joint.name.c_str(), required_interface.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    joint_motor_indices_[i] = map_joint_to_motor(joint.name);
    if (is_independent_motor_layout_ && joint_motor_indices_[i] < 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to map holonomic joint '%s' to a Titan motor index. "
        "Expected one of: front_left, front_right, rear_left, rear_right.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto configure_encoder = [&](int motor) {
    if (motor < 0) {
      return;
    }
    titan_driver_->ConfigureEncoder(static_cast<uint8_t>(motor), dist_per_tick_);
    titan_driver_->ResetEncoder(static_cast<uint8_t>(motor));
  };

  for (const int motor : {left_front_motor_, left_rear_motor_, right_front_motor_, right_rear_motor_}) {
    configure_encoder(motor);
  }

  auto apply_if_enabled = [&](bool enabled, int motor, const auto & operation) {
    if (enabled && motor >= 0) {
      operation(static_cast<uint8_t>(motor));
    }
  };

  apply_if_enabled(
    invert_left_front_encoder_, left_front_motor_,
    [this](uint8_t motor) { titan_driver_->InvertEncoderDirection(motor); });
  apply_if_enabled(
    invert_left_rear_encoder_, left_rear_motor_,
    [this](uint8_t motor) { titan_driver_->InvertEncoderDirection(motor); });
  apply_if_enabled(
    invert_right_front_encoder_, right_front_motor_,
    [this](uint8_t motor) { titan_driver_->InvertEncoderDirection(motor); });
  apply_if_enabled(
    invert_right_rear_encoder_, right_rear_motor_,
    [this](uint8_t motor) { titan_driver_->InvertEncoderDirection(motor); });

  apply_if_enabled(
    invert_left_front_motor_, left_front_motor_,
    [this](uint8_t motor) { titan_driver_->InvertMotor(motor); });
  apply_if_enabled(
    invert_left_rear_motor_, left_rear_motor_,
    [this](uint8_t motor) { titan_driver_->InvertMotor(motor); });
  apply_if_enabled(
    invert_right_front_motor_, right_front_motor_,
    [this](uint8_t motor) { titan_driver_->InvertMotor(motor); });
  apply_if_enabled(
    invert_right_rear_motor_, right_rear_motor_,
    [this](uint8_t motor) { titan_driver_->InvertMotor(motor); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VmxSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "commanded_velocity", &hw_commanded_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "feedback_age", &hw_feedback_ages_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "encoder_fresh", &hw_encoder_fresh_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "command_saturated", &hw_command_saturated_[i]));
  }

  if (imu_enabled_) {
    state_interfaces.emplace_back(
      imu_sensor_name_, "orientation.x", &imu_orientation_x_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "orientation.y", &imu_orientation_y_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "orientation.z", &imu_orientation_z_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "orientation.w", &imu_orientation_w_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "angular_velocity.x", &imu_angular_velocity_x_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "angular_velocity.y", &imu_angular_velocity_y_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "angular_velocity.z", &imu_angular_velocity_z_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "linear_acceleration.x", &imu_linear_acceleration_x_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "linear_acceleration.y", &imu_linear_acceleration_y_);
    state_interfaces.emplace_back(
      imu_sensor_name_, "linear_acceleration.z", &imu_linear_acceleration_z_);
  }

  state_interfaces.emplace_back(
    titan_sensor_name_, "controller_temperature", &titan_controller_temperature_c_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "temperature_age", &titan_temperature_age_sec_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "pid_supported", &titan_pid_supported_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "pid_type", &titan_pid_type_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "fault_latched", &titan_fault_latched_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "firmware_major", &titan_firmware_major_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "firmware_minor", &titan_firmware_minor_);
  state_interfaces.emplace_back(
    titan_sensor_name_, "firmware_patch", &titan_firmware_patch_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VmxSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

void VmxSystemHardware::latch_fault(const std::string & reason)
{
  if (!fault_latched_) {
    fault_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "Titan safety fault latched: %s", reason.c_str());
    last_fault_stop_time_ = {};
  }
  fault_latched_ = true;
  titan_fault_latched_ = 1.0;
  std::fill(hw_commanded_velocities_.begin(), hw_commanded_velocities_.end(), 0.0);
  enforce_fault_stop();
}

bool VmxSystemHardware::stop_all_motors()
{
  if (!titan_driver_) {
    return false;
  }
  bool success = true;
  const std::vector<int> motors = {
    left_front_motor_, left_rear_motor_, right_front_motor_, right_rear_motor_};
  for (const int motor : motors) {
    if (motor < 0) {
      continue;
    }
    if (control_mode_ == MotorControlMode::VELOCITY_PID) {
      success = titan_driver_->TrySetTargetVelocity(static_cast<uint8_t>(motor), 0.0f) && success;
    } else {
      titan_driver_->SetSpeed(static_cast<uint8_t>(motor), 0.0);
    }
  }
  return success;
}

void VmxSystemHardware::enforce_fault_stop()
{
  if (!fault_latched_ || !titan_driver_) {
    return;
  }
  constexpr auto kFaultStopRefresh = std::chrono::milliseconds(100);
  const auto now = std::chrono::steady_clock::now();
  if (
    last_fault_stop_time_.time_since_epoch().count() != 0 &&
    now - last_fault_stop_time_ < kFaultStopRefresh)
  {
    return;
  }
  if (!stop_all_motors()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Failed to refresh one or more zero-RPM commands while Titan fault is latched.");
  }
  last_fault_stop_time_ = now;
}

bool VmxSystemHardware::wait_for_safe_controller_temperature()
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(controller_temp_error_timeout_sec_);
  bool received_fresh_sample = false;
  double last_temperature_c = std::numeric_limits<double>::quiet_NaN();

  while (std::chrono::steady_clock::now() < deadline) {
    float temperature_c = 0.0f;
    bool is_fresh = false;
    if (titan_driver_->GetControllerTempFresh(temperature_c, is_fresh) && is_fresh) {
      received_fresh_sample = true;
      last_temperature_c = static_cast<double>(temperature_c);
      if (velocity_pid_safety::safe_temperature_sample(
          last_temperature_c, controller_temp_error_c_))
      {
        titan_controller_temperature_c_ = last_temperature_c;
        temperature_seen_ = true;
        last_temperature_time_ = std::chrono::steady_clock::now();
        titan_temperature_age_sec_ = 0.0;
        return true;
      }
      RCLCPP_WARN(
        get_logger(),
        "Ignoring unsafe/invalid Titan startup temperature sample %.2f C; "
        "motors remain disabled while waiting for a valid sample below %.2f C.",
        last_temperature_c, controller_temp_error_c_);
    }
    vmx_->time.DelayMilliseconds(20);
  }

  if (received_fresh_sample) {
    RCLCPP_ERROR(
      get_logger(),
      "Titan activation refused: no safe temperature sample below %.2f C within %.2f s "
      "(last sample %.2f C).",
      controller_temp_error_c_, controller_temp_error_timeout_sec_, last_temperature_c);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Titan activation refused: no fresh controller-temperature telemetry within %.2f s.",
      controller_temp_error_timeout_sec_);
  }
  return false;
}

hardware_interface::CallbackReturn VmxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!titan_driver_) { // Safety check - should not happen if on_init is successful
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in on_activate!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (imu_enabled_ && !imu_driver_) {
    RCLCPP_ERROR(get_logger(), "IMU driver is not initialized in on_activate!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  fault_latched_ = false;
  fault_reason_.clear();
  titan_fault_latched_ = 0.0;
  last_fault_stop_time_ = {};
  temperature_seen_ = false;
  titan_controller_temperature_c_ = std::numeric_limits<double>::quiet_NaN();
  titan_temperature_age_sec_ = std::numeric_limits<double>::infinity();
  std::fill(rpm_feedback_seen_.begin(), rpm_feedback_seen_.end(), false);
  std::fill(encoder_feedback_seen_.begin(), encoder_feedback_seen_.end(), false);
  std::fill(hw_feedback_ages_.begin(), hw_feedback_ages_.end(), std::numeric_limits<double>::infinity());
  std::fill(hw_encoder_fresh_.begin(), hw_encoder_fresh_.end(), 0.0);

  // Set safe defaults before enabling motor output.
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
    }
    hw_commands_[i] = 0.0;
    hw_commanded_velocities_[i] = 0.0;
    hw_command_saturated_[i] = 0.0;
  }

  if (imu_enabled_) {
    imu_driver_->ZeroYaw();
  }

  if (control_mode_ == MotorControlMode::VELOCITY_PID) {
    if (titan_pid_supported_ < 0.5) {
      RCLCPP_ERROR(get_logger(), "MCV2 velocity PID is unavailable; Titan activation refused.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const int motor : {
        left_front_motor_, left_rear_motor_, right_front_motor_, right_rear_motor_})
    {
      if (motor < 0) {
        continue;
      }
      if (!titan_driver_->TrySetMotorPIDType(static_cast<uint8_t>(motor), pid_type_) ||
          !titan_driver_->TrySetSensitivity(static_cast<uint8_t>(motor), pid_sensitivity_))
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to configure MCV2 PID for Titan motor %d.", motor);
        stop_all_motors();
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Titan temperature is broadcast at about 1 Hz. Require a fresh, safe sample
  // while outputs are still disabled so a startup frame cannot falsely latch motion.
  if (
    control_mode_ == MotorControlMode::VELOCITY_PID &&
    !wait_for_safe_controller_temperature())
  {
    stop_all_motors();
    titan_driver_->TryEnable(false);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!stop_all_motors() || !titan_driver_->TryEnable(true)) {
    RCLCPP_ERROR(get_logger(), "Failed to zero and enable Titan safely.");
    titan_driver_->TryEnable(false);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "Titan activated in %s mode (PID type=%u sensitivity=%u).",
    control_mode_name_.c_str(), static_cast<unsigned>(pid_type_),
    static_cast<unsigned>(pid_sensitivity_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (titan_driver_) { // Check if titan_driver_ is valid before using
    stop_all_motors();
    titan_driver_->TryEnable(false);
  } else {
    RCLCPP_WARN(get_logger(), "Titan driver is not initialized in on_deactivate, nothing to disable.");
  }

  RCLCPP_INFO(get_logger(), "Titan Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VmxSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!titan_driver_) {
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in read()!");
    return hardware_interface::return_type::ERROR;
  }

  const auto steady_now = std::chrono::steady_clock::now();
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    std::vector<int> motors;
    if (is_independent_motor_layout_) {
      motors.push_back((i < joint_motor_indices_.size()) ? joint_motor_indices_[i] : -1);
    } else if (i == 0) {
      motors = {left_front_motor_, left_rear_motor_};
    } else {
      motors = {right_front_motor_, right_rear_motor_};
    }

    double distance_sum = 0.0;
    double rpm_sum = 0.0;
    size_t distance_count = 0;
    size_t rpm_count = 0;
    bool encoder_fresh = false;
    bool rpm_fresh = false;

    for (const int motor : motors) {
      if (motor < 0) {
        continue;
      }
      double distance = 0.0;
      bool distance_is_fresh = false;
      if (titan_driver_->GetEncoderDistanceFresh(
          static_cast<uint8_t>(motor), distance, distance_is_fresh))
      {
        distance_sum += distance;
        ++distance_count;
        encoder_fresh = encoder_fresh || distance_is_fresh;
      }

      float rpm = 0.0f;
      bool rpm_is_fresh = false;
      if (titan_driver_->GetRPMFresh(static_cast<uint8_t>(motor), rpm, rpm_is_fresh)) {
        rpm_sum += static_cast<double>(rpm);
        ++rpm_count;
        rpm_fresh = rpm_fresh || rpm_is_fresh;
      }
    }

    if (distance_count > 0) {
      const double distance = distance_sum / static_cast<double>(distance_count);
      hw_positions_[i] = distance / wheel_radius_;
      if (encoder_fresh || !encoder_feedback_seen_[i]) {
        encoder_feedback_seen_[i] = true;
        last_encoder_feedback_times_[i] = steady_now;
      }
    }
    if (rpm_count > 0) {
      const double rpm = rpm_sum / static_cast<double>(rpm_count);
      hw_velocities_[i] = rpm * kRpmToRadPerSec;
      if (rpm_fresh || !rpm_feedback_seen_[i]) {
        rpm_feedback_seen_[i] = true;
        last_rpm_feedback_times_[i] = steady_now;
      }
    }

    const double encoder_age = encoder_feedback_seen_[i] ?
      std::chrono::duration<double>(steady_now - last_encoder_feedback_times_[i]).count() :
      std::numeric_limits<double>::infinity();
    const double rpm_age = rpm_feedback_seen_[i] ?
      std::chrono::duration<double>(steady_now - last_rpm_feedback_times_[i]).count() :
      std::numeric_limits<double>::infinity();
    hw_feedback_ages_[i] = std::max(encoder_age, rpm_age);
    hw_encoder_fresh_[i] = hw_feedback_ages_[i] <= feedback_warn_timeout_sec_ ? 1.0 : 0.0;

    if (
      velocity_pid_safety::stale_feedback_while_moving(
        hw_commanded_velocities_[i], hw_feedback_ages_[i], feedback_error_timeout_sec_))
    {
      latch_fault(
        "encoder/RPM feedback timeout on " + info_.joints[i].name +
        " (age=" + std::to_string(hw_feedback_ages_[i]) + " s)");
    }
  }

  float temperature_c = 0.0f;
  bool temperature_fresh = false;
  if (titan_driver_->GetControllerTempFresh(temperature_c, temperature_fresh)) {
    titan_controller_temperature_c_ = static_cast<double>(temperature_c);
    if (temperature_fresh || !temperature_seen_) {
      temperature_seen_ = true;
      last_temperature_time_ = steady_now;
    }
  }
  titan_temperature_age_sec_ = temperature_seen_ ?
    std::chrono::duration<double>(steady_now - last_temperature_time_).count() :
    std::numeric_limits<double>::infinity();
  const bool robot_moving = std::any_of(
    hw_commanded_velocities_.begin(), hw_commanded_velocities_.end(),
    [](double command) {
      return std::abs(command) > velocity_pid_safety::kMotionCommandEpsilon;
    });
  if (velocity_pid_safety::temperature_fault(
      robot_moving, temperature_seen_, titan_controller_temperature_c_,
      titan_temperature_age_sec_, controller_temp_error_c_,
      controller_temp_error_timeout_sec_))
  {
    if (
      temperature_seen_ && !velocity_pid_safety::safe_temperature_sample(
        titan_controller_temperature_c_, controller_temp_error_c_))
    {
      latch_fault(
        "Titan controller temperature is invalid or over limit (temperature=" +
        std::to_string(titan_controller_temperature_c_) + " C, limit=" +
        std::to_string(controller_temp_error_c_) + " C)");
    } else {
      latch_fault(
        "Titan temperature feedback is missing or stale while moving (age=" +
        std::to_string(titan_temperature_age_sec_) + " s, timeout=" +
        std::to_string(controller_temp_error_timeout_sec_) + " s)");
    }
  }

  if (fault_latched_) {
    enforce_fault_stop();
  }

  if (imu_enabled_ && imu_driver_) {
    imu_orientation_x_ = imu_driver_->GetQuaternionX();
    imu_orientation_y_ = imu_driver_->GetQuaternionY();
    imu_orientation_z_ = imu_driver_->GetQuaternionZ();
    imu_orientation_w_ = imu_driver_->GetQuaternionW();

    imu_angular_velocity_x_ = imu_driver_->GetRawGyroX() * kDegreesToRadians;
    imu_angular_velocity_y_ = imu_driver_->GetRawGyroY() * kDegreesToRadians;
    imu_angular_velocity_z_ = imu_driver_->GetRawGyroZ() * kDegreesToRadians;

    imu_linear_acceleration_x_ = imu_driver_->GetWorldLinearAccelX() * kGToMetersPerSecondSquared;
    imu_linear_acceleration_y_ = imu_driver_->GetWorldLinearAccelY() * kGToMetersPerSecondSquared;
    imu_linear_acceleration_z_ = imu_driver_->GetWorldLinearAccelZ() * kGToMetersPerSecondSquared;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VmxSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!titan_driver_) {
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in write()!");
    return hardware_interface::return_type::ERROR;
  }

  if (hw_commands_.size() < 2) {
    RCLCPP_ERROR(get_logger(), "Expected at least 2 command interfaces.");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    const auto limited = velocity_pid_safety::limit_command(
      hw_commands_[i], max_wheel_angular_velocity_rad_s_);
    if (!limited.finite) {
      latch_fault("non-finite velocity command on " + info_.joints[i].name);
      hw_commanded_velocities_[i] = 0.0;
      continue;
    }
    hw_commanded_velocities_[i] = limited.value;
    hw_command_saturated_[i] = limited.saturated ? 1.0 : 0.0;
    if (hw_command_saturated_[i] > 0.5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel command saturated for %s: requested=%.3f rad/s limited=%.3f rad/s",
        info_.joints[i].name.c_str(), hw_commands_[i], hw_commanded_velocities_[i]);
    }
  }

  if (fault_latched_) {
    std::fill(hw_commanded_velocities_.begin(), hw_commanded_velocities_.end(), 0.0);
    enforce_fault_stop();
    return hardware_interface::return_type::OK;
  }

  auto send_command = [&](int motor, double command_rad_s) -> bool {
    if (motor < 0) {
      return true;
    }
    if (control_mode_ == MotorControlMode::VELOCITY_PID) {
      const float target_rpm = velocity_pid_safety::rad_per_sec_to_rpm(command_rad_s);
      return titan_driver_->TrySetTargetVelocity(static_cast<uint8_t>(motor), target_rpm);
    }
    const double duty = std::clamp(
      (command_rad_s / max_wheel_angular_velocity_rad_s_) * speed_scale_, -1.0, 1.0);
    titan_driver_->SetSpeed(static_cast<uint8_t>(motor), duty);
    return true;
  };

  bool write_success = true;
  if (is_independent_motor_layout_) {
    for (size_t i = 0; i < hw_commanded_velocities_.size(); ++i) {
      const int motor = (i < joint_motor_indices_.size()) ? joint_motor_indices_[i] : -1;
      write_success = send_command(motor, hw_commanded_velocities_[i]) && write_success;
    }
  } else {
    write_success = send_command(left_front_motor_, hw_commanded_velocities_[0]) && write_success;
    write_success = send_command(left_rear_motor_, hw_commanded_velocities_[0]) && write_success;
    write_success = send_command(right_front_motor_, hw_commanded_velocities_[1]) && write_success;
    write_success = send_command(right_rear_motor_, hw_commanded_velocities_[1]) && write_success;
  }

  if (velocity_pid_safety::can_write_fault(write_success)) {
    latch_fault("one or more Titan CAN target writes failed");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace studica_vmxpi_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  studica_vmxpi_ros2::VmxSystemHardware, hardware_interface::SystemInterface)
