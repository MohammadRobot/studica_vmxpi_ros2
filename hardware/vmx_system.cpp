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
constexpr double kSaturationWarnEpsilon = 1e-9;
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
  if (speed_scale_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "speed_scale must be >= 0.0.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (max_wheel_angular_velocity_rad_s_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "max_wheel_angular_velocity_rad_s must be > 0.0.");
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

  if (!info_.sensors.empty()) {
    if (info_.sensors.size() > 1) {
      RCLCPP_WARN(
        get_logger(),
        "Multiple sensors defined; using first sensor '%s' for IMU interfaces.",
        info_.sensors.front().name.c_str());
    }
    const auto & imu_sensor = info_.sensors.front();
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

  try {
    vmx_ = std::make_shared<VMXPi>(true, 50);
    if (!vmx_ || !vmx_->IsOpen()) {
      RCLCPP_ERROR(get_logger(), "Unable to open VMXPi device for VmxSystemHardware.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    titan_driver_ = std::make_unique<studica_driver::Titan>(
      can_id_, motor_freq_, static_cast<float>(dist_per_tick_), vmx_);
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
  joint_motor_indices_.assign(info_.joints.size(), -1);
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

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
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
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  if (imu_enabled_) {
    imu_driver_->ZeroYaw();
  }

  titan_driver_->Enable(true);

  RCLCPP_INFO(get_logger(), "Titan Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (titan_driver_) { // Check if titan_driver_ is valid before using
    titan_driver_->Enable(false);
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

  auto motor_distance = [&](int motor) -> double {
    if (motor < 0) {
      return 0.0;
    }
    return titan_driver_->GetEncoderDistance(static_cast<uint8_t>(motor));
  };

  auto motor_rpm = [&](int motor) -> double {
    if (motor < 0) {
      return 0.0;
    }
    return static_cast<double>(titan_driver_->GetRPM(static_cast<uint8_t>(motor)));
  };

  auto average_pair = [&](int a, int b, const auto & getter) -> double {
    if (a >= 0 && b >= 0) {
      return (getter(a) + getter(b)) / 2.0;
    }
    if (a >= 0) {
      return getter(a);
    }
    if (b >= 0) {
      return getter(b);
    }
    return 0.0;
  };

  if (is_independent_motor_layout_) {
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      const int motor = (i < joint_motor_indices_.size()) ? joint_motor_indices_[i] : -1;
      const double distance = motor_distance(motor);
      const double rpm = motor_rpm(motor);

      hw_positions_[i] = (wheel_radius_ > 0.0) ? (distance / wheel_radius_) : 0.0;
      hw_velocities_[i] = rpm * kRpmToRadPerSec;
    }
  } else {
    const double left_distance = average_pair(
      left_front_motor_, left_rear_motor_, motor_distance);
    const double right_distance = average_pair(
      right_front_motor_, right_rear_motor_, motor_distance);
    const double left_rpm = average_pair(left_front_motor_, left_rear_motor_, motor_rpm);
    const double right_rpm = average_pair(right_front_motor_, right_rear_motor_, motor_rpm);

    if (wheel_radius_ > 0.0 && hw_positions_.size() >= 2) {
      hw_positions_[0] = left_distance / wheel_radius_;
      hw_positions_[1] = right_distance / wheel_radius_;
    }

    if (hw_velocities_.size() >= 2) {
      hw_velocities_[0] = left_rpm * kRpmToRadPerSec;
      hw_velocities_[1] = right_rpm * kRpmToRadPerSec;
    }
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

  auto set_speed = [&](int motor, double value) {
    if (motor < 0) {
      return;
    }
    titan_driver_->SetSpeed(static_cast<uint8_t>(motor), value);
  };

  if (is_independent_motor_layout_) {
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      const int motor = (i < joint_motor_indices_.size()) ? joint_motor_indices_[i] : -1;
      if (motor < 0) {
        continue;
      }

      double cmd_rad_s = hw_commands_[i];
      if (std::isnan(cmd_rad_s)) {
        cmd_rad_s = 0.0;
      }

      const double cmd_scaled =
        (cmd_rad_s / max_wheel_angular_velocity_rad_s_) * speed_scale_;
      const double cmd = std::clamp(cmd_scaled, -1.0, 1.0);

      if (std::abs(cmd_scaled - cmd) > kSaturationWarnEpsilon) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Holonomic wheel command saturated for joint index %zu: raw=%.3f scaled=%.3f "
          "max_wheel_angular_velocity_rad_s=%.3f speed_scale=%.3f",
          i, cmd_rad_s, cmd_scaled, max_wheel_angular_velocity_rad_s_, speed_scale_);
      }

      set_speed(motor, cmd);
    }
  } else {
    double left_cmd_rad_s = hw_commands_[0];
    double right_cmd_rad_s = hw_commands_[1];

    if (std::isnan(left_cmd_rad_s)) {
      left_cmd_rad_s = 0.0;
    }
    if (std::isnan(right_cmd_rad_s)) {
      right_cmd_rad_s = 0.0;
    }

    const double left_cmd_scaled =
      (left_cmd_rad_s / max_wheel_angular_velocity_rad_s_) * speed_scale_;
    const double right_cmd_scaled =
      (right_cmd_rad_s / max_wheel_angular_velocity_rad_s_) * speed_scale_;

    const double left_cmd = std::clamp(left_cmd_scaled, -1.0, 1.0);
    const double right_cmd = std::clamp(right_cmd_scaled, -1.0, 1.0);

    if (
      std::abs(left_cmd_scaled - left_cmd) > kSaturationWarnEpsilon ||
      std::abs(right_cmd_scaled - right_cmd) > kSaturationWarnEpsilon)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel command saturated to Titan range [-1, 1]: raw(rad/s) left=%.3f right=%.3f, "
        "scaled left=%.3f right=%.3f, max_wheel_angular_velocity_rad_s=%.3f, speed_scale=%.3f",
        left_cmd_rad_s, right_cmd_rad_s, left_cmd_scaled, right_cmd_scaled,
        max_wheel_angular_velocity_rad_s_, speed_scale_);
    }

    set_speed(left_front_motor_, left_cmd);
    set_speed(left_rear_motor_, left_cmd);
    set_speed(right_front_motor_, right_cmd);
    set_speed(right_rear_motor_, right_cmd);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace studica_vmxpi_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  studica_vmxpi_ros2::VmxSystemHardware, hardware_interface::SystemInterface)
