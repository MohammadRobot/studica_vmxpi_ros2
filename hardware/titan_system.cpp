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

#include "vmxpi_ros2/titan_system.hpp"

#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>
#include <math.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vmxpi_ros2
{
hardware_interface::CallbackReturn TitanSystemHardware::on_init(
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

  can_id_ = static_cast<uint8_t>(can_id);
  motor_freq_ = static_cast<uint16_t>(motor_freq);
  ticks_per_rotation_ = ticks_per_rotation;
  wheel_radius_ = wheel_radius;

  if (!get_double_param("speed_scale", speed_scale_, false)) {
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

  dist_per_tick_ = 2.0 * M_PI * wheel_radius_ / static_cast<double>(ticks_per_rotation_);

  try {
    titan_driver_ = std::make_unique<studica_driver::Titan>(
      can_id_, motor_freq_, static_cast<float>(dist_per_tick_));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error initializing Titan driver: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Titan driver initialized in on_init");

  // Initialize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
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
  }

  auto configure_encoder = [&](int motor) {
    if (motor < 0) {
      return;
    }
    titan_driver_->ConfigureEncoder(static_cast<uint8_t>(motor), dist_per_tick_);
    titan_driver_->ResetEncoder(static_cast<uint8_t>(motor));
  };

  configure_encoder(left_front_motor_);
  configure_encoder(left_rear_motor_);
  configure_encoder(right_front_motor_);
  configure_encoder(right_rear_motor_);

  if (invert_left_front_encoder_ && left_front_motor_ >= 0) {
    titan_driver_->InvertEncoderDirection(static_cast<uint8_t>(left_front_motor_));
  }
  if (invert_left_rear_encoder_ && left_rear_motor_ >= 0) {
    titan_driver_->InvertEncoderDirection(static_cast<uint8_t>(left_rear_motor_));
  }
  if (invert_right_front_encoder_ && right_front_motor_ >= 0) {
    titan_driver_->InvertEncoderDirection(static_cast<uint8_t>(right_front_motor_));
  }
  if (invert_right_rear_encoder_ && right_rear_motor_ >= 0) {
    titan_driver_->InvertEncoderDirection(static_cast<uint8_t>(right_rear_motor_));
  }

  if (invert_left_front_motor_ && left_front_motor_ >= 0) {
    titan_driver_->InvertMotor(static_cast<uint8_t>(left_front_motor_));
  }
  if (invert_left_rear_motor_ && left_rear_motor_ >= 0) {
    titan_driver_->InvertMotor(static_cast<uint8_t>(left_rear_motor_));
  }
  if (invert_right_front_motor_ && right_front_motor_ >= 0) {
    titan_driver_->InvertMotor(static_cast<uint8_t>(right_front_motor_));
  }
  if (invert_right_rear_motor_ && right_rear_motor_ >= 0) {
    titan_driver_->InvertMotor(static_cast<uint8_t>(right_rear_motor_));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TitanSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TitanSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TitanSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!titan_driver_) { // Safety check - should not happen if on_init is successful
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in on_activate!");
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

  titan_driver_->Enable(true);

  RCLCPP_INFO(get_logger(), "Titan Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TitanSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (titan_driver_) { // Check if titan_driver_ is valid before using
    titan_driver_->Enable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(5.0 * 1000))); //This Delay is just to check if the Titan does disable
  } else {
    RCLCPP_WARN(get_logger(), "Titan driver is not initialized in on_deactivate, nothing to disable.");
  }

  RCLCPP_INFO(get_logger(), "Titan Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TitanSystemHardware::read(
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

  const double left_distance = average_pair(left_front_motor_, left_rear_motor_, motor_distance);
  const double right_distance = average_pair(right_front_motor_, right_rear_motor_, motor_distance);

  if (wheel_radius_ > 0.0 && hw_positions_.size() >= 2) {
    hw_positions_[0] = left_distance / wheel_radius_;
    hw_positions_[1] = right_distance / wheel_radius_;
  }

  const double left_rpm = average_pair(left_front_motor_, left_rear_motor_, motor_rpm);
  const double right_rpm = average_pair(right_front_motor_, right_rear_motor_, motor_rpm);
  const double rpm_to_rad_s = 2.0 * M_PI / 60.0;

  if (hw_velocities_.size() >= 2) {
    hw_velocities_[0] = left_rpm * rpm_to_rad_s;
    hw_velocities_[1] = right_rpm * rpm_to_rad_s;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type vmxpi_ros2 ::TitanSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!titan_driver_) {
    RCLCPP_ERROR(get_logger(), "Titan driver is not initialized in write()!");
    return hardware_interface::return_type::ERROR;
  }

  if (hw_commands_.size() < 2) {
    RCLCPP_ERROR(get_logger(), "Expected at least 2 command interfaces for diff drive.");
    return hardware_interface::return_type::ERROR;
  }

  double left_cmd = hw_commands_[0];
  double right_cmd = hw_commands_[1];

  if (std::isnan(left_cmd)) {
    left_cmd = 0.0;
  }
  if (std::isnan(right_cmd)) {
    right_cmd = 0.0;
  }

  left_cmd *= speed_scale_;
  right_cmd *= speed_scale_;

  auto set_speed = [&](int motor, double value) {
    if (motor < 0) {
      return;
    }
    titan_driver_->SetSpeed(static_cast<uint8_t>(motor), value);
  };

  set_speed(left_front_motor_, left_cmd);
  set_speed(left_rear_motor_, left_cmd);
  set_speed(right_front_motor_, right_cmd);
  set_speed(right_rear_motor_, right_cmd);

  return hardware_interface::return_type::OK;
}

}// namespace vmxpi_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vmxpi_ros2::TitanSystemHardware, hardware_interface::SystemInterface)
