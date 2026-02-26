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

#ifndef VMXPI_ROS2_TITAN_HPP_
#define VMXPI_ROS2_TITAN_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "titan.h"// Include Titan driver header

namespace studica_vmxpi_ros2
{
class TitanSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TitanSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
    
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
 
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::unique_ptr<studica_driver::Titan> titan_driver_;
  std::vector<double> hw_positions_; // Store current joint positions
  std::vector<double> hw_velocities_; // Store current joint velocities
  std::vector<double> hw_commands_; // Store commanded joint velocities

  uint8_t can_id_{0};
  uint16_t motor_freq_{0};
  int ticks_per_rotation_{0};
  double wheel_radius_{0.0};
  double dist_per_tick_{0.0};
  double speed_scale_{1.0};

  int left_front_motor_{-1};
  int left_rear_motor_{-1};
  int right_front_motor_{-1};
  int right_rear_motor_{-1};

  bool invert_left_front_motor_{false};
  bool invert_left_rear_motor_{false};
  bool invert_right_front_motor_{false};
  bool invert_right_rear_motor_{false};

  bool invert_left_front_encoder_{false};
  bool invert_left_rear_encoder_{false};
  bool invert_right_front_encoder_{false};
  bool invert_right_rear_encoder_{false};
};

}  // namespace studica_vmxpi_ros2

#endif  // VMXPI_ROS2_TITAN_HPP_
