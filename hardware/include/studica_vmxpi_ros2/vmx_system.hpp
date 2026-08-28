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

#ifndef STUDICA_VMXPI_ROS2__VMX_SYSTEM_HPP_
#define STUDICA_VMXPI_ROS2__VMX_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <limits>
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

#include "studica_vmxpi_ros2/local_enable_gate.hpp"

#include "dio.h"
#include "imu.h"
#include "titan.h"

namespace studica_vmxpi_ros2
{
class VmxSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VmxSystemHardware)

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
  enum class MotorControlMode { OPEN_LOOP, VELOCITY_PID };

  void latch_fault(const std::string & reason);
  bool stop_all_motors();
  void enforce_fault_stop();
  bool wait_for_safe_controller_temperature();
  bool drive_healthy() const noexcept;
  local_enable_gate::Result update_local_enable_gate();
  void update_local_enable_state_interfaces(
    const local_enable_gate::Inputs & inputs,
    const local_enable_gate::Result & result) noexcept;
 
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<VMXPi> vmx_;
  std::unique_ptr<studica_driver::Titan> titan_driver_;
  std::unique_ptr<studica_driver::Imu> imu_driver_;
  std::unique_ptr<studica_driver::DIO> estop_ok_input_;
  std::unique_ptr<studica_driver::DIO> local_enable_input_;
  std::unique_ptr<local_enable_gate::LocalEnableGate> local_enable_gate_;
  local_enable_gate::Config local_enable_gate_config_;
  std::vector<double> hw_positions_; // Store current joint positions
  std::vector<double> hw_velocities_; // Store current joint velocities
  std::vector<double> hw_commands_; // Store commanded joint velocities
  std::vector<double> hw_commanded_velocities_;
  std::vector<double> hw_feedback_ages_;
  std::vector<double> hw_encoder_fresh_;
  std::vector<double> hw_command_saturated_;
  std::vector<int> joint_motor_indices_; // Per-joint motor mapping for 4-wheel layouts
  std::vector<std::chrono::steady_clock::time_point> last_rpm_feedback_times_;
  std::vector<std::chrono::steady_clock::time_point> last_encoder_feedback_times_;
  std::vector<bool> rpm_feedback_seen_;
  std::vector<bool> encoder_feedback_seen_;

  uint8_t can_id_{0};
  uint16_t motor_freq_{0};
  int ticks_per_rotation_{0};
  double wheel_radius_{0.0};
  double dist_per_tick_{0.0};
  double speed_scale_{1.0};
  double max_wheel_angular_velocity_rad_s_{20.0};
  bool is_independent_motor_layout_{false};
  MotorControlMode control_mode_{MotorControlMode::OPEN_LOOP};
  std::string control_mode_name_{"open_loop"};
  uint8_t pid_type_{TITAN_PID_TYPE_MCV2};
  uint8_t pid_sensitivity_{5};
  bool pid_require_supported_{true};
  bool wheel_radius_calibrated_{true};
  double feedback_warn_timeout_sec_{0.1};
  double feedback_error_timeout_sec_{0.25};
  double controller_temp_error_c_{80.0};
  double controller_temp_error_timeout_sec_{3.0};
  bool fault_latched_{false};
  std::string fault_reason_;
  std::chrono::steady_clock::time_point last_fault_stop_time_{};
  bool titan_output_enabled_{false};

  int estop_ok_dio_channel_{-1};
  int local_enable_dio_channel_{-1};

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

  bool imu_enabled_{false};
  std::string imu_sensor_name_{"imu_sensor"};
  std::string titan_sensor_name_{"titan_controller"};
  std::string safety_sensor_name_{"hardware_safety"};
  double titan_controller_temperature_c_{std::numeric_limits<double>::quiet_NaN()};
  double titan_temperature_age_sec_{std::numeric_limits<double>::infinity()};
  double titan_pid_supported_{0.0};
  double titan_pid_type_{0.0};
  double titan_fault_latched_{0.0};
  double titan_firmware_major_{0.0};
  double titan_firmware_minor_{0.0};
  double titan_firmware_patch_{0.0};
  double safety_input_valid_{0.0};
  double safety_estop_ok_{0.0};
  double safety_enable_active_{0.0};
  double safety_drive_healthy_{0.0};
  double safety_motion_enabled_{0.0};
  double safety_gate_state_{0.0};
  double safety_fault_reason_{0.0};
  std::chrono::steady_clock::time_point last_temperature_time_{};
  bool temperature_seen_{false};
  double imu_orientation_x_{0.0};
  double imu_orientation_y_{0.0};
  double imu_orientation_z_{0.0};
  double imu_orientation_w_{1.0};
  double imu_angular_velocity_x_{0.0};
  double imu_angular_velocity_y_{0.0};
  double imu_angular_velocity_z_{0.0};
  double imu_linear_acceleration_x_{0.0};
  double imu_linear_acceleration_y_{0.0};
  double imu_linear_acceleration_z_{0.0};
};

}  // namespace studica_vmxpi_ros2

#endif  // STUDICA_VMXPI_ROS2__VMX_SYSTEM_HPP_
