// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "studica_vmxpi_ros2/safety_supervisor.hpp"

namespace safety = studica_vmxpi_ros2::safety_supervisor;

class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode()
  : Node("safety_supervisor"), last_publish_time_(SteadyClock::now())
  {
    input_topic_ = this->declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    control_source_ = this->declare_parameter<std::string>("control_source", "application");
    joystick_input_topic_ = this->declare_parameter<std::string>(
      "joystick_cmd_vel_topic", "/cmd_vel/joy");
    joystick_state_topic_ = this->declare_parameter<std::string>(
      "joystick_state_topic", "/joy");
    joystick_deadman_button_ = this->declare_parameter<int64_t>(
      "joystick_deadman_button", 4);
    joystick_state_timeout_ = this->declare_parameter<double>(
      "joystick_state_timeout_sec", 0.25);
    output_topic_ = this->declare_parameter<std::string>(
      "output_cmd_vel_topic", "/robot_base_controller/cmd_vel");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/robot/state");
    reason_topic_ = this->declare_parameter<std::string>(
      "safety_reason_topic", "/robot/safety_reason");
    frame_id_ = this->declare_parameter<std::string>("cmd_vel_frame_id", "base_link");
    hardware_mode_ = this->declare_parameter<bool>("hardware_mode", false);
    allow_software_arm_ = this->declare_parameter<bool>("allow_software_arm", false);
    command_timeout_ = this->declare_parameter<double>("command_timeout_sec", 0.25);
    hardware_state_timeout_ = this->declare_parameter<double>(
      "hardware_state_timeout_sec", 0.5);
    publish_rate_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    limits_.max_linear_x = this->declare_parameter<double>("max_linear_x", 0.5);
    limits_.max_linear_y = this->declare_parameter<double>("max_linear_y", 0.5);
    limits_.max_angular_z = this->declare_parameter<double>("max_angular_z", 1.5);
    max_linear_acceleration_ = this->declare_parameter<double>(
      "max_linear_acceleration", 1.0);
    max_angular_acceleration_ = this->declare_parameter<double>(
      "max_angular_acceleration", 3.0);
    validateParameters();
    joystick_source_ = control_source_ == "joystick";
    active_input_topic_ = joystick_source_ ? joystick_input_topic_ : input_topic_;
    hardware_disarm_inhibit_ = hardware_mode_;
    parameter_callback_ = this->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & /* parameters */) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "safety-supervisor parameters are immutable after startup";
        return result;
      });

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_publisher_ = this->create_publisher<std_msgs::msg::String>(state_topic_, status_qos);
    reason_publisher_ = this->create_publisher<std_msgs::msg::String>(reason_topic_, status_qos);
    command_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    command_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      active_input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&SafetySupervisorNode::commandCallback, this, std::placeholders::_1));
    if (joystick_source_) {
      joystick_state_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joystick_state_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(
          &SafetySupervisorNode::joystickStateCallback, this, std::placeholders::_1));
    }
    if (hardware_mode_) {
      hardware_state_subscription_ =
        this->create_subscription<control_msgs::msg::DynamicJointState>(
        "/dynamic_joint_states", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(
          &SafetySupervisorNode::hardwareStateCallback, this, std::placeholders::_1));
    }
    arm_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/robot/arm",
      std::bind(
        &SafetySupervisorNode::armCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    disarm_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/robot/disarm",
      std::bind(
        &SafetySupervisorNode::disarmCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    publishState();
    std::string boot_reason;
    if (hardware_mode_) {
      boot_reason = "waiting for local hardware safety state";
      publishReason("WAITING_FOR_HARDWARE_SAFETY");
    } else {
      const auto boot_result = state_machine_.complete_boot(true);
      boot_reason = boot_result.reason;
      publishState();
      publishReason("READY_DISARMED");
    }
    publishZero();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = this->create_wall_timer(
      period, std::bind(&SafetySupervisorNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "%s. %s input %s; sole controller output %s; software arm %s%s",
      boot_reason.c_str(), control_source_.c_str(), active_input_topic_.c_str(),
      output_topic_.c_str(),
      allow_software_arm_ ? "enabled" : "disabled",
      hardware_mode_ ? " (hardware mode)" : "");
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  static double steadySeconds(SteadyClock::time_point point) noexcept
  {
    return std::chrono::duration<double>(point.time_since_epoch()).count();
  }

  void validateParameters() const
  {
    const bool topics_valid = !input_topic_.empty() && !output_topic_.empty() &&
      !joystick_input_topic_.empty() && !joystick_state_topic_.empty() &&
      !state_topic_.empty() && !reason_topic_.empty() && !frame_id_.empty();
    const bool rates_valid = std::isfinite(command_timeout_) && command_timeout_ > 0.0 &&
      command_timeout_ <= 0.5 && std::isfinite(publish_rate_) && publish_rate_ >= 10.0 &&
      publish_rate_ <= 200.0 && std::isfinite(hardware_state_timeout_) &&
      hardware_state_timeout_ >= 0.1 && hardware_state_timeout_ <= 2.0 &&
      std::isfinite(joystick_state_timeout_) && joystick_state_timeout_ >= 0.1 &&
      joystick_state_timeout_ <= 0.5;
    const bool limits_valid = std::isfinite(limits_.max_linear_x) &&
      limits_.max_linear_x > 0.0 && std::isfinite(limits_.max_linear_y) &&
      limits_.max_linear_y > 0.0 && std::isfinite(limits_.max_angular_z) &&
      limits_.max_angular_z > 0.0 && std::isfinite(max_linear_acceleration_) &&
      max_linear_acceleration_ > 0.0 && std::isfinite(max_angular_acceleration_) &&
      max_angular_acceleration_ > 0.0;
    const bool source_valid = control_source_ == "application" || control_source_ == "joystick";
    const bool deadman_valid = joystick_deadman_button_ >= 0 && joystick_deadman_button_ <= 31;
    if (!topics_valid || !rates_valid || !limits_valid || !source_valid || !deadman_valid) {
      throw std::invalid_argument(
              "Safety supervisor topics must be non-empty; control_source must be application "
              "or joystick; joystick_deadman_button must be in [0, 31]; command_timeout_sec "
              "must be in "
              "(0, 0.5], publish_rate_hz in [10, 200], hardware_state_timeout_sec must be "
              "in [0.1, 2.0], joystick_state_timeout_sec must be in [0.1, 0.5], and limits "
              "must be positive");
    }
    if (hardware_mode_ && allow_software_arm_) {
      throw std::invalid_argument("Software arming is forbidden in hardware mode");
    }
  }

  safety::Command fromMessage(const geometry_msgs::msg::Twist & message) const noexcept
  {
    return {
      message.linear.x, message.linear.y, message.linear.z,
      message.angular.x, message.angular.y, message.angular.z};
  }

  geometry_msgs::msg::TwistStamped toMessage(const safety::Command & command)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = this->now();
    message.header.frame_id = frame_id_;
    message.twist.linear.x = command.linear_x;
    message.twist.linear.y = command.linear_y;
    message.twist.angular.z = command.angular_z;
    return message;
  }

  void publishState()
  {
    std_msgs::msg::String message;
    message.data = safety::state_name(state_machine_.state());
    state_publisher_->publish(message);
  }

  void publishReason(const std::string & reason)
  {
    if (reason == last_reason_) {
      return;
    }
    last_reason_ = reason;
    std_msgs::msg::String message;
    message.data = reason;
    reason_publisher_->publish(message);
  }

  void publishZero()
  {
    last_output_ = {};
    last_publish_time_ = SteadyClock::now();
    command_publisher_->publish(toMessage(last_output_));
  }

  void clearCommandAndStop()
  {
    has_command_ = false;
    last_command_ = {};
    publishZero();
  }

  void publishCommandReason(const std::string & reason)
  {
    if (!hardware_mode_ || state_machine_.state() == safety::RobotState::ARMED) {
      publishReason(reason);
    }
  }

  void resetJoystickForArm()
  {
    if (joystick_source_) {
      joystick_deadman_gate_.require_release();
      has_command_ = false;
      last_command_ = {};
    }
  }

  void stopJoystick(const std::string & reason)
  {
    joystick_deadman_gate_.invalidate();
    clearCommandAndStop();
    publishCommandReason(reason);
  }

  bool joystickEligible(SteadyClock::time_point now)
  {
    const std::size_t publisher_count = this->count_publishers(joystick_state_topic_);
    if (publisher_count != 1U) {
      stopJoystick(
        publisher_count == 0U ?
        "JOYSTICK_STATE_SOURCE_LOST" : "JOYSTICK_STATE_SOURCE_CONFLICT");
      return false;
    }
    if (!joystick_status_received_) {
      const double startup_age = std::chrono::duration<double>(now - node_started_at_).count();
      if (startup_age > joystick_state_timeout_) {
        stopJoystick("JOYSTICK_STATE_STARTUP_TIMEOUT");
      } else {
        clearCommandAndStop();
        publishCommandReason("WAITING_FOR_JOYSTICK_STATE");
      }
      return false;
    }
    const double state_age =
      std::chrono::duration<double>(now - last_joystick_status_time_).count();
    if (state_age > joystick_state_timeout_) {
      stopJoystick("JOYSTICK_STATE_STALE");
      return false;
    }
    if (!joystick_deadman_gate_.active()) {
      clearCommandAndStop();
      return false;
    }
    return true;
  }

  void commandCallback(const geometry_msgs::msg::Twist::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = SteadyClock::now();
    if (joystick_source_ && !joystickEligible(now)) {
      return;
    }
    last_command_ = fromMessage(*message);
    received_at_ = now;
    has_command_ = true;
    evaluateAndPublish(received_at_);
  }

  void joystickStateCallback(const sensor_msgs::msg::Joy::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto received = SteadyClock::now();
    joystick_status_received_ = true;
    last_joystick_status_time_ = received;

    const std::size_t publisher_count = this->count_publishers(joystick_state_topic_);
    if (publisher_count != 1U) {
      stopJoystick(
        publisher_count == 0U ?
        "JOYSTICK_STATE_SOURCE_LOST" : "JOYSTICK_STATE_SOURCE_CONFLICT");
      return;
    }

    const std::size_t button_index = static_cast<std::size_t>(joystick_deadman_button_);
    const bool button_present = button_index < message->buttons.size();
    const bool button_valid = button_present &&
      (message->buttons[button_index] == 0 || message->buttons[button_index] == 1);
    const bool was_active = joystick_deadman_gate_.active();
    const auto result = joystick_deadman_gate_.update(
      button_valid, button_valid && message->buttons[button_index] == 1);
    if (!result.active) {
      clearCommandAndStop();
      if (result.decision == safety::JoystickDeadmanDecision::INVALID) {
        publishCommandReason("JOYSTICK_STATE_MALFORMED");
      } else if (result.decision == safety::JoystickDeadmanDecision::RELEASE_REQUIRED) {
        publishCommandReason("JOYSTICK_DEADMAN_RELEASE_REQUIRED");
      } else {
        publishCommandReason("JOYSTICK_DEADMAN_RELEASED");
      }
      return;
    }
    if (!was_active) {
      clearCommandAndStop();
      publishCommandReason("JOYSTICK_DEADMAN_ACTIVE_WAITING_FOR_COMMAND");
    }
  }

  void latchHardwareFault(const std::string & reason)
  {
    const bool state_changed = state_machine_.state() != safety::RobotState::FAULT;
    state_machine_.latch_fault();
    resetJoystickForArm();
    if (state_changed) {
      clearCommandAndStop();
      publishState();
    }
    publishReason(reason);
  }

  void applyHardwareSafety(const safety::HardwareSafetyStatus & status)
  {
    if (!status.encoding_valid) {
      latchHardwareFault("HARDWARE_SAFETY_MALFORMED");
      return;
    }
    if (status.gate_state == safety::HardwareGateState::FAULT_LATCHED) {
      latchHardwareFault("HARDWARE_SAFETY_FAULT");
      return;
    }

    auto current_state = state_machine_.state();
    if (current_state == safety::RobotState::BOOTING) {
      state_machine_.complete_boot(true);
      clearCommandAndStop();
      publishState();
      current_state = state_machine_.state();
    }

    if (status.gate_state != safety::HardwareGateState::ENABLED) {
      hardware_disarm_inhibit_ = false;
      if (current_state == safety::RobotState::ARMED) {
        state_machine_.disarm();
        resetJoystickForArm();
        clearCommandAndStop();
        publishState();
      } else if (
        current_state == safety::RobotState::FAULT &&
        status.gate_state == safety::HardwareGateState::READY)
      {
        const auto acknowledged = state_machine_.acknowledge_fault(true, true);
        if (acknowledged.accepted) {
          clearCommandAndStop();
          publishState();
        }
      }
      publishReason(
        status.gate_state == safety::HardwareGateState::READY ?
        "HARDWARE_READY_LOCAL_ENABLE_OFF" :
        "WAITING_FOR_LOCAL_SAFE_RELEASE");
      return;
    }

    if (hardware_disarm_inhibit_) {
      if (state_machine_.state() == safety::RobotState::ARMED) {
        state_machine_.disarm();
        clearCommandAndStop();
        publishState();
      }
      publishReason("HARDWARE_DISARMED_WAITING_FOR_LOCAL_RELEASE");
      return;
    }
    if (state_machine_.state() == safety::RobotState::READY_DISARMED) {
      const safety::ArmConditions conditions{
        status.motion_enabled, status.drive_healthy, status.estop_ok, true};
      const auto armed = state_machine_.arm(conditions);
      if (armed.accepted) {
        resetJoystickForArm();
      }
      clearCommandAndStop();
      publishState();
      publishReason(
        armed.accepted ? "HARDWARE_ARMED_WAITING_FOR_COMMAND" : "HARDWARE_ARM_REJECTED");
    } else if (state_machine_.state() == safety::RobotState::FAULT) {
      publishReason("HARDWARE_FAULT_REQUIRES_LOCAL_RELEASE");
    }
  }

  void hardwareStateCallback(
    const control_msgs::msg::DynamicJointState::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto received = SteadyClock::now();
    hardware_status_received_ = true;
    last_hardware_status_time_ = received;

    const std::size_t publisher_count = this->count_publishers("/dynamic_joint_states");
    if (publisher_count != 1U) {
      latchHardwareFault(
        publisher_count == 0U ?
        "HARDWARE_SAFETY_SOURCE_LOST" : "HARDWARE_SAFETY_SOURCE_CONFLICT");
      return;
    }

    const control_msgs::msg::InterfaceValue * safety_interfaces = nullptr;
    std::size_t matches = 0U;
    for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
      if (message->joint_names[index] == "hardware_safety") {
        ++matches;
        if (index < message->interface_values.size()) {
          safety_interfaces = &message->interface_values[index];
        }
      }
    }
    if (matches != 1U || safety_interfaces == nullptr) {
      latchHardwareFault("HARDWARE_SAFETY_STATE_MISSING");
      return;
    }
    applyHardwareSafety(safety::decode_hardware_safety(
        safety_interfaces->interface_names, safety_interfaces->values));
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = SteadyClock::now();
    if (hardware_mode_) {
      const double startup_age = std::chrono::duration<double>(now - node_started_at_).count();
      const double state_age = hardware_status_received_ ?
        std::chrono::duration<double>(now - last_hardware_status_time_).count() :
        startup_age;
      const std::size_t publisher_count = this->count_publishers("/dynamic_joint_states");
      if (
        hardware_status_received_ && publisher_count != 1U)
      {
        latchHardwareFault(
          publisher_count == 0U ?
          "HARDWARE_SAFETY_SOURCE_LOST" : "HARDWARE_SAFETY_SOURCE_CONFLICT");
      } else if (state_age > hardware_state_timeout_) {
        latchHardwareFault(
          hardware_status_received_ ?
          "HARDWARE_SAFETY_STALE" : "HARDWARE_SAFETY_STARTUP_TIMEOUT");
      }
    }
    if (!joystick_source_ || joystickEligible(now)) {
      evaluateAndPublish(now);
    }
    if (std::chrono::duration<double>(now - last_state_publish_time_).count() >= 1.0) {
      publishState();
      last_state_publish_time_ = now;
    }
  }

  void evaluateAndPublish(SteadyClock::time_point now)
  {
    const std::size_t publisher_count = this->count_publishers(active_input_topic_);
    if (joystick_source_ && joystick_deadman_gate_.active() && publisher_count != 1U) {
      stopJoystick(
        publisher_count == 0U ?
        "COMMAND_SOURCE_LOST" : "COMMAND_SOURCE_CONFLICT");
      return;
    }
    const auto evaluation = safety::evaluate(
      state_machine_.state(), last_command_, has_command_, steadySeconds(received_at_),
      steadySeconds(now), publisher_count, command_timeout_, limits_);
    if (
      joystick_source_ &&
      (evaluation.decision == safety::CommandDecision::STALE ||
      evaluation.decision == safety::CommandDecision::INVALID_TIME ||
      evaluation.decision == safety::CommandDecision::NONFINITE ||
      evaluation.decision == safety::CommandDecision::UNSUPPORTED_3D))
    {
      stopJoystick(safety::decision_name(evaluation.decision));
      return;
    }
    safety::Command output;
    if (evaluation.decision == safety::CommandDecision::ACCEPTED) {
      const double elapsed = std::chrono::duration<double>(now - last_publish_time_).count();
      output = safety::rate_limit_planar(
        evaluation.output, last_output_, elapsed, max_linear_acceleration_,
        max_angular_acceleration_);
    }
    last_output_ = output;
    last_publish_time_ = now;
    command_publisher_->publish(toMessage(output));
    publishCommandReason(safety::decision_name(evaluation.decision));
  }

  void armCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!allow_software_arm_) {
      response->success = false;
      response->message = hardware_mode_ ?
        "software arming is forbidden on hardware; a local safety input is required" :
        "software arming is disabled";
      publishReason("ARM_REJECTED");
      return;
    }
    const safety::ArmConditions conditions{true, true, true, true};
    const auto result = state_machine_.arm(conditions);
    if (result.accepted) {
      resetJoystickForArm();
    }
    clearCommandAndStop();
    publishState();
    publishReason(result.accepted ? "ARMED_WAITING_FOR_COMMAND" : "ARM_REJECTED");
    response->success = result.accepted;
    response->message = result.reason;
  }

  void disarmCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hardware_mode_) {
      hardware_disarm_inhibit_ = true;
    }
    const auto result = state_machine_.disarm();
    resetJoystickForArm();
    clearCommandAndStop();
    publishState();
    publishReason(
      result.accepted && hardware_mode_ ?
      "HARDWARE_DISARMED_WAITING_FOR_LOCAL_RELEASE" :
      (result.accepted ? "READY_DISARMED" : "DISARM_REJECTED"));
    response->success = result.accepted;
    response->message = result.reason;
  }

  std::string input_topic_;
  std::string control_source_;
  std::string active_input_topic_;
  std::string joystick_input_topic_;
  std::string joystick_state_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string reason_topic_;
  std::string frame_id_;
  bool hardware_mode_{false};
  bool allow_software_arm_{false};
  bool joystick_source_{false};
  int64_t joystick_deadman_button_{4};
  double command_timeout_{0.25};
  double joystick_state_timeout_{0.25};
  double hardware_state_timeout_{0.5};
  double publish_rate_{50.0};
  safety::Limits limits_{};
  double max_linear_acceleration_{1.0};
  double max_angular_acceleration_{3.0};

  std::mutex mutex_;
  safety::RobotStateMachine state_machine_;
  safety::JoystickDeadmanGate joystick_deadman_gate_;
  safety::Command last_command_{};
  safety::Command last_output_{};
  bool has_command_{false};
  bool hardware_status_received_{false};
  bool joystick_status_received_{false};
  bool hardware_disarm_inhibit_{false};
  SteadyClock::time_point received_at_{};
  SteadyClock::time_point node_started_at_{SteadyClock::now()};
  SteadyClock::time_point last_hardware_status_time_{};
  SteadyClock::time_point last_joystick_status_time_{};
  SteadyClock::time_point last_publish_time_;
  SteadyClock::time_point last_state_publish_time_{SteadyClock::now()};
  std::string last_reason_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_state_subscription_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
    hardware_state_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
