// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
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
    output_topic_ = this->declare_parameter<std::string>(
      "output_cmd_vel_topic", "/robot_base_controller/cmd_vel");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/robot/state");
    reason_topic_ = this->declare_parameter<std::string>(
      "safety_reason_topic", "/robot/safety_reason");
    frame_id_ = this->declare_parameter<std::string>("cmd_vel_frame_id", "base_link");
    hardware_mode_ = this->declare_parameter<bool>("hardware_mode", false);
    allow_software_arm_ = this->declare_parameter<bool>("allow_software_arm", false);
    command_timeout_ = this->declare_parameter<double>("command_timeout_sec", 0.25);
    publish_rate_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    limits_.max_linear_x = this->declare_parameter<double>("max_linear_x", 0.5);
    limits_.max_linear_y = this->declare_parameter<double>("max_linear_y", 0.5);
    limits_.max_angular_z = this->declare_parameter<double>("max_angular_z", 1.5);
    max_linear_acceleration_ = this->declare_parameter<double>(
      "max_linear_acceleration", 1.0);
    max_angular_acceleration_ = this->declare_parameter<double>(
      "max_angular_acceleration", 3.0);
    validateParameters();
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
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&SafetySupervisorNode::commandCallback, this, std::placeholders::_1));
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
    const auto boot_result = state_machine_.complete_boot(true);
    publishState();
    publishReason("READY_DISARMED");
    publishZero();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = this->create_wall_timer(
      period, std::bind(&SafetySupervisorNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "%s. Input %s; sole controller output %s; software arm %s%s",
      boot_result.reason, input_topic_.c_str(), output_topic_.c_str(),
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
      !state_topic_.empty() && !reason_topic_.empty() && !frame_id_.empty();
    const bool rates_valid = std::isfinite(command_timeout_) && command_timeout_ > 0.0 &&
      command_timeout_ <= 0.5 && std::isfinite(publish_rate_) && publish_rate_ >= 10.0 &&
      publish_rate_ <= 200.0;
    const bool limits_valid = std::isfinite(limits_.max_linear_x) &&
      limits_.max_linear_x > 0.0 && std::isfinite(limits_.max_linear_y) &&
      limits_.max_linear_y > 0.0 && std::isfinite(limits_.max_angular_z) &&
      limits_.max_angular_z > 0.0 && std::isfinite(max_linear_acceleration_) &&
      max_linear_acceleration_ > 0.0 && std::isfinite(max_angular_acceleration_) &&
      max_angular_acceleration_ > 0.0;
    if (!topics_valid || !rates_valid || !limits_valid) {
      throw std::invalid_argument(
              "Safety supervisor topics must be non-empty; command_timeout_sec must be in "
              "(0, 0.5], publish_rate_hz in [10, 200], and limits must be positive");
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

  void commandCallback(const geometry_msgs::msg::Twist::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = fromMessage(*message);
    received_at_ = SteadyClock::now();
    has_command_ = true;
    evaluateAndPublish(received_at_);
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = SteadyClock::now();
    evaluateAndPublish(now);
    if (std::chrono::duration<double>(now - last_state_publish_time_).count() >= 1.0) {
      publishState();
      last_state_publish_time_ = now;
    }
  }

  void evaluateAndPublish(SteadyClock::time_point now)
  {
    const auto evaluation = safety::evaluate(
      state_machine_.state(), last_command_, has_command_, steadySeconds(received_at_),
      steadySeconds(now), this->count_publishers(input_topic_), command_timeout_, limits_);
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
    publishReason(safety::decision_name(evaluation.decision));
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
    const auto result = state_machine_.disarm();
    clearCommandAndStop();
    publishState();
    publishReason(result.accepted ? "READY_DISARMED" : "DISARM_REJECTED");
    response->success = result.accepted;
    response->message = result.reason;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string reason_topic_;
  std::string frame_id_;
  bool hardware_mode_{false};
  bool allow_software_arm_{false};
  double command_timeout_{0.25};
  double publish_rate_{50.0};
  safety::Limits limits_{};
  double max_linear_acceleration_{1.0};
  double max_angular_acceleration_{3.0};

  std::mutex mutex_;
  safety::RobotStateMachine state_machine_;
  safety::Command last_command_{};
  safety::Command last_output_{};
  bool has_command_{false};
  SteadyClock::time_point received_at_{};
  SteadyClock::time_point last_publish_time_;
  SteadyClock::time_point last_state_publish_time_{SteadyClock::now()};
  std::string last_reason_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
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
