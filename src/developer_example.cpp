// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("studica_cpp_example");
  const auto motion = node->declare_parameter<bool>("publish_motion", false);
  const auto speed = node->declare_parameter<double>("speed", 0.05);
  const auto duration = node->declare_parameter<double>("duration", 1.0);
  if (!std::isfinite(speed) || speed <= 0 || speed > 0.05 ||
    !std::isfinite(duration) || duration <= 0 || duration > 2.0)
  {
    RCLCPP_ERROR(node->get_logger(), "speed must be (0, 0.05]; duration must be (0, 2]");
    rclcpp::shutdown();
    return 2;
  }
  auto odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    [node](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "x=%.3f m vx=%.3f m/s",
        msg->pose.pose.position.x, msg->twist.twist.linear.x);
    });
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  if (motion) {
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  }
  const auto started = std::chrono::steady_clock::now();
  int result = 0;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    if (elapsed >= 1.0 + duration + 0.5) {
      break;
    }
    if (publisher) {
      if (node->count_publishers("/cmd_vel") != 1) {
        RCLCPP_ERROR(node->get_logger(), "Another command publisher is present");
        result = 1;
        break;
      }
      geometry_msgs::msg::Twist command;
      if (elapsed >= 1.0 && elapsed < 1.0 + duration) {
        command.linear.x = speed;
      }
      publisher->publish(command);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (publisher && rclcpp::ok()) {
    publisher->publish(geometry_msgs::msg::Twist());
  }
  rclcpp::shutdown();
  return result;
}
