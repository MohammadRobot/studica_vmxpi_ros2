// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
// Simple reactive patrol behavior for classroom demos.
// The node reads LaserScan data and publishes TwistStamped commands.

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kDirectionOffsetRadians = kPi / 2.0;

constexpr int kLaserAngularRangeDegrees = 360;
constexpr int kForwardWindowStartDegrees = 90;
constexpr int kForwardWindowEndDegrees = 270;
constexpr int kCenterWindowStartDegrees = 140;
constexpr int kCenterWindowEndDegrees = 220;

constexpr double kNearObstacleDistanceMeters = 1.0;
constexpr double kMediumObstacleDistanceMeters = 2.0;
constexpr double kSlowLinearSpeedMetersPerSecond = 0.2;
constexpr double kCruiseLinearSpeedMetersPerSecond = 0.5;
constexpr int kDebugLogThrottleMs = 2000;
constexpr double kScanTimeoutSeconds = 1.0;
}  // namespace

class Patrol : public rclcpp::Node
{
public:
  Patrol()
  : Node("robot_patrol_node")
  {
    const auto cmd_vel_topic =
      this->declare_parameter<std::string>("cmd_vel_topic", "/robot_base_controller/cmd_vel");
    const auto scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);
    timer_ = this->create_wall_timer(200ms, std::bind(&Patrol::timer_callback, this));
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "Patrol using scan topic '%s' and cmd_vel topic '%s'",
      scan_topic.c_str(), cmd_vel_topic.c_str());
  }

private:
  static bool is_valid_range(
    float range_value, const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg)
  {
    return std::isfinite(range_value) && range_value >= scan_msg->range_min &&
           range_value <= scan_msg->range_max;
  }

  static double normalize_angle(double angle_radians)
  {
    return std::atan2(std::sin(angle_radians), std::cos(angle_radians));
  }

  double direction_from_degrees(double angle_degrees) const
  {
    const double angle_radians = angle_degrees * kDegreesToRadians;
    return normalize_angle(angle_radians) - kDirectionOffsetRadians;
  }

  int angle_to_scan_index(int angle_degrees, int range_count) const
  {
    const double scale =
      static_cast<double>(laser_scan_points_) / static_cast<double>(kLaserAngularRangeDegrees);
    const int point = static_cast<int>(std::lround(angle_degrees * scale));
    return std::clamp(point, 0, range_count - 1);
  }

  double scan_index_to_angle_degrees(int scan_index) const
  {
    return static_cast<double>(scan_index) * static_cast<double>(kLaserAngularRangeDegrees) /
           static_cast<double>(laser_scan_points_);
  }

  void timer_callback()
  {
    // Zero velocity if scan data has gone stale.
    if (last_scan_time_.nanoseconds() > 0) {
      const double age = (this->now() - last_scan_time_).seconds();
      if (age > kScanTimeoutSeconds) {
        linear_velocity_x_ = 0.0;
        angular_velocity_z_ = 0.0;
        direction_radians_ = 0.0;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), kDebugLogThrottleMs,
          "No scan data for %.1f s — stopping robot.", age);
      }
    }

    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";
    message.twist.linear.x = linear_velocity_x_;
    message.twist.angular.z = angular_velocity_z_;

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), kDebugLogThrottleMs,
      "Publishing patrol cmd: linear=%.3f angular=%.3f direction=%.3f",
      linear_velocity_x_, angular_velocity_z_, direction_radians_);

    publisher_->publish(message);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    const int range_count = static_cast<int>(scan_msg->ranges.size());
    if (range_count == 0) {
      return;
    }

    laser_scan_points_ = range_count;
    last_scan_time_ = this->now();

    double front_min_distance = scan_msg->range_max;
    double front_min_distance_angle = 0.0;
    double max_distance = 0.0;
    double max_distance_angle = 0.0;

    const int start_point = angle_to_scan_index(kForwardWindowStartDegrees, range_count);
    const int end_point = angle_to_scan_index(kForwardWindowEndDegrees, range_count);
    const int center_start_point = angle_to_scan_index(kCenterWindowStartDegrees, range_count);
    const int center_end_point = angle_to_scan_index(kCenterWindowEndDegrees, range_count);

    for (int i = start_point; i <= end_point; ++i) {
      const float range_value = scan_msg->ranges[static_cast<size_t>(i)];
      if (!is_valid_range(range_value, scan_msg)) {
        continue;
      }

      if (range_value > max_distance) {
        max_distance = range_value;
        max_distance_angle = scan_index_to_angle_degrees(i);
      }
    }

    for (int i = center_start_point; i <= center_end_point; ++i) {
      const float range_value = scan_msg->ranges[static_cast<size_t>(i)];
      if (!is_valid_range(range_value, scan_msg)) {
        continue;
      }

      if (range_value < front_min_distance) {
        front_min_distance = range_value;
        front_min_distance_angle = scan_index_to_angle_degrees(i);
      }
    }

    if (front_min_distance <= kNearObstacleDistanceMeters) {
      linear_velocity_x_ = kSlowLinearSpeedMetersPerSecond;
      direction_radians_ = direction_from_degrees(front_min_distance_angle);
      angular_velocity_z_ = -direction_radians_;
      return;
    }

    if (front_min_distance <= kMediumObstacleDistanceMeters) {
      linear_velocity_x_ = kCruiseLinearSpeedMetersPerSecond;
      direction_radians_ = direction_from_degrees(max_distance_angle);
      angular_velocity_z_ = direction_radians_ / 2.0;
      return;
    }

    linear_velocity_x_ = kCruiseLinearSpeedMetersPerSecond;
    direction_radians_ = 0.0;
    angular_velocity_z_ = 0.0;
  }

  double direction_radians_{0.0};
  double linear_velocity_x_{0.0};
  double angular_velocity_z_{0.0};

  int laser_scan_points_{1};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
