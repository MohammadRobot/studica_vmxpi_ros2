// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef STUDICA_VMXPI_ROS2__TOPIC_ADAPTER_HPP_
#define STUDICA_VMXPI_ROS2__TOPIC_ADAPTER_HPP_

#include <cstddef>
#include <limits>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace studica_vmxpi_ros2::topic_adapter
{

inline geometry_msgs::msg::TwistStamped stamp_twist(
  const geometry_msgs::msg::Twist & twist,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = stamp;
  stamped.header.frame_id = frame_id;
  stamped.twist = twist;
  return stamped;
}

inline nav_msgs::msg::Odometry alias_odometry(const nav_msgs::msg::Odometry & odometry)
{
  return odometry;
}

inline sensor_msgs::msg::Imu imu_from_odometry(
  const nav_msgs::msg::Odometry & odometry, const std::string & frame_id)
{
  sensor_msgs::msg::Imu imu;
  imu.header = odometry.header;
  imu.header.frame_id = frame_id;
  imu.orientation = odometry.pose.pose.orientation;
  imu.angular_velocity = odometry.twist.twist.angular;
  imu.orientation_covariance = {
    1000000.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0005};
  imu.angular_velocity_covariance = {
    1000000.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0005};
  imu.linear_acceleration_covariance = {
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5};
  return imu;
}

inline sensor_msgs::msg::LaserScan clear_mock_scan(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  std::size_t sample_count,
  double publish_rate)
{
  constexpr float kPi = 3.14159265358979323846F;
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = frame_id;
  scan.angle_min = -kPi;
  scan.angle_max = kPi;
  scan.angle_increment =
    (scan.angle_max - scan.angle_min) / static_cast<float>(sample_count - 1U);
  scan.time_increment = 0.0F;
  scan.scan_time = static_cast<float>(1.0 / publish_rate);
  scan.range_min = 0.10F;
  scan.range_max = 10.0F;
  scan.ranges.assign(sample_count, std::numeric_limits<float>::infinity());
  scan.intensities.assign(sample_count, 0.0F);
  return scan;
}

}  // namespace studica_vmxpi_ros2::topic_adapter

#endif  // STUDICA_VMXPI_ROS2__TOPIC_ADAPTER_HPP_
