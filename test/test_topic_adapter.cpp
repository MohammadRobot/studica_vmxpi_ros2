// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "studica_vmxpi_ros2/topic_adapter.hpp"

namespace adapter = studica_vmxpi_ros2::topic_adapter;

TEST(TopicAdapter, ConvertsPublicTwistToStampedControllerCommand)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.25;
  command.angular.z = -0.5;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;

  const auto result = adapter::stamp_twist(command, stamp, "base_link");

  EXPECT_EQ(result.header.stamp, stamp);
  EXPECT_EQ(result.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(result.twist.linear.x, 0.25);
  EXPECT_DOUBLE_EQ(result.twist.angular.z, -0.5);
}

TEST(TopicAdapter, OdometryAliasPreservesMessage)
{
  nav_msgs::msg::Odometry input;
  input.header.frame_id = "odom";
  input.child_frame_id = "base_footprint";
  input.pose.pose.position.x = 1.5;
  input.twist.twist.linear.x = 0.2;

  const auto result = adapter::alias_odometry(input);

  EXPECT_EQ(result.header.frame_id, "odom");
  EXPECT_EQ(result.child_frame_id, "base_footprint");
  EXPECT_DOUBLE_EQ(result.pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(result.twist.twist.linear.x, 0.2);
}

TEST(TopicAdapter, ImuFallbackUsesOdometryOrientationAndAngularVelocity)
{
  nav_msgs::msg::Odometry input;
  input.pose.pose.orientation.w = 1.0;
  input.twist.twist.angular.z = 0.4;

  const auto result = adapter::imu_from_odometry(input, "imu_link");

  EXPECT_EQ(result.header.frame_id, "imu_link");
  EXPECT_DOUBLE_EQ(result.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(result.angular_velocity.z, 0.4);
  EXPECT_GT(result.orientation_covariance[0], 1000.0);
}

TEST(TopicAdapter, MockScanIsAValidClearScan)
{
  builtin_interfaces::msg::Time stamp;
  const auto result = adapter::clear_mock_scan(stamp, "laser_scan_frame", 180U, 5.0);

  EXPECT_EQ(result.header.frame_id, "laser_scan_frame");
  EXPECT_EQ(result.ranges.size(), 180U);
  EXPECT_EQ(result.intensities.size(), 180U);
  EXPECT_TRUE(std::isinf(result.ranges.front()));
  EXPECT_GT(result.angle_increment, 0.0F);
  EXPECT_FLOAT_EQ(result.scan_time, 0.2F);
}
