#include <limits>

#include <gtest/gtest.h>

#include "studica_vmxpi_ros2/velocity_pid_safety.hpp"

namespace safety = studica_vmxpi_ros2::velocity_pid_safety;

TEST(VelocityPidSafety, ConvertsRobotFrameRadiansPerSecondToSignedRpm)
{
  EXPECT_NEAR(safety::rad_per_sec_to_rpm(2.0 * safety::kPi), 60.0, 1e-5);
  EXPECT_NEAR(safety::rad_per_sec_to_rpm(-safety::kPi), -30.0, 1e-5);
}

TEST(VelocityPidSafety, ValidatesMcv2PidConfigurationAndFirmware)
{
  EXPECT_TRUE(safety::valid_pid_config("mcv2", 5));
  EXPECT_FALSE(safety::valid_pid_config("legacy", 5));
  EXPECT_FALSE(safety::valid_pid_config("mcv2", 11));
  EXPECT_FALSE(safety::firmware_supports_mcv2(1));
  EXPECT_TRUE(safety::firmware_supports_mcv2(2));
}

TEST(VelocityPidSafety, ClampsCommandsAndRejectsNonfiniteValues)
{
  const auto normal = safety::limit_command(2.0, 20.0);
  EXPECT_TRUE(normal.finite);
  EXPECT_FALSE(normal.saturated);
  EXPECT_DOUBLE_EQ(normal.value, 2.0);

  const auto clamped = safety::limit_command(25.0, 20.0);
  EXPECT_TRUE(clamped.finite);
  EXPECT_TRUE(clamped.saturated);
  EXPECT_DOUBLE_EQ(clamped.value, 20.0);

  const auto invalid = safety::limit_command(
    std::numeric_limits<double>::quiet_NaN(), 20.0);
  EXPECT_FALSE(invalid.finite);
  EXPECT_DOUBLE_EQ(invalid.value, 0.0);
}

TEST(VelocityPidSafety, LatchesStaleMovingFeedbackAndTemperatureErrors)
{
  EXPECT_FALSE(safety::stale_feedback_while_moving(0.0, 10.0, 0.25));
  EXPECT_FALSE(safety::stale_feedback_while_moving(2.0, 0.1, 0.25));
  EXPECT_TRUE(safety::stale_feedback_while_moving(2.0, 0.3, 0.25));
  EXPECT_TRUE(safety::stale_feedback_while_moving(
    2.0, std::numeric_limits<double>::infinity(), 0.25));

  EXPECT_TRUE(safety::safe_temperature_sample(40.0, 80.0));
  EXPECT_FALSE(safety::safe_temperature_sample(85.0, 80.0));
  EXPECT_FALSE(safety::safe_temperature_sample(
    std::numeric_limits<double>::quiet_NaN(), 80.0));

  // Temperature is broadcast at about 1 Hz, so it uses its own 3-second timeout.
  EXPECT_TRUE(safety::temperature_fault(true, false, 0.0, 0.0, 80.0, 3.0));
  EXPECT_TRUE(safety::temperature_fault(false, true, 85.0, 0.01, 80.0, 3.0));
  EXPECT_FALSE(safety::temperature_fault(true, true, 40.0, 1.2, 80.0, 3.0));
  EXPECT_TRUE(safety::temperature_fault(true, true, 40.0, 3.1, 80.0, 3.0));
  EXPECT_FALSE(safety::temperature_fault(false, true, 40.0, 10.0, 80.0, 3.0));
}

TEST(VelocityPidSafety, CanFailureRequiresFaultAndSafeZeroPath)
{
  EXPECT_FALSE(safety::can_write_fault(true));
  EXPECT_TRUE(safety::can_write_fault(false));
}
