#include <gtest/gtest.h>

#include "studica_vmxpi_ros2/imu_conventions.hpp"

namespace imu = studica_vmxpi_ros2::imu_conventions;

TEST(ImuConventions, ConvertsSensorFrameAccelerationFromGToSi)
{
  EXPECT_DOUBLE_EQ(imu::acceleration_g_to_meters_per_second_squared(0.0), 0.0);
  EXPECT_DOUBLE_EQ(
    imu::acceleration_g_to_meters_per_second_squared(1.0),
    imu::kStandardGravityMetersPerSecondSquared);
  EXPECT_DOUBLE_EQ(
    imu::acceleration_g_to_meters_per_second_squared(-0.5),
    -0.5 * imu::kStandardGravityMetersPerSecondSquared);
}
