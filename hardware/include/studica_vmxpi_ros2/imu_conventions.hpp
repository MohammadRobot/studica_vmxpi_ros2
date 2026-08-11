#ifndef STUDICA_VMXPI_ROS2__IMU_CONVENTIONS_HPP_
#define STUDICA_VMXPI_ROS2__IMU_CONVENTIONS_HPP_

namespace studica_vmxpi_ros2::imu_conventions
{

/// Standard gravity used to convert the VMX accelerometer output from g to SI.
constexpr double kStandardGravityMetersPerSecondSquared = 9.80665;

/// Convert a sensor-frame acceleration sample in g to metres per second squared.
constexpr double acceleration_g_to_meters_per_second_squared(double acceleration_g)
{
  return acceleration_g * kStandardGravityMetersPerSecondSquared;
}

}  // namespace studica_vmxpi_ros2::imu_conventions

#endif  // STUDICA_VMXPI_ROS2__IMU_CONVENTIONS_HPP_
