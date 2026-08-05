// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace studica_vmxpi_ros2::velocity_pid_safety
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadPerSecToRpm = 60.0 / (2.0 * kPi);
constexpr double kMotionCommandEpsilon = 1e-3;

struct LimitedCommand
{
  double value{0.0};
  bool finite{true};
  bool saturated{false};
};

inline LimitedCommand limit_command(double command_rad_s, double maximum_rad_s)
{
  if (!std::isfinite(command_rad_s) || !std::isfinite(maximum_rad_s) || maximum_rad_s <= 0.0) {
    return {0.0, false, false};
  }
  const double limited = std::clamp(command_rad_s, -maximum_rad_s, maximum_rad_s);
  return {limited, true, std::abs(command_rad_s - limited) > 1e-9};
}

inline float rad_per_sec_to_rpm(double command_rad_s)
{
  return static_cast<float>(command_rad_s * kRadPerSecToRpm);
}

inline bool valid_pid_config(const std::string & type, int sensitivity)
{
  return type == "mcv2" && sensitivity >= 0 && sensitivity <= 10;
}

inline bool firmware_supports_mcv2(uint8_t firmware_major)
{
  return firmware_major >= 2u;
}

inline bool safe_temperature_sample(double temperature_c, double error_temperature_c)
{
  return std::isfinite(temperature_c) && std::isfinite(error_temperature_c) &&
         temperature_c < error_temperature_c;
}

inline bool stale_feedback_while_moving(
  double command_rad_s, double feedback_age_sec, double error_timeout_sec)
{
  return std::abs(command_rad_s) > kMotionCommandEpsilon &&
         (!std::isfinite(feedback_age_sec) || feedback_age_sec > error_timeout_sec);
}

inline bool temperature_fault(
  bool moving, bool temperature_seen, double temperature_c, double temperature_age_sec,
  double error_temperature_c, double error_timeout_sec)
{
  if (temperature_seen && !safe_temperature_sample(temperature_c, error_temperature_c))
  {
    return true;
  }
  return moving &&
         (!temperature_seen || !std::isfinite(temperature_age_sec) ||
         temperature_age_sec > error_timeout_sec);
}

inline bool can_write_fault(bool write_succeeded)
{
  return !write_succeeded;
}
}  // namespace studica_vmxpi_ros2::velocity_pid_safety
