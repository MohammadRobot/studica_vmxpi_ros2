// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef STUDICA_VMXPI_ROS2__SAFETY_SUPERVISOR_HPP_
#define STUDICA_VMXPI_ROS2__SAFETY_SUPERVISOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace studica_vmxpi_ros2::safety_supervisor
{

enum class RobotState
{
  BOOTING,
  READY_DISARMED,
  ARMED,
  FAULT,
  UPDATING,
  SHUTTING_DOWN,
};

inline const char * state_name(RobotState state)
{
  switch (state) {
    case RobotState::BOOTING:
      return "BOOTING";
    case RobotState::READY_DISARMED:
      return "READY_DISARMED";
    case RobotState::ARMED:
      return "ARMED";
    case RobotState::FAULT:
      return "FAULT";
    case RobotState::UPDATING:
      return "UPDATING";
    case RobotState::SHUTTING_DOWN:
      return "SHUTTING_DOWN";
  }
  return "UNKNOWN";
}

struct TransitionResult
{
  bool accepted{false};
  const char * reason{"transition rejected"};
};

struct ArmConditions
{
  bool local_authorized{false};
  bool drive_healthy{false};
  bool emergency_stop_ok{false};
  bool update_inactive{false};
};

enum class HardwareGateState
{
  WAITING_FOR_SAFE_RELEASE = 0,
  READY = 1,
  ENABLED = 2,
  FAULT_LATCHED = 3,
};

enum class HardwareFaultReason
{
  NONE = 0,
  INPUT_INVALID = 1,
  ESTOP_NOT_OK = 2,
  DRIVE_UNHEALTHY = 3,
  TIME_INVALID = 4,
};

struct HardwareSafetyStatus
{
  bool encoding_valid{false};
  bool input_valid{false};
  bool estop_ok{false};
  bool enable_active{false};
  bool drive_healthy{false};
  bool motion_enabled{false};
  HardwareGateState gate_state{HardwareGateState::FAULT_LATCHED};
  HardwareFaultReason fault_reason{HardwareFaultReason::INPUT_INVALID};
};

enum class JoystickDeadmanDecision
{
  ACTIVE,
  RELEASED,
  RELEASE_REQUIRED,
  INVALID,
};

struct JoystickDeadmanResult
{
  bool active{false};
  JoystickDeadmanDecision decision{JoystickDeadmanDecision::RELEASE_REQUIRED};
};

class JoystickDeadmanGate
{
public:
  JoystickDeadmanResult update(bool sample_valid, bool pressed) noexcept
  {
    if (!sample_valid) {
      invalidate();
      return {false, JoystickDeadmanDecision::INVALID};
    }
    if (!pressed) {
      release_required_ = false;
      active_ = false;
      return {false, JoystickDeadmanDecision::RELEASED};
    }
    if (release_required_) {
      active_ = false;
      return {false, JoystickDeadmanDecision::RELEASE_REQUIRED};
    }
    active_ = true;
    return {true, JoystickDeadmanDecision::ACTIVE};
  }

  void require_release() noexcept
  {
    release_required_ = true;
    active_ = false;
  }

  void invalidate() noexcept {require_release();}
  bool active() const noexcept {return active_;}

private:
  bool release_required_{true};
  bool active_{false};
};

inline HardwareSafetyStatus decode_hardware_safety(
  const std::vector<std::string> & names,
  const std::vector<double> & values)
{
  HardwareSafetyStatus status;
  constexpr std::size_t expected_interface_count = 7U;
  if (
    names.size() != expected_interface_count ||
    values.size() != expected_interface_count)
  {
    return status;
  }

  auto get_unique = [&](const std::string & expected, double & output) -> bool {
    std::size_t matches = 0U;
    for (std::size_t index = 0; index < names.size(); ++index) {
      if (names[index] == expected) {
        output = values[index];
        ++matches;
      }
    }
    return matches == 1U && std::isfinite(output);
  };
  const auto binary = [](double value) {return value == 0.0 || value == 1.0;};

  double input_valid = 0.0;
  double estop_ok = 0.0;
  double enable_active = 0.0;
  double drive_healthy = 0.0;
  double motion_enabled = 0.0;
  double gate_state = 0.0;
  double fault_reason = 0.0;
  if (
    !get_unique("input_valid", input_valid) ||
    !get_unique("estop_ok", estop_ok) ||
    !get_unique("enable_active", enable_active) ||
    !get_unique("drive_healthy", drive_healthy) ||
    !get_unique("motion_enabled", motion_enabled) ||
    !get_unique("gate_state", gate_state) ||
    !get_unique("fault_reason", fault_reason) ||
    !binary(input_valid) || !binary(estop_ok) || !binary(enable_active) ||
    !binary(drive_healthy) || !binary(motion_enabled))
  {
    return status;
  }

  if (
    std::trunc(gate_state) != gate_state || std::trunc(fault_reason) != fault_reason ||
    gate_state < static_cast<double>(HardwareGateState::WAITING_FOR_SAFE_RELEASE) ||
    gate_state > static_cast<double>(HardwareGateState::FAULT_LATCHED) ||
    fault_reason < static_cast<double>(HardwareFaultReason::NONE) ||
    fault_reason > static_cast<double>(HardwareFaultReason::TIME_INVALID))
  {
    return status;
  }
  const int gate_state_value = static_cast<int>(gate_state);
  const int fault_reason_value = static_cast<int>(fault_reason);

  status.input_valid = input_valid == 1.0;
  status.estop_ok = estop_ok == 1.0;
  status.enable_active = enable_active == 1.0;
  status.drive_healthy = drive_healthy == 1.0;
  status.motion_enabled = motion_enabled == 1.0;
  status.gate_state = static_cast<HardwareGateState>(gate_state_value);
  status.fault_reason = static_cast<HardwareFaultReason>(fault_reason_value);

  const bool is_fault = status.gate_state == HardwareGateState::FAULT_LATCHED;
  const bool fault_consistent = is_fault ?
    status.fault_reason != HardwareFaultReason::NONE :
    status.fault_reason == HardwareFaultReason::NONE;
  const bool motion_consistent = status.motion_enabled ==
    (status.gate_state == HardwareGateState::ENABLED);
  const bool enabled_consistent = status.gate_state != HardwareGateState::ENABLED ||
    (status.input_valid && status.estop_ok && status.enable_active && status.drive_healthy);
  const bool healthy_nonfault = is_fault ||
    (status.input_valid && status.estop_ok && status.drive_healthy);
  status.encoding_valid =
    fault_consistent && motion_consistent && enabled_consistent && healthy_nonfault;
  return status;
}

class RobotStateMachine
{
public:
  RobotState state() const noexcept {return state_;}
  bool motion_allowed() const noexcept {return state_ == RobotState::ARMED;}

  TransitionResult complete_boot(bool healthy) noexcept
  {
    if (state_ != RobotState::BOOTING) {
      return {false, "boot already completed"};
    }
    state_ = healthy ? RobotState::READY_DISARMED : RobotState::FAULT;
    return {true, healthy ? "boot complete; robot disarmed" : "boot health check failed"};
  }

  TransitionResult arm(const ArmConditions & conditions) noexcept
  {
    if (state_ != RobotState::READY_DISARMED) {
      return {false, "robot is not ready and disarmed"};
    }
    if (!conditions.local_authorized) {
      return {false, "local operator authorization is required"};
    }
    if (!conditions.drive_healthy) {
      return {false, "drive health is not valid"};
    }
    if (!conditions.emergency_stop_ok) {
      return {false, "emergency-stop input is not valid"};
    }
    if (!conditions.update_inactive) {
      return {false, "an update is active"};
    }
    state_ = RobotState::ARMED;
    return {true, "robot armed"};
  }

  TransitionResult disarm() noexcept
  {
    if (state_ == RobotState::READY_DISARMED) {
      return {true, "robot already disarmed"};
    }
    if (state_ != RobotState::ARMED) {
      return {false, "robot cannot enter ready-disarmed from its current state"};
    }
    state_ = RobotState::READY_DISARMED;
    return {true, "robot disarmed"};
  }

  void latch_fault() noexcept {state_ = RobotState::FAULT;}

  TransitionResult acknowledge_fault(bool faults_clear, bool local_authorized) noexcept
  {
    if (state_ != RobotState::FAULT) {
      return {false, "robot is not faulted"};
    }
    if (!faults_clear) {
      return {false, "critical faults are still active"};
    }
    if (!local_authorized) {
      return {false, "local operator acknowledgement is required"};
    }
    state_ = RobotState::READY_DISARMED;
    return {true, "fault acknowledged; robot disarmed"};
  }

  TransitionResult begin_update() noexcept
  {
    if (state_ != RobotState::READY_DISARMED) {
      return {false, "updates require the ready-disarmed state"};
    }
    state_ = RobotState::UPDATING;
    return {true, "update started"};
  }

  TransitionResult finish_update(bool healthy) noexcept
  {
    if (state_ != RobotState::UPDATING) {
      return {false, "no update is active"};
    }
    state_ = healthy ? RobotState::READY_DISARMED : RobotState::FAULT;
    return {true, healthy ? "update complete; robot disarmed" : "update health check failed"};
  }

  void begin_shutdown() noexcept {state_ = RobotState::SHUTTING_DOWN;}

private:
  RobotState state_{RobotState::BOOTING};
};

struct Command
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

struct Limits
{
  double max_linear_x{0.5};
  double max_linear_y{0.5};
  double max_angular_z{1.5};
};

enum class CommandDecision
{
  ACCEPTED,
  DISARMED,
  NO_COMMAND,
  SOURCE_MISSING,
  SOURCE_CONFLICT,
  STALE,
  INVALID_TIME,
  NONFINITE,
  UNSUPPORTED_3D,
};

inline const char * decision_name(CommandDecision decision)
{
  switch (decision) {
    case CommandDecision::ACCEPTED:
      return "COMMAND_ACCEPTED";
    case CommandDecision::DISARMED:
      return "MOTION_DISABLED";
    case CommandDecision::NO_COMMAND:
      return "WAITING_FOR_COMMAND";
    case CommandDecision::SOURCE_MISSING:
      return "COMMAND_SOURCE_LOST";
    case CommandDecision::SOURCE_CONFLICT:
      return "COMMAND_SOURCE_CONFLICT";
    case CommandDecision::STALE:
      return "COMMAND_STALE";
    case CommandDecision::INVALID_TIME:
      return "COMMAND_TIME_INVALID";
    case CommandDecision::NONFINITE:
      return "COMMAND_NONFINITE";
    case CommandDecision::UNSUPPORTED_3D:
      return "COMMAND_UNSUPPORTED_3D";
  }
  return "COMMAND_REJECTED";
}

struct Evaluation
{
  Command output{};
  CommandDecision decision{CommandDecision::NO_COMMAND};
};

inline bool finite_command(const Command & command) noexcept
{
  return std::isfinite(command.linear_x) && std::isfinite(command.linear_y) &&
         std::isfinite(command.linear_z) && std::isfinite(command.angular_x) &&
         std::isfinite(command.angular_y) && std::isfinite(command.angular_z);
}

inline Evaluation evaluate(
  RobotState state, const Command & command, bool has_command,
  double received_at, double now, std::size_t publisher_count,
  double timeout, const Limits & limits) noexcept
{
  if (state != RobotState::ARMED) {
    return {{}, CommandDecision::DISARMED};
  }
  if (!has_command) {
    return {{}, CommandDecision::NO_COMMAND};
  }
  if (publisher_count == 0U) {
    return {{}, CommandDecision::SOURCE_MISSING};
  }
  if (publisher_count > 1U) {
    return {{}, CommandDecision::SOURCE_CONFLICT};
  }
  if (!std::isfinite(received_at) || !std::isfinite(now) || now < received_at) {
    return {{}, CommandDecision::INVALID_TIME};
  }
  if ((now - received_at) > timeout) {
    return {{}, CommandDecision::STALE};
  }
  if (!finite_command(command)) {
    return {{}, CommandDecision::NONFINITE};
  }
  constexpr double kPlanarEpsilon = 1e-9;
  if (std::fabs(command.linear_z) > kPlanarEpsilon ||
      std::fabs(command.angular_x) > kPlanarEpsilon ||
      std::fabs(command.angular_y) > kPlanarEpsilon)
  {
    return {{}, CommandDecision::UNSUPPORTED_3D};
  }

  Command output;
  output.linear_x = std::clamp(command.linear_x, -limits.max_linear_x, limits.max_linear_x);
  output.linear_y = std::clamp(command.linear_y, -limits.max_linear_y, limits.max_linear_y);
  output.angular_z = std::clamp(command.angular_z, -limits.max_angular_z, limits.max_angular_z);
  return {output, CommandDecision::ACCEPTED};
}

inline double rate_limit(double target, double previous, double max_rate, double elapsed) noexcept
{
  if (!std::isfinite(target) || !std::isfinite(previous) ||
      !std::isfinite(max_rate) || !std::isfinite(elapsed) ||
      max_rate <= 0.0 || elapsed <= 0.0)
  {
    return previous;
  }
  const double maximum_step = max_rate * elapsed;
  return previous + std::clamp(target - previous, -maximum_step, maximum_step);
}

inline Command rate_limit_planar(
  const Command & target, const Command & previous, double elapsed,
  double max_linear_acceleration, double max_angular_acceleration) noexcept
{
  Command output;
  output.linear_x = rate_limit(
    target.linear_x, previous.linear_x, max_linear_acceleration, elapsed);
  output.linear_y = rate_limit(
    target.linear_y, previous.linear_y, max_linear_acceleration, elapsed);
  output.angular_z = rate_limit(
    target.angular_z, previous.angular_z, max_angular_acceleration, elapsed);
  return output;
}

}  // namespace studica_vmxpi_ros2::safety_supervisor

#endif  // STUDICA_VMXPI_ROS2__SAFETY_SUPERVISOR_HPP_
