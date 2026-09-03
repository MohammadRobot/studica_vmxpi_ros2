// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef STUDICA_VMXPI_ROS2__LOCAL_ENABLE_GATE_HPP_
#define STUDICA_VMXPI_ROS2__LOCAL_ENABLE_GATE_HPP_

#include <cmath>
#include <stdexcept>

namespace studica_vmxpi_ros2::local_enable_gate
{

enum class GateState
{
  WAITING_FOR_SAFE_RELEASE,
  READY,
  ENABLED,
  FAULT_LATCHED,
};

inline const char * state_name(GateState state)
{
  switch (state) {
    case GateState::WAITING_FOR_SAFE_RELEASE:
      return "WAITING_FOR_SAFE_RELEASE";
    case GateState::READY:
      return "READY";
    case GateState::ENABLED:
      return "ENABLED";
    case GateState::FAULT_LATCHED:
      return "FAULT_LATCHED";
  }
  return "UNKNOWN";
}

enum class FaultReason
{
  NONE,
  INPUT_INVALID,
  ESTOP_NOT_OK,
  DRIVE_UNHEALTHY,
  TIME_INVALID,
};

inline const char * fault_name(FaultReason reason)
{
  switch (reason) {
    case FaultReason::NONE:
      return "NONE";
    case FaultReason::INPUT_INVALID:
      return "INPUT_INVALID";
    case FaultReason::ESTOP_NOT_OK:
      return "ESTOP_NOT_OK";
    case FaultReason::DRIVE_UNHEALTHY:
      return "DRIVE_UNHEALTHY";
    case FaultReason::TIME_INVALID:
      return "TIME_INVALID";
  }
  return "UNKNOWN";
}

struct Inputs
{
  bool sample_valid{false};
  bool estop_ok{false};
  bool drive_healthy{false};
  bool drive_fault_clearable{false};
  bool start_active{false};
  bool reset_active{false};
  bool stop_ok{false};
};

struct Config
{
  double button_debounce_sec{0.10};
  double safe_release_sec{0.50};
};

struct Result
{
  GateState state{GateState::WAITING_FOR_SAFE_RELEASE};
  FaultReason fault{FaultReason::NONE};
  bool motion_enabled{false};
};

class LocalEnableGate
{
public:
  explicit LocalEnableGate(const Config & config = {})
  : config_(config)
  {
    if (!std::isfinite(config_.button_debounce_sec) ||
      !std::isfinite(config_.safe_release_sec) ||
      config_.button_debounce_sec < 0.0 || config_.safe_release_sec < 0.0)
    {
      throw std::invalid_argument("Control-panel debounce durations must be finite and nonnegative");
    }
  }

  GateState state() const noexcept {return state_;}
  FaultReason fault() const noexcept {return fault_;}
  bool motion_enabled() const noexcept {return state_ == GateState::ENABLED;}

  Result update(const Inputs & inputs, double now) noexcept
  {
    if (!std::isfinite(now) || (have_time_ && now < last_update_time_)) {
      latch_fault(FaultReason::TIME_INVALID);
      return result();
    }
    have_time_ = true;
    last_update_time_ = now;

    if (state_ == GateState::FAULT_LATCHED) {
      // The operational drive_healthy signal remains false while the lower
      // level Titan fault latch is set. A separate clearable signal proves
      // that the original condition is gone and the zero/disable path works,
      // allowing a deliberate Reset press to acknowledge the fault.
      if (!inputs.sample_valid) {
        latch_fault(FaultReason::INPUT_INVALID);
        return result();
      }
      if (!inputs.estop_ok) {
        latch_fault(FaultReason::ESTOP_NOT_OK);
        return result();
      }
      if (!inputs.drive_fault_clearable) {
        latch_fault(FaultReason::DRIVE_UNHEALTHY);
        return result();
      }
    } else {
      if (!inputs.sample_valid) {
        latch_fault(FaultReason::INPUT_INVALID);
        return result();
      }
      if (!inputs.estop_ok) {
        latch_fault(FaultReason::ESTOP_NOT_OK);
        return result();
      }
      if (!inputs.drive_healthy) {
        latch_fault(FaultReason::DRIVE_UNHEALTHY);
        return result();
      }
    }

    // Only a fully valid, safe sample may contribute to a physical-button
    // debounce interval. Time spent with a failed input or drive must not
    // count as a Start or Reset acknowledgement.
    observe_button(
      inputs.start_active, now, have_start_observation_, last_start_active_,
      start_changed_at_);
    observe_button(
      inputs.reset_active, now, have_reset_observation_, last_reset_active_,
      reset_changed_at_);

    if (state_ == GateState::FAULT_LATCHED) {
      if (!inputs.reset_active) {
        fault_reset_release_seen_ = true;
      }
      const bool reset_debounced =
        inputs.reset_active &&
        now - reset_changed_at_ >= config_.button_debounce_sec;
      if (
        inputs.stop_ok && !inputs.start_active && fault_reset_release_seen_ &&
        reset_debounced)
      {
        // Reset only acknowledges the cleared fault. It never enables motion;
        // Reset and Start must be released for a complete safe interval before
        // a later, fresh Start press can arm the gate.
        state_ = GateState::WAITING_FOR_SAFE_RELEASE;
        fault_ = FaultReason::NONE;
        safe_release_observing_ = false;
      }
      return result();
    }

    if (!inputs.stop_ok) {
      // The NC Stop circuit opens when pressed or broken. Stopping is
      // deliberately immediate and requires a full safe release plus a new
      // Start edge before motion can be authorized again.
      state_ = GateState::WAITING_FOR_SAFE_RELEASE;
      fault_ = FaultReason::NONE;
      safe_release_observing_ = false;
      return result();
    }

    switch (state_) {
      case GateState::WAITING_FOR_SAFE_RELEASE:
        if (!inputs.start_active && !inputs.reset_active) {
          if (!safe_release_observing_) {
            safe_release_observing_ = true;
            safe_release_started_at_ = now;
          } else if (now - safe_release_started_at_ >= config_.safe_release_sec) {
            state_ = GateState::READY;
            fault_ = FaultReason::NONE;
            safe_release_observing_ = false;
          }
        } else {
          safe_release_observing_ = false;
        }
        break;
      case GateState::READY:
        if (
          !inputs.reset_active && inputs.start_active &&
          now - start_changed_at_ >= config_.button_debounce_sec)
        {
          state_ = GateState::ENABLED;
        }
        break;
      case GateState::ENABLED:
        // Start is momentary. Authorization remains latched after release and
        // is cleared only by Stop, E-stop, an invalid sample, a drive fault, or
        // process restart. The ROS joystick deadman is an additional gate.
        break;
      case GateState::FAULT_LATCHED:
        // Handled before the Stop and normal-state logic above.
        break;
    }
    return result();
  }

private:
  static void observe_button(
    bool active, double now, bool & have_observation, bool & last_active,
    double & changed_at) noexcept
  {
    if (!have_observation || active != last_active) {
      have_observation = true;
      last_active = active;
      changed_at = now;
    }
  }

  void latch_fault(FaultReason reason) noexcept
  {
    if (state_ != GateState::FAULT_LATCHED) {
      fault_ = reason;
    }
    state_ = GateState::FAULT_LATCHED;
    have_start_observation_ = false;
    have_reset_observation_ = false;
    safe_release_observing_ = false;
    fault_reset_release_seen_ = false;
  }

  Result result() const noexcept
  {
    return {state_, fault_, motion_enabled()};
  }

  Config config_;
  GateState state_{GateState::WAITING_FOR_SAFE_RELEASE};
  FaultReason fault_{FaultReason::NONE};
  bool have_time_{false};
  double last_update_time_{0.0};
  bool have_start_observation_{false};
  bool last_start_active_{false};
  double start_changed_at_{0.0};
  bool have_reset_observation_{false};
  bool last_reset_active_{false};
  double reset_changed_at_{0.0};
  bool safe_release_observing_{false};
  double safe_release_started_at_{0.0};
  bool fault_reset_release_seen_{false};
};

}  // namespace studica_vmxpi_ros2::local_enable_gate

#endif  // STUDICA_VMXPI_ROS2__LOCAL_ENABLE_GATE_HPP_
