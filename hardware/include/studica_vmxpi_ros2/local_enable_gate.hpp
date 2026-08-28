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
  bool enable_active{false};
};

struct Config
{
  double enable_debounce_sec{0.10};
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
    if (!std::isfinite(config_.enable_debounce_sec) ||
      !std::isfinite(config_.safe_release_sec) ||
      config_.enable_debounce_sec < 0.0 || config_.safe_release_sec < 0.0)
    {
      throw std::invalid_argument("Local-enable debounce durations must be finite and nonnegative");
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

    // Only a fully valid, safe sample may contribute to either debounce
    // interval. Time spent with a failed input or drive must not count as a
    // physical acknowledgement.
    observe_enable(inputs.enable_active, now);

    const double stable_for = now - enable_changed_at_;
    switch (state_) {
      case GateState::WAITING_FOR_SAFE_RELEASE:
        if (!inputs.enable_active && stable_for >= config_.safe_release_sec) {
          state_ = GateState::READY;
          fault_ = FaultReason::NONE;
        }
        break;
      case GateState::READY:
        if (inputs.enable_active && stable_for >= config_.enable_debounce_sec) {
          state_ = GateState::ENABLED;
        }
        break;
      case GateState::ENABLED:
        if (!inputs.enable_active) {
          // Stopping is deliberately not debounced.
          state_ = GateState::READY;
        }
        break;
      case GateState::FAULT_LATCHED:
        if (!inputs.enable_active && stable_for >= config_.safe_release_sec) {
          // Holding the physical switch OFF after clearing the cause is the
          // local acknowledgement. A new ON edge is still required to enable.
          state_ = GateState::READY;
          fault_ = FaultReason::NONE;
        }
        break;
    }
    return result();
  }

private:
  void observe_enable(bool active, double now) noexcept
  {
    if (!have_enable_observation_ || active != last_enable_active_) {
      have_enable_observation_ = true;
      last_enable_active_ = active;
      enable_changed_at_ = now;
    }
  }

  void latch_fault(FaultReason reason) noexcept
  {
    if (state_ != GateState::FAULT_LATCHED) {
      fault_ = reason;
    }
    state_ = GateState::FAULT_LATCHED;
    have_enable_observation_ = false;
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
  bool have_enable_observation_{false};
  bool last_enable_active_{false};
  double enable_changed_at_{0.0};
};

}  // namespace studica_vmxpi_ros2::local_enable_gate

#endif  // STUDICA_VMXPI_ROS2__LOCAL_ENABLE_GATE_HPP_
