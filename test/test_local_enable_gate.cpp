// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "studica_vmxpi_ros2/local_enable_gate.hpp"

namespace gate = studica_vmxpi_ros2::local_enable_gate;

namespace
{
const gate::Inputs kReleased{true, true, true, true, false, false, true};
const gate::Inputs kStartPressed{true, true, true, true, true, false, true};
const gate::Inputs kResetPressed{true, true, true, true, false, true, true};
const gate::Inputs kStopPressed{true, true, true, true, false, false, false};
}  // namespace

TEST(LocalEnableGate, MomentaryStartLatchesOnlyAfterSafeReleaseAndDebounce)
{
  gate::LocalEnableGate safety_gate;

  EXPECT_EQ(
    safety_gate.update(kStartPressed, 0.0).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(
    safety_gate.update(kStartPressed, 10.0).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);

  EXPECT_EQ(
    safety_gate.update(kReleased, 10.1).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(safety_gate.update(kReleased, 10.6).state, gate::GateState::READY);

  EXPECT_EQ(safety_gate.update(kStartPressed, 10.7).state, gate::GateState::READY);
  const auto enabled = safety_gate.update(kStartPressed, 10.8);
  EXPECT_EQ(enabled.state, gate::GateState::ENABLED);
  EXPECT_TRUE(enabled.motion_enabled);

  const auto released_after_start = safety_gate.update(kReleased, 10.801);
  EXPECT_EQ(released_after_start.state, gate::GateState::ENABLED);
  EXPECT_TRUE(released_after_start.motion_enabled);
}

TEST(LocalEnableGate, NormallyClosedStopDropsImmediatelyAndRequiresNewStart)
{
  gate::LocalEnableGate safety_gate({0.1, 0.2});
  ASSERT_EQ(safety_gate.update(kReleased, 0.0).state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(kReleased, 0.2).state, gate::GateState::READY);
  ASSERT_EQ(safety_gate.update(kStartPressed, 0.3).state, gate::GateState::READY);
  ASSERT_TRUE(safety_gate.update(kStartPressed, 0.41).motion_enabled);
  ASSERT_TRUE(safety_gate.update(kReleased, 0.42).motion_enabled);

  const auto stopped = safety_gate.update(kStopPressed, 0.421);
  EXPECT_EQ(stopped.state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_FALSE(stopped.motion_enabled);

  EXPECT_EQ(
    safety_gate.update(kReleased, 0.5).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(safety_gate.update(kReleased, 0.71).state, gate::GateState::READY);
  EXPECT_FALSE(safety_gate.motion_enabled());

  EXPECT_EQ(safety_gate.update(kStartPressed, 0.8).state, gate::GateState::READY);
  EXPECT_TRUE(safety_gate.update(kStartPressed, 0.91).motion_enabled);
}

TEST(LocalEnableGate, ClearedEstopRequiresReleasedResetThenNewStart)
{
  gate::LocalEnableGate safety_gate({0.1, 0.2});
  ASSERT_EQ(safety_gate.update(kReleased, 0.0).state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(kReleased, 0.2).state, gate::GateState::READY);
  ASSERT_EQ(safety_gate.update(kStartPressed, 0.3).state, gate::GateState::READY);
  ASSERT_TRUE(safety_gate.update(kStartPressed, 0.41).motion_enabled);

  const gate::Inputs estop_pressed{true, false, true, true, false, false, true};
  const auto faulted = safety_gate.update(estop_pressed, 0.5);
  EXPECT_EQ(faulted.state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(faulted.fault, gate::FaultReason::ESTOP_NOT_OK);
  EXPECT_FALSE(faulted.motion_enabled);

  // Reset held while the fault clears is not a fresh acknowledgement.
  EXPECT_EQ(
    safety_gate.update(kResetPressed, 1.0).state,
    gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(safety_gate.update(kReleased, 1.1).state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(safety_gate.update(kResetPressed, 1.2).state, gate::GateState::FAULT_LATCHED);
  const auto acknowledged = safety_gate.update(kResetPressed, 1.31);
  EXPECT_EQ(acknowledged.state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(acknowledged.fault, gate::FaultReason::NONE);
  EXPECT_FALSE(acknowledged.motion_enabled);

  EXPECT_EQ(
    safety_gate.update(kReleased, 1.4).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(safety_gate.update(kReleased, 1.61).state, gate::GateState::READY);
  EXPECT_EQ(safety_gate.update(kStartPressed, 1.7).state, gate::GateState::READY);
  EXPECT_TRUE(safety_gate.update(kStartPressed, 1.81).motion_enabled);
}

TEST(LocalEnableGate, ResetNeverEnablesMotion)
{
  gate::LocalEnableGate safety_gate({0.0, 0.1});
  ASSERT_EQ(safety_gate.update(kReleased, 0.0).state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(kReleased, 0.1).state, gate::GateState::READY);

  EXPECT_EQ(safety_gate.update(kResetPressed, 0.2).state, gate::GateState::READY);
  EXPECT_FALSE(safety_gate.motion_enabled());
  EXPECT_EQ(safety_gate.update(kResetPressed, 10.0).state, gate::GateState::READY);
  EXPECT_FALSE(safety_gate.motion_enabled());
}

TEST(LocalEnableGate, InvalidInputAndDriveHealthAreFailClosed)
{
  gate::LocalEnableGate invalid_input_gate({0.0, 0.0});
  auto invalid = invalid_input_gate.update(
    {false, true, true, true, false, false, true}, 0.0);
  EXPECT_EQ(invalid.state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(invalid.fault, gate::FaultReason::INPUT_INVALID);

  gate::LocalEnableGate unhealthy_drive_gate({0.0, 0.0});
  auto unhealthy = unhealthy_drive_gate.update(
    {true, true, false, false, false, false, true}, 0.0);
  EXPECT_EQ(unhealthy.state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(unhealthy.fault, gate::FaultReason::DRIVE_UNHEALTHY);
}

TEST(LocalEnableGate, DriveFaultResetRequiresVerifiedClearablePath)
{
  gate::LocalEnableGate safety_gate({0.1, 0.2});
  ASSERT_EQ(safety_gate.update(kReleased, 0.0).state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(kReleased, 0.2).state, gate::GateState::READY);
  ASSERT_EQ(safety_gate.update(kStartPressed, 0.3).state, gate::GateState::READY);
  ASSERT_TRUE(safety_gate.update(kStartPressed, 0.41).motion_enabled);

  const gate::Inputs drive_fault{true, true, false, false, false, false, true};
  const auto faulted = safety_gate.update(drive_fault, 0.5);
  ASSERT_EQ(faulted.state, gate::GateState::FAULT_LATCHED);
  ASSERT_EQ(faulted.fault, gate::FaultReason::DRIVE_UNHEALTHY);

  const gate::Inputs premature_reset{true, true, false, false, false, true, true};
  EXPECT_EQ(
    safety_gate.update(premature_reset, 1.0).state,
    gate::GateState::FAULT_LATCHED);

  const gate::Inputs clearable_released{true, true, false, true, false, false, true};
  const gate::Inputs clearable_reset{true, true, false, true, false, true, true};
  EXPECT_EQ(
    safety_gate.update(clearable_released, 1.1).state,
    gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(
    safety_gate.update(clearable_reset, 1.2).state,
    gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(
    safety_gate.update(clearable_reset, 1.31).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
}

TEST(LocalEnableGate, RejectsInvalidClockAndConfiguration)
{
  gate::LocalEnableGate safety_gate({0.0, 0.0});
  ASSERT_EQ(
    safety_gate.update(kReleased, 1.0).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(kReleased, 1.0).state, gate::GateState::READY);
  EXPECT_EQ(
    safety_gate.update(kStartPressed, 0.9).fault,
    gate::FaultReason::TIME_INVALID);

  EXPECT_THROW(
    gate::LocalEnableGate(
      {std::numeric_limits<double>::quiet_NaN(), 0.1}),
    std::invalid_argument);
  EXPECT_THROW(gate::LocalEnableGate({-0.1, 0.1}), std::invalid_argument);
}
