// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "studica_vmxpi_ros2/local_enable_gate.hpp"

namespace gate = studica_vmxpi_ros2::local_enable_gate;

TEST(LocalEnableGate, RequiresOffAfterBootThenADebouncedOnEdge)
{
  gate::LocalEnableGate safety_gate;
  const gate::Inputs held_on{true, true, true, true};
  EXPECT_EQ(
    safety_gate.update(held_on, 0.0).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(
    safety_gate.update(held_on, 10.0).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);

  const gate::Inputs released{true, true, true, false};
  EXPECT_EQ(
    safety_gate.update(released, 10.1).state,
    gate::GateState::WAITING_FOR_SAFE_RELEASE);
  EXPECT_EQ(safety_gate.update(released, 10.6).state, gate::GateState::READY);

  EXPECT_EQ(safety_gate.update(held_on, 10.7).state, gate::GateState::READY);
  const auto enabled = safety_gate.update(held_on, 10.8);
  EXPECT_EQ(enabled.state, gate::GateState::ENABLED);
  EXPECT_TRUE(enabled.motion_enabled);
}

TEST(LocalEnableGate, ReleaseStopsImmediatelyWithoutDebounce)
{
  gate::LocalEnableGate safety_gate({0.1, 0.1});
  const gate::Inputs released{true, true, true, false};
  const gate::Inputs active{true, true, true, true};
  ASSERT_EQ(safety_gate.update(released, 0.0).state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(released, 0.1).state, gate::GateState::READY);
  ASSERT_EQ(safety_gate.update(active, 0.2).state, gate::GateState::READY);
  ASSERT_TRUE(safety_gate.update(active, 0.31).motion_enabled);

  const auto stopped = safety_gate.update(released, 0.311);
  EXPECT_EQ(stopped.state, gate::GateState::READY);
  EXPECT_FALSE(stopped.motion_enabled);
}

TEST(LocalEnableGate, EstopLossLatchesUntilSafePhysicalAcknowledgement)
{
  gate::LocalEnableGate safety_gate({0.0, 0.2});
  const gate::Inputs released{true, true, true, false};
  const gate::Inputs active{true, true, true, true};
  ASSERT_EQ(safety_gate.update(released, 0.0).state, gate::GateState::WAITING_FOR_SAFE_RELEASE);
  ASSERT_EQ(safety_gate.update(released, 0.2).state, gate::GateState::READY);
  ASSERT_TRUE(safety_gate.update(active, 0.3).motion_enabled);

  const auto faulted = safety_gate.update({true, false, true, true}, 0.4);
  EXPECT_EQ(faulted.state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(faulted.fault, gate::FaultReason::ESTOP_NOT_OK);
  EXPECT_FALSE(faulted.motion_enabled);

  // Clearing the E-stop while enable remains ON cannot re-enable or acknowledge.
  EXPECT_EQ(safety_gate.update(active, 1.0).state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(safety_gate.update(released, 1.1).state, gate::GateState::FAULT_LATCHED);
  const auto acknowledged = safety_gate.update(released, 1.3);
  EXPECT_EQ(acknowledged.state, gate::GateState::FAULT_LATCHED);
  const auto safe_acknowledged = safety_gate.update(released, 1.51);
  EXPECT_EQ(safe_acknowledged.state, gate::GateState::READY);
  EXPECT_EQ(safe_acknowledged.fault, gate::FaultReason::NONE);
  EXPECT_FALSE(safe_acknowledged.motion_enabled);
}

TEST(LocalEnableGate, FaultedTimeDoesNotCountAsSafeRelease)
{
  gate::LocalEnableGate safety_gate({0.0, 0.5});
  const gate::Inputs invalid_off{false, true, true, false};
  const gate::Inputs valid_off{true, true, true, false};

  ASSERT_EQ(
    safety_gate.update(invalid_off, 0.0).state,
    gate::GateState::FAULT_LATCHED);
  ASSERT_EQ(
    safety_gate.update(invalid_off, 10.0).state,
    gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(
    safety_gate.update(valid_off, 10.1).state,
    gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(safety_gate.update(valid_off, 10.61).state, gate::GateState::READY);
}

TEST(LocalEnableGate, InvalidInputAndDriveHealthAreFailClosed)
{
  gate::LocalEnableGate invalid_input_gate({0.0, 0.0});
  auto invalid = invalid_input_gate.update({false, true, true, false}, 0.0);
  EXPECT_EQ(invalid.state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(invalid.fault, gate::FaultReason::INPUT_INVALID);

  gate::LocalEnableGate unhealthy_drive_gate({0.0, 0.0});
  auto unhealthy = unhealthy_drive_gate.update({true, true, false, false}, 0.0);
  EXPECT_EQ(unhealthy.state, gate::GateState::FAULT_LATCHED);
  EXPECT_EQ(unhealthy.fault, gate::FaultReason::DRIVE_UNHEALTHY);
}

TEST(LocalEnableGate, RejectsInvalidClockAndConfiguration)
{
  gate::LocalEnableGate safety_gate({0.0, 0.0});
  ASSERT_EQ(
    safety_gate.update({true, true, true, false}, 1.0).state,
    gate::GateState::READY);
  EXPECT_EQ(
    safety_gate.update({true, true, true, true}, 0.9).fault,
    gate::FaultReason::TIME_INVALID);

  EXPECT_THROW(
    gate::LocalEnableGate(
      {std::numeric_limits<double>::quiet_NaN(), 0.1}),
    std::invalid_argument);
  EXPECT_THROW(gate::LocalEnableGate({-0.1, 0.1}), std::invalid_argument);
}
