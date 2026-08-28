// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "studica_vmxpi_ros2/safety_supervisor.hpp"

namespace safety = studica_vmxpi_ros2::safety_supervisor;

namespace
{
const std::vector<std::string> kHardwareSafetyNames = {
  "input_valid", "estop_ok", "enable_active", "drive_healthy",
  "motion_enabled", "gate_state", "fault_reason"};
}  // namespace

TEST(SafetyStateMachine, BootsDisarmedAndRequiresAllArmConditions)
{
  safety::RobotStateMachine machine;
  EXPECT_EQ(machine.state(), safety::RobotState::BOOTING);
  EXPECT_FALSE(machine.motion_allowed());

  EXPECT_TRUE(machine.complete_boot(true).accepted);
  EXPECT_EQ(machine.state(), safety::RobotState::READY_DISARMED);
  EXPECT_FALSE(machine.arm({false, true, true, true}).accepted);
  EXPECT_FALSE(machine.arm({true, false, true, true}).accepted);
  EXPECT_FALSE(machine.arm({true, true, false, true}).accepted);
  EXPECT_FALSE(machine.arm({true, true, true, false}).accepted);
  EXPECT_FALSE(machine.motion_allowed());

  EXPECT_TRUE(machine.arm({true, true, true, true}).accepted);
  EXPECT_EQ(machine.state(), safety::RobotState::ARMED);
  EXPECT_TRUE(machine.motion_allowed());
  EXPECT_TRUE(machine.disarm().accepted);
  EXPECT_EQ(machine.state(), safety::RobotState::READY_DISARMED);
}

TEST(SafetyStateMachine, FaultsLatchAndUpdatesRequireDisarmedState)
{
  safety::RobotStateMachine machine;
  ASSERT_TRUE(machine.complete_boot(true).accepted);
  ASSERT_TRUE(machine.arm({true, true, true, true}).accepted);
  EXPECT_FALSE(machine.begin_update().accepted);

  machine.latch_fault();
  EXPECT_EQ(machine.state(), safety::RobotState::FAULT);
  EXPECT_FALSE(machine.acknowledge_fault(false, true).accepted);
  EXPECT_FALSE(machine.acknowledge_fault(true, false).accepted);
  EXPECT_TRUE(machine.acknowledge_fault(true, true).accepted);
  EXPECT_EQ(machine.state(), safety::RobotState::READY_DISARMED);

  EXPECT_TRUE(machine.begin_update().accepted);
  EXPECT_EQ(machine.state(), safety::RobotState::UPDATING);
  EXPECT_TRUE(machine.finish_update(false).accepted);
  EXPECT_EQ(machine.state(), safety::RobotState::FAULT);
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyCommand, RejectsMotionUnlessArmedFreshFinitePlanarAndSingleSource)
{
  const safety::Limits limits{0.5, 0.4, 1.0};
  safety::Command command;
  command.linear_x = 0.2;
  command.angular_z = -0.3;

  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::READY_DISARMED, command, true, 1.0, 1.1, 1U, 0.25, limits).decision,
    safety::CommandDecision::DISARMED);
  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::ARMED, command, false, 1.0, 1.1, 1U, 0.25, limits).decision,
    safety::CommandDecision::NO_COMMAND);
  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::ARMED, command, true, 1.0, 1.1, 0U, 0.25, limits).decision,
    safety::CommandDecision::SOURCE_MISSING);
  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::ARMED, command, true, 1.0, 1.1, 2U, 0.25, limits).decision,
    safety::CommandDecision::SOURCE_CONFLICT);
  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::ARMED, command, true, 1.0, 1.3, 1U, 0.25, limits).decision,
    safety::CommandDecision::STALE);

  command.linear_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::ARMED, command, true, 1.0, 1.1, 1U, 0.25, limits).decision,
    safety::CommandDecision::NONFINITE);
  command.linear_x = 0.2;
  command.angular_x = 0.1;
  EXPECT_EQ(
    safety::evaluate(
      safety::RobotState::ARMED, command, true, 1.0, 1.1, 1U, 0.25, limits).decision,
    safety::CommandDecision::UNSUPPORTED_3D);
}

TEST(SafetyCommand, ClampsSpeedAndLimitsAcceleration)
{
  const safety::Limits limits{0.5, 0.4, 1.0};
  safety::Command command;
  command.linear_x = 4.0;
  command.linear_y = -3.0;
  command.angular_z = 2.0;
  const auto evaluated = safety::evaluate(
    safety::RobotState::ARMED, command, true, 1.0, 1.1, 1U, 0.25, limits);
  ASSERT_EQ(evaluated.decision, safety::CommandDecision::ACCEPTED);
  EXPECT_DOUBLE_EQ(evaluated.output.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(evaluated.output.linear_y, -0.4);
  EXPECT_DOUBLE_EQ(evaluated.output.angular_z, 1.0);

  const safety::Command limited = safety::rate_limit_planar(
    evaluated.output, {}, 0.1, 1.0, 2.0);
  EXPECT_NEAR(limited.linear_x, 0.1, 1e-9);
  EXPECT_NEAR(limited.linear_y, -0.1, 1e-9);
  EXPECT_NEAR(limited.angular_z, 0.2, 1e-9);
}

TEST(JoystickDeadmanGate, RequiresReleaseAfterArmAndInvalidState)
{
  safety::JoystickDeadmanGate deadman;
  EXPECT_EQ(
    deadman.update(true, true).decision,
    safety::JoystickDeadmanDecision::RELEASE_REQUIRED);
  EXPECT_FALSE(deadman.active());

  EXPECT_EQ(
    deadman.update(true, false).decision,
    safety::JoystickDeadmanDecision::RELEASED);
  EXPECT_TRUE(deadman.update(true, true).active);
  EXPECT_TRUE(deadman.active());

  deadman.require_release();
  EXPECT_FALSE(deadman.update(true, true).active);
  EXPECT_FALSE(deadman.update(false, false).active);
  EXPECT_EQ(
    deadman.update(true, true).decision,
    safety::JoystickDeadmanDecision::RELEASE_REQUIRED);
  EXPECT_FALSE(deadman.update(true, false).active);
  EXPECT_TRUE(deadman.update(true, true).active);
}

TEST(HardwareSafetyDecode, AcceptsConsistentReadyEnabledAndFaultStates)
{
  auto ready = safety::decode_hardware_safety(
    kHardwareSafetyNames, {1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0});
  EXPECT_TRUE(ready.encoding_valid);
  EXPECT_EQ(ready.gate_state, safety::HardwareGateState::READY);

  auto enabled = safety::decode_hardware_safety(
    kHardwareSafetyNames, {1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 0.0});
  EXPECT_TRUE(enabled.encoding_valid);
  EXPECT_TRUE(enabled.motion_enabled);

  auto fault = safety::decode_hardware_safety(
    kHardwareSafetyNames, {1.0, 0.0, 1.0, 1.0, 0.0, 3.0, 2.0});
  EXPECT_TRUE(fault.encoding_valid);
  EXPECT_EQ(fault.fault_reason, safety::HardwareFaultReason::ESTOP_NOT_OK);
}

TEST(HardwareSafetyDecode, RejectsMissingDuplicateNonfiniteAndInconsistentState)
{
  EXPECT_FALSE(safety::decode_hardware_safety(
      {"input_valid"}, {1.0}).encoding_valid);

  auto duplicate_names = kHardwareSafetyNames;
  duplicate_names.push_back("estop_ok");
  EXPECT_FALSE(safety::decode_hardware_safety(
      duplicate_names, {1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0}).encoding_valid);

  auto extra_names = kHardwareSafetyNames;
  extra_names.push_back("unexpected");
  EXPECT_FALSE(safety::decode_hardware_safety(
      extra_names, {1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0}).encoding_valid);

  EXPECT_FALSE(safety::decode_hardware_safety(
      kHardwareSafetyNames,
      {1.0, 1.0, 0.0, 1.0, 0.0, 1.0, std::numeric_limits<double>::infinity()})
      .encoding_valid);
  EXPECT_FALSE(safety::decode_hardware_safety(
      kHardwareSafetyNames, {1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0}).encoding_valid);
  EXPECT_FALSE(safety::decode_hardware_safety(
      kHardwareSafetyNames, {1.0, 1.0, 1.0, 0.0, 1.0, 2.0, 0.0}).encoding_valid);
  EXPECT_FALSE(safety::decode_hardware_safety(
      kHardwareSafetyNames, {1.0, 1.0, 0.0, 1.0, 0.0, 3.0, 0.0}).encoding_valid);
}
