// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include "dio.h"

#include <VMXPi.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace
{
using Clock = std::chrono::steady_clock;

struct Options
{
  int estop_channel{8};
  int start_channel{9};
  int reset_channel{10};
  int stop_channel{11};
  int stable_ms{500};
  int stage_timeout_s{90};
  bool runtime_stopped{false};
  bool wheels_lifted{false};
};

struct Stage
{
  const char * instruction;
  std::array<bool, 4> levels_high;
};

struct Input
{
  const char * name;
  int channel;
  studica_driver::DIO * dio;
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program
    << " --confirm-runtime-stopped --confirm-wheels-lifted [options]\n"
    << "\n"
    << "Input wiring required by the production profile:\n"
    << "  E-stop and Stop: normally closed to VMX GND (LOW is healthy)\n"
    << "  Start and Reset: normally open to VMX GND (LOW is pressed)\n"
    << "\n"
    << "Options:\n"
    << "  --estop-channel N       E-stop status FlexDIO (default: 8)\n"
    << "  --start-channel N       Start button FlexDIO (default: 9)\n"
    << "  --reset-channel N       Reset button FlexDIO (default: 10)\n"
    << "  --stop-channel N        Stop button FlexDIO (default: 11)\n"
    << "  --stable-ms N           Required stable state time (default: 500)\n"
    << "  --stage-timeout-s N     Timeout per operator action (default: 90)\n"
    << "  --help                  Show this message\n";
}

bool parse_int(const char * text, int & value)
{
  try {
    std::size_t consumed = 0;
    const std::string input(text);
    const int parsed = std::stoi(input, &consumed);
    if (consumed != input.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

int parse_options(int argc, char ** argv, Options & options)
{
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      print_usage(argv[0]);
      return 1;
    }
    if (argument == "--confirm-runtime-stopped") {
      options.runtime_stopped = true;
      continue;
    }
    if (argument == "--confirm-wheels-lifted") {
      options.wheels_lifted = true;
      continue;
    }

    int * destination = nullptr;
    if (argument == "--estop-channel") {
      destination = &options.estop_channel;
    } else if (argument == "--start-channel") {
      destination = &options.start_channel;
    } else if (argument == "--reset-channel") {
      destination = &options.reset_channel;
    } else if (argument == "--stop-channel") {
      destination = &options.stop_channel;
    } else if (argument == "--stable-ms") {
      destination = &options.stable_ms;
    } else if (argument == "--stage-timeout-s") {
      destination = &options.stage_timeout_s;
    } else {
      std::cerr << "Unknown argument: " << argument << "\n";
      return -1;
    }

    if (++index >= argc || !parse_int(argv[index], *destination)) {
      std::cerr << "Expected an integer after " << argument << "\n";
      return -1;
    }
  }
  return 0;
}

const char * level_name(bool level_high)
{
  return level_high ? "HIGH/open" : "LOW/grounded";
}

void print_levels(
  const std::array<Input, 4> & inputs,
  const std::array<bool, 4> & levels)
{
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    if (index != 0U) {
      std::cout << ", ";
    }
    std::cout << inputs[index].name << "=" << level_name(levels[index]);
  }
  std::cout << "\n" << std::flush;
}

bool read_inputs(
  const std::array<Input, 4> & inputs,
  std::array<bool, 4> & levels)
{
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    if (!inputs[index].dio->TryGet(levels[index])) {
      std::cerr << "FAIL: VMX DIO " << inputs[index].channel << " ("
                << inputs[index].name << ") read failed; inputs are not trustworthy.\n";
      return false;
    }
  }
  return true;
}

bool wait_for_stage(
  const Stage & stage,
  const std::array<Input, 4> & inputs,
  const Options & options)
{
  std::cout << "\nACTION: " << stage.instruction << "\nExpected: ";
  print_levels(inputs, stage.levels_high);

  const auto deadline = Clock::now() + std::chrono::seconds(options.stage_timeout_s);
  auto matching_since = Clock::time_point{};
  std::array<bool, 4> previous_levels{};
  bool have_previous = false;

  while (Clock::now() < deadline) {
    std::array<bool, 4> levels{};
    if (!read_inputs(inputs, levels)) {
      return false;
    }

    if (!have_previous || levels != previous_levels) {
      std::cout << "Observed: ";
      print_levels(inputs, levels);
      previous_levels = levels;
      have_previous = true;
    }

    if (levels == stage.levels_high) {
      if (matching_since == Clock::time_point{}) {
        matching_since = Clock::now();
      }
      if (
        Clock::now() - matching_since >=
        std::chrono::milliseconds(options.stable_ms))
      {
        std::cout << "PASS: expected state remained stable for "
                  << options.stable_ms << " ms.\n";
        return true;
      }
    } else {
      matching_since = Clock::time_point{};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cerr << "FAIL: expected state was not stable within "
            << options.stage_timeout_s << " seconds. Check contact type, "
            << "ground, channel assignment, and pull-up behavior.\n";
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  const int parse_result = parse_options(argc, argv, options);
  if (parse_result > 0) {
    return EXIT_SUCCESS;
  }
  if (parse_result < 0) {
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  if (!options.runtime_stopped || !options.wheels_lifted) {
    std::cerr
      << "REFUSED: stop robot bringup, lift and secure all wheels, verify no "
      << "motion, then pass both confirmation flags.\n";
    return EXIT_FAILURE;
  }
  if (geteuid() != 0) {
    std::cerr << "REFUSED: VMX HAL access requires root; run this installed "
                 "binary with sudo.\n";
    return EXIT_FAILURE;
  }

  const std::array<int, 4> channels{
    options.estop_channel, options.start_channel,
    options.reset_channel, options.stop_channel};
  for (const int channel : channels) {
    if (channel < 0 || channel > 29) {
      std::cerr << "REFUSED: every input channel must be in [0, 29].\n";
      return EXIT_FAILURE;
    }
  }
  auto sorted_channels = channels;
  std::sort(sorted_channels.begin(), sorted_channels.end());
  if (
    std::adjacent_find(sorted_channels.begin(), sorted_channels.end()) !=
    sorted_channels.end())
  {
    std::cerr << "REFUSED: all four input channels must be different.\n";
    return EXIT_FAILURE;
  }
  if (options.stable_ms <= 0 || options.stage_timeout_s <= 0) {
    std::cerr << "REFUSED: timing values must be positive.\n";
    return EXIT_FAILURE;
  }

  std::cout
    << "Control-panel input acceptance only. This program never initializes "
    << "Titan and never writes an output.\n"
    << "E-stop=" << options.estop_channel
    << " Start=" << options.start_channel
    << " Reset=" << options.reset_channel
    << " Stop=" << options.stop_channel << "\n";

  auto vmx = std::make_shared<VMXPi>(true, 50);
  if (!vmx || !vmx->IsOpen()) {
    std::cerr << "FAIL: unable to open VMXPi. Stop every other VMX HAL owner "
                 "and retry.\n";
    return EXIT_FAILURE;
  }

  studica_driver::DIO estop_input(
    static_cast<VMXChannelIndex>(options.estop_channel),
    studica_driver::PinMode::INPUT, vmx);
  studica_driver::DIO start_input(
    static_cast<VMXChannelIndex>(options.start_channel),
    studica_driver::PinMode::INPUT, vmx);
  studica_driver::DIO reset_input(
    static_cast<VMXChannelIndex>(options.reset_channel),
    studica_driver::PinMode::INPUT, vmx);
  studica_driver::DIO stop_input(
    static_cast<VMXChannelIndex>(options.stop_channel),
    studica_driver::PinMode::INPUT, vmx);
  if (
    !estop_input.IsInitialized() || !start_input.IsInitialized() ||
    !reset_input.IsInitialized() || !stop_input.IsInitialized())
  {
    std::cerr << "FAIL: unable to initialize all four FlexDIO inputs.\n";
    return EXIT_FAILURE;
  }

  const std::array<Input, 4> inputs{{
    {"E-stop", options.estop_channel, &estop_input},
    {"Start", options.start_channel, &start_input},
    {"Reset", options.reset_channel, &reset_input},
    {"Stop", options.stop_channel, &stop_input},
  }};

  // HIGH/open is fail-safe for the NC E-stop and Stop loops. Start and Reset
  // are NO controls and therefore read HIGH/open while released.
  const std::vector<Stage> stages{
    {"Press E-stop; release Start, Reset, and Stop.", {true, true, true, false}},
    {"Mechanically reset/release E-stop; leave all buttons released.",
      {false, true, true, false}},
    {"Press and hold Start only.", {false, false, true, false}},
    {"Release Start.", {false, true, true, false}},
    {"Press and hold Reset only.", {false, true, false, false}},
    {"Release Reset.", {false, true, true, false}},
    {"Press and hold Stop only.", {false, true, true, true}},
    {"Release Stop.", {false, true, true, false}},
    {"Disconnect the E-stop status wire from DIO signal.", {true, true, true, false}},
    {"Reconnect the E-stop status wire with E-stop released.",
      {false, true, true, false}},
    {"Disconnect the Stop status wire from DIO signal.", {false, true, true, true}},
    {"Reconnect the Stop status wire with Stop released.",
      {false, true, true, false}},
    {"Press E-stop and leave every momentary button released.",
      {true, true, true, false}},
  };

  for (const auto & stage : stages) {
    if (!wait_for_stage(stage, inputs, options)) {
      std::cerr << "\nCONTROL-PANEL INPUT ACCEPTANCE: FAIL\n";
      return EXIT_FAILURE;
    }
  }

  std::cout
    << "\nCONTROL-PANEL INPUT ACCEPTANCE: PASS\n"
    << "Leave E-stop pressed. Archive this output with wiring photos and the "
    << "separate LED checkout record.\n";
  return EXIT_SUCCESS;
}
