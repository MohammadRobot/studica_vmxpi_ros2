// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include "dio.h"

#include <VMXPi.h>

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;

struct Options {
  int estop_channel{8};
  int enable_channel{9};
  int stable_ms{500};
  int stage_timeout_s{90};
  bool motor_power_disconnected{false};
};

struct Stage {
  const char *instruction;
  bool estop_level_high;
  bool enable_level_high;
};

void print_usage(const char *program) {
  std::cout
      << "Usage: " << program
      << " --confirm-motor-power-disconnected [options]\n"
      << "\n"
      << "Options:\n"
      << "  --estop-channel N       ESTOP_OK FlexDIO channel (default: 8)\n"
      << "  --enable-channel N      LOCAL_ENABLE FlexDIO channel (default: 9)\n"
      << "  --stable-ms N           Required stable state time (default: 500)\n"
      << "  --stage-timeout-s N     Timeout for each operator action (default: "
         "90)\n"
      << "  --help                  Show this message\n";
}

bool parse_int(const char *text, int &value) {
  try {
    std::size_t consumed = 0;
    const int parsed = std::stoi(text, &consumed);
    if (text[consumed] != '\0') {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

int parse_options(int argc, char **argv, Options &options) {
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      print_usage(argv[0]);
      return 1;
    }
    if (argument == "--confirm-motor-power-disconnected") {
      options.motor_power_disconnected = true;
      continue;
    }

    int *destination = nullptr;
    if (argument == "--estop-channel") {
      destination = &options.estop_channel;
    } else if (argument == "--enable-channel") {
      destination = &options.enable_channel;
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

const char *level_name(bool level_high) {
  return level_high ? "HIGH/open" : "LOW/grounded";
}

bool wait_for_stage(const Stage &stage, studica_driver::DIO &estop_input,
                    studica_driver::DIO &enable_input, const Options &options) {
  std::cout << "\nACTION: " << stage.instruction << "\n"
            << "Expected: E-stop=" << level_name(stage.estop_level_high)
            << ", enable=" << level_name(stage.enable_level_high) << "\n"
            << std::flush;

  const auto deadline =
      Clock::now() + std::chrono::seconds(options.stage_timeout_s);
  auto matching_since = Clock::time_point{};
  bool previous_estop_high = false;
  bool previous_enable_high = false;
  bool have_previous = false;

  while (Clock::now() < deadline) {
    bool estop_high = true;
    bool enable_high = true;
    if (!estop_input.TryGet(estop_high) || !enable_input.TryGet(enable_high)) {
      std::cerr << "FAIL: VMX DIO read failed; inputs are not trustworthy.\n";
      return false;
    }

    if (!have_previous || estop_high != previous_estop_high ||
        enable_high != previous_enable_high) {
      std::cout << "Observed: E-stop=" << level_name(estop_high)
                << ", enable=" << level_name(enable_high) << "\n"
                << std::flush;
      previous_estop_high = estop_high;
      previous_enable_high = enable_high;
      have_previous = true;
    }

    const bool matches = estop_high == stage.estop_level_high &&
                         enable_high == stage.enable_level_high;
    if (matches) {
      if (matching_since == Clock::time_point{}) {
        matching_since = Clock::now();
      }
      if (Clock::now() - matching_since >=
          std::chrono::milliseconds(options.stable_ms)) {
        std::cout << "PASS: expected state remained stable for "
                  << options.stable_ms << " ms.\n";
        return true;
      }
    } else {
      matching_since = Clock::time_point{};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cerr
      << "FAIL: operator action did not produce the expected state within "
      << options.stage_timeout_s << " seconds.\n";
  return false;
}
} // namespace

int main(int argc, char **argv) {
  Options options;
  const int parse_result = parse_options(argc, argv, options);
  if (parse_result > 0) {
    return EXIT_SUCCESS;
  }
  if (parse_result < 0) {
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  if (!options.motor_power_disconnected) {
    std::cerr
        << "REFUSED: disconnect Titan motor power, lift the wheels, then pass "
        << "--confirm-motor-power-disconnected.\n";
    return EXIT_FAILURE;
  }
  if (geteuid() != 0) {
    std::cerr << "REFUSED: VMX HAL access requires root; run this installed "
                 "binary with sudo.\n";
    return EXIT_FAILURE;
  }
  if (options.estop_channel < 0 || options.estop_channel > 29 ||
      options.enable_channel < 0 || options.enable_channel > 29 ||
      options.estop_channel == options.enable_channel) {
    std::cerr
        << "REFUSED: channels must be different VMX DIO indices in [0, 29].\n";
    return EXIT_FAILURE;
  }
  if (options.stable_ms <= 0 || options.stage_timeout_s <= 0) {
    std::cerr << "REFUSED: timing values must be positive.\n";
    return EXIT_FAILURE;
  }

  std::cout
      << "Safety input acceptance only; this program never initializes Titan.\n"
      << "E-stop channel: " << options.estop_channel
      << ", local-enable channel: " << options.enable_channel << "\n";

  auto vmx = std::make_shared<VMXPi>(true, 50);
  if (!vmx || !vmx->IsOpen()) {
    std::cerr << "FAIL: unable to open VMXPi. Stop any other VMX HAL owner and "
                 "retry.\n";
    return EXIT_FAILURE;
  }

  studica_driver::DIO estop_input(
      static_cast<VMXChannelIndex>(options.estop_channel),
      studica_driver::PinMode::INPUT, vmx);
  studica_driver::DIO enable_input(
      static_cast<VMXChannelIndex>(options.enable_channel),
      studica_driver::PinMode::INPUT, vmx);
  if (!estop_input.IsInitialized() || !enable_input.IsInitialized()) {
    std::cerr << "FAIL: unable to initialize both FlexDIO inputs.\n";
    return EXIT_FAILURE;
  }

  const std::vector<Stage> stages{
      {"Release the E-stop and set local enable OFF.", false, true},
      {"Press the E-stop; leave local enable OFF.", true, true},
      {"Release/reset the E-stop; leave local enable OFF.", false, true},
      {"Set local enable ON while the E-stop remains released.", false, false},
      {"Set local enable OFF.", false, true},
      {"Disconnect the E-stop status wire from its FlexDIO signal.", true,
       true},
      {"Reconnect the E-stop status wire with the E-stop released.", false,
       true},
  };

  for (const auto &stage : stages) {
    if (!wait_for_stage(stage, estop_input, enable_input, options)) {
      std::cerr << "\nSAFETY INPUT ACCEPTANCE: FAIL\n";
      return EXIT_FAILURE;
    }
  }

  std::cout << "\nSAFETY INPUT ACCEPTANCE: PASS\n"
            << "Keep motor power disconnected. Archive this complete terminal "
               "output with wiring photos.\n";
  return EXIT_SUCCESS;
}
