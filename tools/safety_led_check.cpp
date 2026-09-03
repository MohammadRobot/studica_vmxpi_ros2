// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include "dio.h"

#include <VMXPi.h>

#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

namespace
{
struct Options
{
  int start_led_channel{12};
  int stop_led_channel{13};
  bool runtime_stopped{false};
  bool wheels_lifted{false};
  bool output_jumper_5v{false};
  bool high_current_output{false};
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program
    << " --confirm-runtime-stopped --confirm-wheels-lifted "
    << "--confirm-output-jumper-5v --confirm-high-current-output [options]\n\n"
    << "Options:\n"
    << "  --start-led-channel N   Start LED DIO output (default: 12)\n"
    << "  --stop-led-channel N    Stop LED DIO output (default: 13)\n"
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
    if (argument == "--confirm-output-jumper-5v") {
      options.output_jumper_5v = true;
      continue;
    }
    if (argument == "--confirm-high-current-output") {
      options.high_current_output = true;
      continue;
    }

    int * destination = nullptr;
    if (argument == "--start-led-channel") {
      destination = &options.start_led_channel;
    } else if (argument == "--stop-led-channel") {
      destination = &options.stop_led_channel;
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

class OutputsOffGuard
{
public:
  OutputsOffGuard(studica_driver::DIO & start_led, studica_driver::DIO & stop_led)
  : start_led_(start_led), stop_led_(stop_led) {}

  ~OutputsOffGuard()
  {
    start_led_.Set(false);
    stop_led_.Set(false);
  }

private:
  studica_driver::DIO & start_led_;
  studica_driver::DIO & stop_led_;
};

bool operator_confirms(const char * expected)
{
  std::cout << "\nOBSERVE: " << expected << "\n"
            << "Type yes only after the physical indication matches: " << std::flush;
  std::string answer;
  if (!std::getline(std::cin, answer)) {
    std::cerr << "\nFAIL: operator confirmation input ended.\n";
    return false;
  }
  if (answer != "yes") {
    std::cerr << "FAIL: indication was not confirmed.\n";
    return false;
  }
  std::cout << "PASS: operator confirmed the expected indication.\n";
  return true;
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

  if (
    !options.runtime_stopped || !options.wheels_lifted ||
    !options.output_jumper_5v || !options.high_current_output)
  {
    std::cerr
      << "REFUSED: stop robot bringup, lift and secure all wheels, verify the "
      << "VMX output-voltage jumper is physically set to 5 V, and verify the "
      << "High-Current DIO direction jumper is in OUTPUT mode; then pass all "
      << "four confirmation flags.\n";
    return EXIT_FAILURE;
  }
  if (geteuid() != 0) {
    std::cerr << "REFUSED: VMX HAL access requires root; run this installed "
                 "binary with sudo.\n";
    return EXIT_FAILURE;
  }
  if (
    options.start_led_channel < 0 || options.start_led_channel > 29 ||
    options.stop_led_channel < 0 || options.stop_led_channel > 29 ||
    options.start_led_channel == options.stop_led_channel)
  {
    std::cerr << "REFUSED: LED channels must be distinct DIO indices in [0, 29].\n";
    return EXIT_FAILURE;
  }

  std::cout
    << "Control-panel LED acceptance only. This program never initializes Titan.\n"
    << "Start LED=" << options.start_led_channel
    << " Stop LED=" << options.stop_led_channel << "\n";

  auto vmx = std::make_shared<VMXPi>(true, 50);
  if (!vmx || !vmx->IsOpen()) {
    std::cerr << "FAIL: unable to open VMXPi. Stop every other VMX HAL owner and retry.\n";
    return EXIT_FAILURE;
  }

  studica_driver::DIO start_led(
    static_cast<VMXChannelIndex>(options.start_led_channel),
    studica_driver::PinMode::OUTPUT, vmx);
  studica_driver::DIO stop_led(
    static_cast<VMXChannelIndex>(options.stop_led_channel),
    studica_driver::PinMode::OUTPUT, vmx);
  if (!start_led.IsInitialized() || !stop_led.IsInitialized()) {
    std::cerr << "FAIL: unable to initialize both LED outputs.\n";
    return EXIT_FAILURE;
  }
  OutputsOffGuard outputs_off_guard(start_led, stop_led);

  start_led.Set(false);
  stop_led.Set(false);
  if (!operator_confirms("Start LED OFF; Stop LED OFF")) {
    return EXIT_FAILURE;
  }

  start_led.Set(true);
  stop_led.Set(false);
  if (!operator_confirms("Start LED ON only; Stop LED OFF")) {
    return EXIT_FAILURE;
  }

  start_led.Set(false);
  stop_led.Set(true);
  if (!operator_confirms("Stop LED ON only; Start LED OFF")) {
    return EXIT_FAILURE;
  }

  start_led.Set(false);
  stop_led.Set(false);
  if (!operator_confirms("Start LED OFF; Stop LED OFF")) {
    return EXIT_FAILURE;
  }

  std::cout
    << "\nCONTROL-PANEL LED ACCEPTANCE: PASS\n"
    << "Both outputs are OFF. Leave E-stop pressed.\n";
  return EXIT_SUCCESS;
}
