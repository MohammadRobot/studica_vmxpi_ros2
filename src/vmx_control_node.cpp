// Copyright (c) 2026 studica_vmxpi_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include <errno.h>
#include <pthread.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr int kSchedulerPriority = 50;
constexpr long kSignalPollNanoseconds = 100'000'000;

class VmxControllerManager : public controller_manager::ControllerManager
{
public:
  using controller_manager::ControllerManager::ControllerManager;

  bool shutdown_hardware_components()
  {
    return resource_manager_ && resource_manager_->shutdown_components();
  }
};

sigset_t block_termination_signals()
{
  sigset_t signals;
  ::sigemptyset(&signals);
  ::sigaddset(&signals, SIGINT);
  ::sigaddset(&signals, SIGTERM);
  const int result = ::pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  if (result != 0) {
    std::fprintf(
      stderr, "Failed to block SIGINT/SIGTERM for managed VMX shutdown: %s\n",
      std::strerror(result));
    std::exit(1);
  }
  return signals;
}

}  // namespace

int main(int argc, char ** argv)
{
  // Block termination signals before ROS, VMX, or any worker thread exists.
  // A normal thread consumes them below so the real-time loop can be joined
  // before controller and hardware lifecycle shutdown starts.
  const auto termination_signals = block_termination_signals();
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager =
    std::make_shared<VmxControllerManager>(executor, "controller_manager");

  RCLCPP_INFO(
    controller_manager->get_logger(),
    "SIGINT/SIGTERM reserved for ordered VMX shutdown; vendor signal handlers cannot preempt "
    "the control lifecycle.");

  const bool use_sim_time = controller_manager->get_parameter_or("use_sim_time", false);
  if (use_sim_time || controller_manager->get_update_rate() == 0) {
    RCLCPP_ERROR(controller_manager->get_logger(),
      "VMX hardware requires wall-clock ROS stamps and a positive update rate");
    controller_manager.reset();
    rclcpp::shutdown();
    return 1;
  }
  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = controller_manager->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory) {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first) {
      RCLCPP_WARN(
        controller_manager->get_logger(), "Unable to lock the memory: '%s'",
        lock_result.second.c_str());
    }
  }

  RCLCPP_INFO(
    controller_manager->get_logger(), "update rate is %d Hz",
    controller_manager->get_update_rate());
  const int thread_priority =
    controller_manager->get_parameter_or<int>("thread_priority", kSchedulerPriority);
  RCLCPP_INFO(
    controller_manager->get_logger(),
    "Spawning controller_manager RT thread with scheduler priority: %d", thread_priority);

  std::atomic_bool control_loop_running {true};
  std::atomic_bool executor_running {true};
  std::thread control_thread(
    [controller_manager, thread_priority, &control_loop_running]() {
      if (!realtime_tools::configure_sched_fifo(thread_priority)) {
        RCLCPP_WARN(
          controller_manager->get_logger(),
          "Could not enable FIFO RT scheduling policy: error <%i>(%s).", errno,
          std::strerror(errno));
      } else {
        RCLCPP_INFO(
          controller_manager->get_logger(),
          "Successful set up FIFO RT scheduling policy with priority %i.", thread_priority);
      }

      const auto period =
      std::chrono::nanoseconds(1'000'000'000 / controller_manager->get_update_rate());
      auto previous_iteration = std::chrono::steady_clock::now();
      auto next_iteration_time = previous_iteration;

      while (control_loop_running.load(std::memory_order_acquire) && rclcpp::ok()) {
        const auto current_time = controller_manager->now();
        const auto iteration = std::chrono::steady_clock::now();
        const auto measured_period = rclcpp::Duration(
          std::chrono::duration_cast<std::chrono::nanoseconds>(iteration - previous_iteration));
        previous_iteration = iteration;

        controller_manager->read(current_time, measured_period);
        controller_manager->update(current_time, measured_period);
        controller_manager->write(current_time, measured_period);

        next_iteration_time += period;
        // Do not burst through missed cycles or wait on a wall-clock adjustment.
        const auto finished = std::chrono::steady_clock::now();
        if (next_iteration_time < finished) {
          next_iteration_time = finished + period;
        }
        std::this_thread::sleep_until(next_iteration_time);
      }
    });

  executor->add_node(controller_manager);
  std::thread executor_thread(
    [executor, &executor_running]() {
      executor->spin();
      executor_running.store(false, std::memory_order_release);
    });

  int received_signal = 0;
  while (rclcpp::ok() && executor_running.load(std::memory_order_acquire)) {
    const struct timespec timeout {0, kSignalPollNanoseconds};
    const int result = ::sigtimedwait(&termination_signals, nullptr, &timeout);
    if (result == SIGINT || result == SIGTERM) {
      received_signal = result;
      RCLCPP_INFO(
        controller_manager->get_logger(), "Managed shutdown requested by %s.",
        result == SIGINT ? "SIGINT" : "SIGTERM");
      break;
    }
    if (result < 0 && (errno == EAGAIN || errno == EINTR)) {
      continue;
    }
    RCLCPP_ERROR(
      controller_manager->get_logger(), "sigtimedwait failed during managed VMX operation: %s",
      std::strerror(errno));
    break;
  }

  control_loop_running.store(false, std::memory_order_release);
  executor->cancel();
  control_thread.join();
  executor_thread.join();

  // Keep the ROS context valid while controller nodes are removed from their
  // executor. Destroy the manager before rclcpp::shutdown so its registered
  // pre-shutdown callback cannot repeat the lifecycle transitions.
  const bool context_valid = rclcpp::ok();
  const bool controllers_stopped = context_valid && controller_manager->shutdown_controllers();
  const bool hardware_stopped = context_valid && controller_manager->shutdown_hardware_components();
  if (!controllers_stopped || !hardware_stopped) {
    RCLCPP_ERROR(
      controller_manager->get_logger(),
      "Managed VMX shutdown failed (controllers=%s hardware=%s).",
      controllers_stopped ? "stopped" : "failed",
      hardware_stopped ? "stopped" : "failed");
  } else {
    RCLCPP_INFO(
      controller_manager->get_logger(),
      "Managed VMX shutdown completed before HAL resource destruction.");
  }

  if (controller_manager->get_node_base_interface()->get_associated_with_executor_atomic().load()) {
    executor->remove_node(controller_manager);
  }
  controller_manager.reset();
  rclcpp::shutdown();
  return received_signal != 0 && controllers_stopped && hardware_stopped ? 0 : 1;
}
