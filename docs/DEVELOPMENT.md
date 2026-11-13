# Application development

Use the [ROS 2 developer guide](ROS2_DEVELOPER_GUIDE.md) for the interface contract,
PC build, isolated tests, standard launch commands and Python/C++ examples.

Keep platform development in `~/studica_ws`, student applications in `~/ros2_ws`,
and RoboHiTec adapters in `~/projects/robohitec-os`. Do not duplicate platform
packages in the student workspace. Source Humble and the platform install before
building student applications; never source the student overlay to build hardware.

Applications run on the PC. The Pi owns acquisition, low-level control, safety
supervision and odometry. Simulation and hardware use the same message interfaces
with explicit clock, robot profile and per-terminal DDS configuration.

For the recorded robot installation use [Robot Manual](ROBOT_MANUAL.md). For the
local, undeployed startup/update candidate see [Minimal runtime](MINIMAL_RUNTIME.md).
Native hardware compilation is documented in [Native build](NATIVE_BUILD.md).
