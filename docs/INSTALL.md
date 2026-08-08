# Installation

This guide prepares Ubuntu 22.04 for the ROS 2 Humble classroom. Simulation is
the default. Hardware mode is for the arm64 VMXPi robot image and requires the
Studica vendor SDK to be installed already.

## Before you begin

You need:

- Ubuntu 22.04 with an `amd64` or `arm64` processor;
- internet access and a user account allowed to run `sudo`;
- at least 12 GiB free for simulation or 6 GiB for hardware;
- a normal terminal outside Conda or another Python environment;
- the supported VMXPi image and SDK for hardware mode.

The installer rejects a different Ubuntu release. ROS 2 Humble binary packages
target Ubuntu 22.04, and mixing platform releases makes classroom results
unpredictable.

## 1. Put the repository in a workspace

Choose the workspace once:

```bash
export STUDICA_WS="$HOME/ros2_ws"
mkdir -p "$STUDICA_WS/src"
git clone https://github.com/MohammadRobot/studica_vmxpi_ros2.git \
  "$STUDICA_WS/src/studica_vmxpi_ros2"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
```

If your instructor supplied the repository another way, it must still be at
`$STUDICA_WS/src/studica_vmxpi_ros2`.

## 2. Check without changing the computer

For a student PC:

```bash
./scripts/setup_ubuntu.sh --mode simulation --check-only
```

Expected final line:

```text
[setup] check-only complete; no files or packages were changed
```

For a VMXPi robot computer, an instructor may run:

```bash
./scripts/setup_ubuntu.sh --mode hardware --check-only
```

Hardware preflight also checks the arm64 architecture and VMXPi HAL headers and
library. A failure here means the vendor image or SDK must be repaired before
building the robot workspace.

## 3. Install simulation

```bash
./scripts/setup_ubuntu.sh --mode simulation
```

The script performs these visible, repeatable operations:

1. verifies Ubuntu, architecture, disk, network, and sudo access;
2. configures the official ROS 2 and Gazebo package repositories;
3. installs ROS 2 Humble Desktop and classroom tools;
4. imports `dependencies/simulation.repos` with `vcstool`;
5. pins `gz_ros2_control` to commit
   `a2d290e37be67ba082744e323339d82031f051c0`;
6. runs `rosdep`, validates profiles and launch syntax, and builds with
   `colcon --symlink-install`;
7. runs first-party non-motion tests.

The script never edits `.bashrc`, launches Gazebo, or publishes `/cmd_vel`.

## 4. Source each terminal

After installation, every new terminal needs:

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
```

Check the environment:

```bash
printenv ROS_DISTRO
ros2 pkg prefix studica_vmxpi_ros2
```

Expected values include `humble` and a path below `$STUDICA_WS/install`.

The setup script prints the exact source command at the end. Adding it to a
shell startup file is a personal choice; the project does not do that for you.

## 5. Prove idempotence

Run the same setup command a second time:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode simulation
```

Existing repositories are kept, packages are already satisfied, and the same
pinned overlay and build are reused. It must finish without duplicating source
entries or changing shell startup files.

## Non-interactive classroom imaging

For a machine already configured with passwordless sudo:

```bash
./scripts/setup_ubuntu.sh --mode simulation --non-interactive
```

The option fails instead of prompting if passwordless sudo is unavailable. It is
appropriate for a controlled classroom image, not a way to weaken sudo policy.

## Hardware installation

Only an instructor should run this on the VMXPi robot computer:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode hardware
```

Hardware mode imports:

- first-party drivers, monitoring, and optional accessories;
- the pinned Orbbec and YDLidar ROS drivers;
- the pinned YDLidar SDK source when the image does not already provide it.

It does not install Gazebo on the robot and does not start the HAL or motors.
Continue with [Supervised hardware](HARDWARE.md), not with an improvised launch.

## What the manifests mean

`dependencies/simulation.repos` contains only simulation/course dependencies.
`dependencies/hardware.repos` adds the sensor drivers and optional accessory
package. Vendor repositories are pinned and are not modified by this project.

The first-party repositories track `main` so their coordinated classroom APIs
stay together. The Harmonic control overlay and vendor snapshots use exact
commits to prevent an upstream update from changing a lesson unexpectedly.

## Manual build after editing code

```bash
cd "$STUDICA_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build one package while working on a lab:

```bash
colcon build --symlink-install --packages-select <PACKAGE_NAME>
```

Always source `install/setup.bash` again after a build.

## Uninstall scope

The setup script intentionally has no automatic uninstall operation. Removing
system packages or repositories can affect other ROS projects on the computer.
For a classroom machine, restore the prepared image; for a personal machine,
review installed packages with an instructor or system administrator first.

For failures, use [Troubleshooting](TROUBLESHOOTING.md) and include the exact
command plus its first error message when asking for help.
