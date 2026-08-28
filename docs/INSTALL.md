# Installation

This guide prepares Ubuntu 22.04 for the ROS 2 Humble classroom. Simulation is
the default. Hardware mode is for the arm64 VMXPi robot image and requires the
Studica vendor SDK to be installed already.

After installation, continue with [Quick start](QUICK_START.md) for the shortest
simulation and physical mapping launch sequences.

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

Use the dedicated `~/studica_ws` workspace. Separating this pinned simulation
stack from a general-purpose ROS workspace avoids dependency and overlay
collisions. `STUDICA_WS` keeps the remaining commands independent of usernames.

```bash
export STUDICA_WS="$HOME/studica_ws"
mkdir -p "$STUDICA_WS/src"
git clone https://github.com/MohammadRobot/studica_vmxpi_ros2.git \
  "$STUDICA_WS/src/studica_vmxpi_ros2"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
```

If your instructor supplied the repository another way, it must still be at
`$STUDICA_WS/src/studica_vmxpi_ros2`.

## 2. Check without changing the computer

For a development PC:

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
   `colcon build --symlink-install`;
7. runs first-party non-motion tests.

The script never edits `.bashrc`, launches Gazebo, or publishes `/cmd_vel`.

For a later manual simulation rebuild, select Harmonic before invoking
`colcon`; otherwise the pinned Humble overlay assumes Gazebo Fortress:

```bash
export GZ_VERSION=harmonic
cd "$STUDICA_WS"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install \
  --packages-select gz_ros2_control studica_vmxpi_ros2 \
  --allow-overriding gz_ros2_control
```

## 4. Configure simulation and source each terminal

Generate the loopback-only Cyclone DDS profile once after installation:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py sim --domain-id 1
```

After that, every new simulation terminal needs:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
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

The simulation environment selects loopback discovery and exports
`GZ_VERSION=harmonic`, so later builds select the Harmonic compatibility path.

For a PC-to-VMXPi session, generate a peer profile separately on each computer
with its current interface and address. The helper validates the local address,
prints the narrow UFW rule, and never edits shell startup files. Follow
[Networking and Cyclone DDS](NETWORKING.md) for Wi-Fi and Ethernet examples.

Use one workspace overlay per terminal. Check for an older automatically sourced
workspace with:

```bash
printenv AMENT_PREFIX_PATH | tr ':' '\n'
```

The current workspace should precede its dependencies. If an unrelated or older
workspace appears, remove its `install/setup.bash` line from the shell startup
file and open a new login session before rebuilding. Sourcing `/opt/ros/humble`
does not reliably erase paths inherited from an earlier overlay.

## Recover missing system dependencies

If a launch reports a missing Debian-provided ROS package such as
`diagnostic_aggregator`, install all declared dependencies instead of repairing
them one at a time:

```bash
export STUDICA_WS="$HOME/studica_ws"
source /opt/ros/humble/setup.bash
sudo apt update
rosdep install --from-paths "$STUDICA_WS/src" --ignore-src \
  --rosdistro humble -r -y
```

Confirm the package, source the workspace, and relaunch:

```bash
ros2 pkg prefix diagnostic_aggregator
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Installing a runtime dependency does not normally require a rebuild. A Python
`pkg_resources is deprecated` warning from the Ubuntu `rosdep` launcher is not
the failure if the command ends with `All required rosdeps installed
successfully`.

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
Generate and test the selected peer DDS profile, then continue with
[Supervised hardware](HARDWARE.md), not with an improvised launch.

## What the manifests mean

`dependencies/simulation.repos` contains only simulation/course dependencies.
`dependencies/hardware.repos` adds the sensor drivers and optional accessory
package. Vendor repositories are pinned and are not modified by this project.

Every repository is locked to a full commit. Simulation and hardware manifests
must use the same commit for shared first-party packages. Update a pin only in a
reviewed pull request that updates both manifests, CI checkout references, and
the release notes together. This prevents a branch update from changing a
classroom or robot installation without a new product release.

## Manual build after editing code

```bash
cd "$STUDICA_WS"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build one package while working on a lab:

```bash
colcon build --symlink-install --packages-select <PACKAGE_NAME>
```

Always source `install/setup.bash` again after a build.

Keep the build mode consistent. The project installer creates a symlink-install
workspace, so its later builds should retain `--symlink-install`. If an existing
workspace was originally built with plain `colcon build`, continue without the
flag. Switching modes in the same `build/` and `install/` trees can cause an
`existing path cannot be removed: Is a directory` error. See
[Troubleshooting](TROUBLESHOOTING.md#colcon-cannot-create-a-symbolic-link).

Normal project behavior belongs in the sibling `studica_robot_apps` package,
not in the platform or vendor dependencies. Build that package selectively,
test it in simulation, then copy source only and build it natively on the
VMXPi. Follow [Application development and deployment](DEVELOPMENT.md).

## Uninstall scope

The setup script intentionally has no automatic uninstall operation. Removing
system packages or repositories can affect other ROS projects on the computer.
For a classroom machine, restore the prepared image; for a personal machine,
review installed packages with an instructor or system administrator first.

For failures, use [Troubleshooting](TROUBLESHOOTING.md) and include the exact
command plus its first error message when asking for help.
