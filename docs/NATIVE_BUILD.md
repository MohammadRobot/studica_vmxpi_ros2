# Native ARM64 build without Docker

Local recipe candidate, updated 2026-11-10. The current dirty changes have not
been built on ARM64 or deployed. A build is not a release activation. Use a
separate Ubuntu 22.04 ARM64 builder with Humble and the licensed VMXPi SDK; a Pi
can be that builder during separately authorized maintenance, never while driving.
No Docker is required on the robot.

## Prepare exact source

Review and commit the platform **and driver** changes first, then update
`dependencies/hardware.repos` to the reviewed driver SHA. Current pins intentionally
still identify the baseline: do not claim this uncommitted candidate is reproducible
from them. Preserve the independent repository histories and existing dated installs.

Create a dedicated workspace, clone the selected platform commit into
`src/studica_vmxpi_ros2`, then import its pinned dependencies:

```bash
vcs import "$NATIVE_WS/src" \
  < "$NATIVE_WS/src/studica_vmxpi_ros2/dependencies/hardware.repos"
python3 "$NATIVE_WS/src/studica_vmxpi_ros2/scripts/verify_hardware_checkout.py" \
  --workspace "$NATIVE_WS"
```

`NATIVE_WS` must be an absolute path to that new workspace. Acquire dependencies
before an offline build. If a private commit is unavailable remotely, transfer a
reviewed Git bundle and retain the expected origin metadata; do not silently
replace a pin. Required build tools and ROS packages are recorded in the versioned
builder/runtime profiles under `deployment/`. Run rosdep against `hardware.repos`
sources only; do not install the simulation stack on Pi. Preserve the working
vendor SDK at `/usr/local/include/vmxpi` and `/usr/local/lib/vmxpi`.

## Compile

Use a fresh terminal with no ROS overlay sourced:

```bash
bash "$NATIVE_WS/src/studica_vmxpi_ros2/scripts/build_native_arm64.sh" "$NATIVE_WS"
```

The script checks ARM64/Ubuntu, clean source and exact dependency pins, sources
Humble, builds the LiDAR SDK into the workspace, then builds a merged production
install with one worker. It records exact sources and the native Debian package
inventory. It installs no OS packages, executes no tests or ROS nodes, and starts
no services. It never edits `/opt/studica/current`. It now requires a fresh
workspace without existing build/install/log/vendor outputs, so stale artifacts
cannot be mixed into the release. Existing directories cause refusal and are
never deleted. Use an exclusively owned build workspace. Source cleanliness and
the platform commit are rechecked after compilation.

Review and execute software tests only on a network-isolated, non-actuating host
without physical device passthrough, using the procedure in the
[developer guide](ROS2_DEVELOPER_GUIDE.md). On native builds, inspect which tests
require Gazebo and record any omissions. Measure build/runtime resource headroom
separately; the one-worker setting is not a performance qualification.

## Development bundle

After reviewing native test results and artifact dependencies:

```bash
python3 "$NATIVE_WS/src/studica_vmxpi_ros2/scripts/build_release_bundle.py" \
  --native --source-root "$NATIVE_WS/src/studica_vmxpi_ros2" \
  --install-prefix "$NATIVE_WS/install" --vmxpi-sdk-root /usr/local \
  --dpkg-inventory "$NATIVE_WS/native-dpkg-inventory.tsv" \
  --output-dir "$NATIVE_WS/release-artifacts"
python3 "$NATIVE_WS/src/studica_vmxpi_ros2/scripts/verify_release_artifacts.py" \
  --output-dir "$NATIVE_WS/release-artifacts"
```

The native profile records `native-colcon`, ARM64 and the native package inventory
hash; it does not fabricate a Docker digest. The bundle still requires real
AArch64 binaries (including `vmx_control_node`), production install layout, pinned
source metadata and SDK inventory. It rejects PC stubs. It remains a development
artifact with `DO_NOT_ACTIVATE` and is not signed or production approved.

The native build now writes `native-runtime-libraries.json`. Native packaging
independently rescans the staged install and embeds `metadata/runtime-libraries.json`
with a bound checksum. It records each ELF's content hash, DT_NEEDED dependencies
and RPATH/RUNPATH strings without executing binaries or loading libraries.

Missing dependencies, wrong architectures, ambiguous library names and dependency
symlink escapes reject the build. Approved external roots are Humble's library
directory, `/usr/lib/aarch64-linux-gnu` and the supplied VMX SDK library directory.
A YDLidar shared library left only under `workspace/vendor` is therefore rejected:
package the required library into the release after reviewing its licensing and
relocation requirements; do not add the build directory as an external target root.

This is dependency availability evidence, not a full ELF loader simulation. Search
path precedence, PT_INTERP, dlopen/plugin discovery and actual ABI compatibility
still need validation on a clean ARM64 target image. RPATH/RUNPATH are recorded,
not interpreted. The external libraries are hashed, not bundled or installed.
Native build output does not by itself prove a self-contained runtime bundle.
Keep the existing dated install and configuration available until a separately
approved pilot passes the physical and recovery gates in
[Minimal runtime](MINIMAL_RUNTIME.md) and [Release process](RELEASE_PROCESS.md).
