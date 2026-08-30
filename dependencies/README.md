# Dependency lock policy

The `.repos` files in this directory are release inputs, not development branch
lists. Every `version` must be a full 40-character Git commit.

`simulation.repos` is the minimum desktop/simulation overlay.
`hardware.repos` is the VMXPi overlay and adds hardware accessories and sensor
drivers. Repositories present in both manifests must use the same commit.

The `apt/` directory separates binary development dependencies from the
headless robot-core `package.xml`:

- `development-core.txt` contains source build, test, and headless ROS tools;
- `development-desktop.txt` contains classroom GUI, visualization, mapping,
  navigation, recording, and support tools;
- `simulation-harmonic.txt` contains Gazebo Harmonic packages used only in
  simulation mode.

These bundles are inputs to `scripts/setup_ubuntu.sh`; they are not production
image inputs. The production boundary is the separately validated
`deployment/vmxpi-runtime-packages-v1.json`. Keep every text bundle sorted,
with one APT package per line and no package duplicated between bundles.

To update a dependency:

1. Review the upstream changes and license impact.
2. Test the exact commit in simulation and, where applicable, on VMXPi hardware.
3. Change all affected manifests and matching ROS CI checkout references.
4. Run `./scripts/check_project.sh`.
5. Record the change and compatibility impact in `CHANGELOG.md`.

Never replace a commit with `main`, `master`, a floating tag, or a pull-request
reference. Development can use a separate workspace overlay, but releases must
build from these locks.

The isolated production-build candidate imports `hardware.repos` into a new
temporary workspace and runs `scripts/verify_hardware_checkout.py`. The build
stops if a checkout has the wrong commit, a different origin, or any generated
or local changes. See `deployment/arm64-builder-v1.json` for the versioned
container and input contract.
Dependency acquisition runs in a networked container that has no VMXPi SDK
mount; the verified prepared tree is then mounted read-only for the
network-disabled SDK build.
