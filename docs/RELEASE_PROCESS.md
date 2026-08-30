# Release process

The project is currently pre-production. This process establishes repeatable
development releases before over-the-air installation is enabled.

## Release inputs

- A clean `studica_vmxpi_ros2` commit.
- Exact commits in `dependencies/simulation.repos` and
  `dependencies/hardware.repos`.
- A package version matching a dated `CHANGELOG.md` section.
- A qualified Ubuntu/ROS/platform support matrix.
- Green source, simulation, ARM64 build, and hardware-in-the-loop gates.

Branches, moving tags, and uncommitted robot files are not release inputs.

## Preparation

1. Reconcile all robot-side experiments into reviewed repository commits.
2. Update dependency commits and matching CI checkout references.
3. Run `./scripts/check_project.sh`.
4. Import the selected `.repos` manifest into an empty workspace.
5. Build and test without sourcing another development workspace.
6. Run the hardware acceptance checklist on a bench robot with wheels clear.
7. Set the package version and move `Unreleased` notes to that dated version.

## Artifacts

Each release must publish:

- An immutable ARM64 application bundle or Debian repository snapshot.
- Source archive and exact `.repos` manifests.
- SHA-256 checksum list and signature.
- SPDX or CycloneDX software bill of materials.
- Build provenance/attestation.
- Release notes, supported hardware/OS matrix, known issues, and rollback notes.

The initial installation layout is:

```text
/opt/studica/releases/<version>/
/opt/studica/current -> /opt/studica/releases/<version>/
/var/lib/studica/update-state/
/var/log/studica/ or bounded journald storage
```

The release builder, not the robot, compiles source. The robot never runs
`git pull` as an install or boot action.

## Development ARM64 bundle

The checked-in builder creates the first application artifact without
installing or activating it. Build in a clean Ubuntu 22.04 arm64 hardware
workspace using the pinned `hardware.repos` sources. Use a merged, copied
install—not `--symlink-install`—and select the production filesystem profile:

### Isolated builder

The supported entry point is the fail-closed Docker/Buildx orchestrator:

```bash
./scripts/build_arm64_release.sh \
  --sdk-root <QUALIFIED_VMXPI_SDK_ROOT> \
  --output-dir "$STUDICA_WS/release-artifacts"
```

The SDK root must contain `include/vmxpi` and `lib/vmxpi`; the script mounts
only those two directories read-only and never copies them into the repository
or artifact. The Git source is also read-only. A temporary workspace receives
fresh clones of every `hardware.repos` dependency, and every commit, origin,
and clean-tree state is verified before compilation.

Source acquisition and the SDK-enabled build are separate containers. The
networked preparation container never receives an SDK mount. The subsequent
compile, test, and bundle container receives the prepared source and SDK as
read-only mounts and runs with `--network none`. After construction,
`scripts/verify_release_artifacts.py` checks the outer checksum, internal
payload checksums, source commit, builder provenance, platform, and explicit
activation denial without extracting the archive.

Native ARM64 Docker is required by default. `--allow-emulation` is an explicit
development escape hatch for a Buildx worker that already advertises
`linux/arm64`; the script does not install or register QEMU. Use `--check-only`
to validate the worker without building anything.

The builder uses `deployment/arm64-builder.Dockerfile` and the versioned
`deployment/arm64-builder-v1.json` contract. Its official Ubuntu Jammy base is
pinned by dated tag and complete OCI digest, and the ROS APT bootstrap package
is pinned by version and SHA-256. The upstream references are the
[official Ubuntu image tags](https://hub.docker.com/_/ubuntu/tags?name=jammy)
and the
[ROS APT source releases](https://github.com/ros-infrastructure/ros-apt-source/releases).
Separate container targets produce the source build and the minimal target-root
`dpkg` inventory. The final metadata records the local builder image ID, base
image, exact installed packages, source pins, and VMXPi SDK content identity.

This is an isolated development builder, not yet a fully reproducible package
repository snapshot. Ubuntu and ROS APT indexes can change between runs; the
profile declares `apt_repository_snapshot: false`. Beta promotion remains
blocked until those repositories and every resolved binary version are locked
to a retained snapshot.

The manual-only `.github/workflows/arm64-development-release.yml` executes this
gate on a protected, ephemeral native ARM64 runner. Provisioning, independent
approval, SDK qualification, execution, archival, and failure handling are in
[ARM64 development release worker](ARM64_RELEASE_WORKER.md). Do not attach a
persistent self-hosted runner to this public repository.

### Manual equivalent

The following commands describe the core operation performed inside the
isolated ARM64 builder. They are retained for diagnosis, not as the preferred
release path:

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths src \
  --merge-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DSTUDICA_PRODUCTION_INSTALL=ON
```

The production CMake profile fails unless the VMXPi hardware interface is
present. It excludes simulation launches, maps, RViz, examples, developer
documentation, and build tools. The release builder also rejects symlinked
developer installs, unexpected ROS packages, non-AArch64 hardware binaries,
world-writable files, escaping symlinks, and credential/cache paths.
It removes copied headers, static libraries, libtool archives, and CMake package
metadata from the staged runtime and records the removed paths in
`metadata/release.json`; the validated input install remains unchanged.

Export the complete package inventory from the matching minimal target root
filesystem, after build-only packages have been removed:

```bash
dpkg-query -W -f='${binary:Package}\t${Version}\t${Architecture}\n' \
  | LC_ALL=C sort > dpkg-inventory.tsv
```

Then build the deterministic archive from a clean Git commit:

```bash
./scripts/build_release_bundle.py \
  --install-prefix <MERGED_ARM64_INSTALL> \
  --vmxpi-sdk-root /usr/local \
  --dpkg-inventory dpkg-inventory.tsv \
  --builder-image-id sha256:<BUILDER_IMAGE_ID> \
  --output-dir "$STUDICA_WS/release-artifacts"
```

The output directory must be outside the Git repository so generated binaries
cannot dirty or accidentally enter the source release.

The archive contains the application overlay under
`/opt/studica/releases/<version>/install`, the exact package and source
inventories, a content-addressed inventory of the unmanaged VMXPi C++ SDK,
SPDX 2.3 SBOM, rollback contract, internal `SHA256SUMS`, and an external archive
checksum. The SDK inventory hashes every header beneath `include/vmxpi` plus
`lib/vmxpi/libvmxpi_hal_cpp.so`; the vendor SDK files themselves remain a
qualified operating-system input and are not copied into the application
bundle. Identical inputs and `SOURCE_DATE_EPOCH` produce identical archive
bytes.

This phase intentionally emits only a `development` artifact with
`activation_authorized: false` and a `DO_NOT_ACTIVATE` marker. It does not
create `/opt/studica/current`, install systemd units, sign the artifact, or
authorize robot motion. Beta/stable promotion remains blocked until the
physical drivetrain gate, signed-update verifier, and interruption rollback
tests pass.

## Channels and rollout

| Channel | Audience | Promotion requirement |
|---|---|---|
| `development` | Maintainers and bench robots | Source and simulation gates |
| `beta` | Named pilot robots | Hardware gates and rollback test |
| `stable` | Supported classroom fleet | Successful beta soak and support approval |

Rollouts are staged. A release can be paused globally, and every device retains
one known-good version. Automatic installation is opt-in and restricted to a
maintenance window while the robot is disarmed.

## Release gate

Do not tag or publish when any of the following is true:

- CI or hardware acceptance is red or inconclusive.
- A dependency is not pinned to a full commit.
- The release was built from a dirty workspace.
- API compatibility impact is undocumented.
- Update interruption and rollback have not been tested.
- Security-critical vulnerabilities or default shared credentials remain.

Tags and published artifacts should be signed. Promotion changes channel
metadata; it does not rebuild the artifact.
