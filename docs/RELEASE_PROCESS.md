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
