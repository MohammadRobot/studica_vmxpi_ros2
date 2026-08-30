# ARM64 development release worker

This runbook provisions the off-robot worker used to execute the isolated
Ubuntu 22.04/ROS 2 Humble release builder. It produces a verified development
artifact; it does not install software, contact a robot, or authorize motion.

The repository contains the workflow and checks, but no dedicated production
worker has been provisioned and no real ARM64 artifact has been archived yet.

For the current one-maintainer development phase, a VMXPi may be used as a
temporary, ephemeral builder because no separate ARM64 host is available. This
exception is limited to non-activatable `development` artifacts. Keep the
E-stop pressed, Titan motor power disconnected, and robot-control services
stopped. Use `--build-workers 1`, monitor the input rail and kernel
undervoltage reports, and stop after any brownout or reboot. A VMXPi build does
not satisfy the independent production release-worker or promotion gate.

## Trust boundary

For production release work, use a dedicated, disposable native ARM64 Linux
host. Outside the temporary development exception above, do not register the
VMXPi robot as a runner or use a general development computer that contains
unrelated credentials. Membership in the Docker group is effectively
root-level access, so the entire release job and its approved source commit are
trusted inputs.

The workflow is intentionally manual-only. GitHub warns that a persistent
self-hosted runner can retain compromise between jobs, especially for a public
repository. Register the worker with `--ephemeral`, allow it to execute exactly
one job, forward its runner logs, then destroy or reimage it. See GitHub's
[self-hosted runner security guidance](https://docs.github.com/en/actions/reference/security/secure-use#hardening-for-self-hosted-runners)
and
[ephemeral runner reference](https://docs.github.com/en/actions/reference/runners/self-hosted-runners#ephemeral-runners-for-autoscaling).

The build has these network boundaries:

| Stage | Network | Product source | VMXPi SDK |
|---|---:|---:|---:|
| Container image construction | Enabled | Build-context allowlist only | Not mounted |
| Pinned dependency acquisition | Enabled | Read-only | Not mounted |
| Compilation, tests, and bundle creation | Disabled | Read-only prepared tree | Read-only |
| Artifact upload | GitHub only | Not executed | Not mounted in a container |

The host runner account can still read the SDK and control Docker. Environment
approval, protected `main`, exact-commit confirmation, full-SHA action pins,
and disposal after one job are therefore mandatory controls—not optional
conveniences.

## Worker prerequisites

- A native AArch64 host; emulation is rejected by the release workflow.
- A clean, disposable Linux installation with current security updates.
- Docker Engine with Buildx, accessible to the dedicated runner account.
- GitHub Actions Runner version 2.327.1 or newer. The pinned artifact action
  uses the Node.js 24 action runtime.
- At least 4 CPU cores, 8 GiB RAM, 30 GiB free temporary storage, and reliable
  internet access for image construction and dependency acquisition.
- A qualified, licensed VMXPi SDK copied from the approved Studica platform
  image or vendor distribution.

Do not copy a runner registration token, SDK binary, GitHub credential, SSH
key, or artifact into this repository.

## Qualify the SDK input

The environment variable points to a root with this exact minimum layout:

```text
/opt/studica-builder-inputs/vmxpi-sdk/
├── include/vmxpi/VMXPi.h
└── lib/vmxpi/libvmxpi_hal_cpp.so
```

The SDK root, both mounted directories, and all files must be owned by an
administrator and not writable by the runner. Keep the SDK outside the Actions
checkout and runner temporary directory. Record its supplier, license source,
supported VMXPi image, and qualification date in the private manufacturing or
release record. The generated bundle records content hashes for every accepted
SDK header and the runtime library without redistributing those files.

Before registration, validate the local worker from a clean checkout:

```bash
./scripts/build_arm64_release.sh \
  --sdk-root /opt/studica-builder-inputs/vmxpi-sdk \
  --output-dir /tmp/studica-arm64-preflight-output \
  --build-workers 1 \
  --check-only
```

The output directory is not created in check-only mode.

## Configure GitHub

Before a runner is allowed online:

1. Protect `main`; require review and passing CI before merge.
2. Create an environment named `arm64-development-release`.
3. Restrict that environment to `main`, add a required reviewer who is not the
   person dispatching the workflow, prevent self-review, and disable
   administrator bypass where the repository plan supports it.
4. Add environment variable `VMXPI_SDK_ROOT` with value
   `/opt/studica-builder-inputs/vmxpi-sdk`. This is a path, not a secret.
5. Restrict the runner group to this repository where runner groups are
   available.

Environment protection rules are evaluated before the build job is sent to the
runner. See GitHub's
[deployment environment documentation](https://docs.github.com/en/actions/reference/workflows-and-actions/deployments-and-environments).
All external actions in the repository are pinned to full commit SHAs, as
recommended by GitHub's
[secure-use reference](https://docs.github.com/en/actions/reference/security/secure-use#using-third-party-actions).

## Execute one release build

1. Review the exact current `main` commit and all dependency locks.
2. Open **Actions → ARM64 Development Release → Run workflow**.
3. Select `main` and paste its complete 40-character commit into
   `approved_commit`.
4. Have the independent environment reviewer confirm the same commit and
   approve the waiting job.
5. Register or start a fresh ARM64 runner with the default `self-hosted`,
   `linux`, and `ARM64` labels plus `studica-arm64-release`. Include
   `--ephemeral` when running GitHub's generated `config.sh` command.
6. Monitor the job, preserve the external runner logs, and allow GitHub to
   deregister the runner after its single job.
7. Destroy or reimage the worker. Do not return it to a shared runner pool.

The workflow independently verifies the typed commit, native architecture,
clean checkout, Docker access, read-only SDK permissions, builder preflight,
archive SHA-256, every internal payload checksum, source provenance, and both
activation-denial records. It retains the GitHub artifact for 14 days.

When the repository has only one maintainer, the independent-review step
cannot be claimed. Keep the environment and artifact in the development
channel, record the exception, and add the reviewer before beta promotion.

After downloading and unpacking the GitHub artifact wrapper, verify it again:

```bash
./scripts/verify_release_artifacts.py \
  --output-dir <UNPACKED_ARTIFACT_DIRECTORY> \
  --expected-commit <APPROVED_40_CHARACTER_COMMIT>
```

Archive the `.tar.gz`, its `.sha256` file, workflow URL, run ID, runner logs,
reviewer identity, SDK qualification record, and test result in the controlled
release record. The artifact must retain `channel: development`,
`activation_authorized: false`, and `metadata/DO_NOT_ACTIVATE`.

## Failure handling

- A failure produces no releasable artifact. Do not upload files manually to
  make a failed run look complete.
- If the SDK boundary or runner may have been compromised, discard the worker,
  requalify the SDK source, and inspect the workflow logs before retrying.
- If source preparation fails, update a reviewed dependency lock; never change
  the worker checkout in place.
- If artifact verification fails, retain logs for diagnosis but destroy the
  output and worker.
- Never copy a development bundle to `/opt/studica`, change
  `/opt/studica/current`, or test it on the live robot. Activation remains
  blocked until the physical drivetrain, signed-update, and rollback gates
  pass.
