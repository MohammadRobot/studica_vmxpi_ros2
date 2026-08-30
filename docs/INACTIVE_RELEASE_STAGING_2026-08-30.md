# Inactive release staging evidence: 2026-08-30

This record covers inactive installation transport only. It does not authorize
activation, systemd autostart, ROS hardware launch, or motor output.

## Inputs

- VMXPi host: `vmx`, AArch64, Ubuntu 22.04.
- Release source commit:
  `4ffe82ee33fbf0c794e8a3d1a532fead056c4009`.
- Release version: `0.1.0-dev.4ffe82ee33fb`.
- Archive SHA-256:
  `ad64986f8a7692c03033af67aaefe9aefbacc6ae5548de5347c7530b85675763`.
- GitHub Actions run: `33313709492`; artifact ID: `9733279164`.
- Inactive installer commit:
  `675f2f72fc2d10c7484379fb2cc6317383ece014`.
- Inactive installer SHA-256:
  `777cb4c36c9d16a0fc056a8dd9e0a4a05d0d7c677a2300f4630c493a45335a9b`.
- Cryptographic publisher signature: not implemented and not verified.

The release and installer were transferred to a private directory beneath
`/home/vmx/studica-release-staging/`. The VMXPi independently passed the outer
archive checksum and `verify_release_artifacts.py` checks for internal
checksums, source provenance, Ubuntu 22.04/Humble ARM64 platform, development
channel, and activation denial.

## Result

`install_inactive_release.py install` completed successfully and staged:

```text
/opt/studica/releases/0.1.0-dev.4ffe82ee33fb
```

The installer returned success only after its post-extraction checksum pass,
atomic rename, versioned inactive-state record, and before/after equality
checks for both protected paths:

```text
/opt/studica/current
/var/lib/studica/update-state/previous-release
```

The independent post-install `sha256sum -c --quiet SHA256SUMS` returned zero.
`/opt/studica/current` was absent. No Studica/robot systemd unit was listed and
no ROS 2, controller-manager, robot-state-publisher, Studica, or Titan process
was found; the process search returned only its own audit shell because the
search command contained the staged path. No service start, ROS invocation, or
motion command was issued.

The VMXPi had 13 GiB free on `/` at 78% use. Load after installation was
`0.45, 0.18, 0.19`.

## Follow-up observation

TCP port 22 became intermittently unreachable after several short SSH/SCP
connections even while ICMP continued to respond. One quiet interval restored
SSH long enough to complete the installation. The cause is not confirmed.
Before relying on remote updates, inspect SSH daemon logs, firewall rate-limit
rules, connection limits, network latency, and power/undervoltage evidence from
a stable local-console or single persistent SSH session.

Activation remains blocked by the physical drivetrain gate, signed-update
verification, and power-interruption rollback qualification.
