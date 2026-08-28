# Dependency lock policy

The `.repos` files in this directory are release inputs, not development branch
lists. Every `version` must be a full 40-character Git commit.

`simulation.repos` is the minimum desktop/simulation overlay.
`hardware.repos` is the VMXPi overlay and adds hardware accessories and sensor
drivers. Repositories present in both manifests must use the same commit.

To update a dependency:

1. Review the upstream changes and license impact.
2. Test the exact commit in simulation and, where applicable, on VMXPi hardware.
3. Change all affected manifests and matching ROS CI checkout references.
4. Run `./scripts/check_project.sh`.
5. Record the change and compatibility impact in `CHANGELOG.md`.

Never replace a commit with `main`, `master`, a floating tag, or a pull-request
reference. Development can use a separate workspace overlay, but releases must
build from these locks.
