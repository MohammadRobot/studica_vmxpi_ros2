# Contributing

Use a separate development workspace and keep each repository clean. Do not
develop directly on a classroom or supported robot without first creating a
recoverable snapshot.

Before proposing a change:

1. Explain the user, API, safety, or maintenance problem being solved.
2. Preserve the public simulation/hardware interface or document a migration.
3. Add focused tests, including failure behavior for motion-related changes.
4. Run `./scripts/check_project.sh` and the affected Colcon tests.
5. Update documentation and `CHANGELOG.md`.

Dependency changes must use full commits in both manifests and CI. Never add a
floating branch to a release input.

Changes to arming, `/cmd_vel`, controller activation, HAL enable/disable,
timeouts, networking permissions, updates, or systemd shutdown are
safety-sensitive. They require a written hazard analysis and supervised VMXPi
hardware validation with the wheels clear before floor testing.
