# Production safety acceptance

Software completion does not make the current wiring production-safe. Before enabling `studica-robot.target` at boot, the physical E-stop must independently remove Titan motor torque while VMXPi logic power remains available. DIO 8 remains status feedback. Record evidence against the exact release digest.

The qualification JSON accepted by `install_robot_platform.py --enable-autostart` is:

```json
{
  "schema_version": 1,
  "cold_boots_passed": 50,
  "independent_torque_removal": true,
  "failure_injection_passed": true,
  "zero_motion_all_boots": true,
  "tested_release_sha256": "<64 lowercase hex characters>"
}
```

For all 50 cold boots, vary E-stop pressed/released, Start held/released, NC Stop open/closed, DIO 8 disconnected, DIO 11 disconnected, joystick absent, network absent and companion absent. Every boot must produce zero wheel motion. Reject the release after any unexplained motion.

Run lifted-wheel tests for every mode transition and inject browser lease expiry, joystick loss, companion exit, Nav2 cancellation, manager exit, safety-supervisor exit, hardware process exit, shutdown and update interruption. Confirm zero output and disabled Titan. Query publisher counts in each mode and require exactly one publisher on `/robot/platform/cmd_vel` and exactly one safety-supervisor publisher on `/robot_base_controller/cmd_vel`.

Only then perform controlled floor testing, `office_nav` localization/navigation, full camera plus LiDAR profiling, update rollback, offline hotspot, authentication rejection and novice usability acceptance. Interrupt activation after the release-pointer switch, reboot, and verify `studica-update-recovery.service` restores the previous release before any robot service starts. Store the raw logs with the qualification record; the JSON gate is an index, not a substitute for evidence or independent review.
