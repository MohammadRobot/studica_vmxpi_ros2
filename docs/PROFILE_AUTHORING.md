# Advanced Robot Profiles

Profiles are an instructor/advanced feature. The classroom default is always
`class_4wd`; application code continues to publish `/cmd_vel` and subscribe to the
standard feedback topics regardless of the controller used internally.

The retained variants are:

| Profile | Layout | Intended use |
|---|---|---|
| `class_4wd` | four-wheel differential | tested course default |
| `class_2wd` | two-wheel differential | advanced variant lesson |
| `class_mecanum` | four mecanum wheels | advanced holonomic lesson |
| `class_omni` | four omni wheels | advanced holonomic lesson |

Do not rename a variant to make it the default. Select it explicitly through
advanced `bringup.launch.py` only after its simulation acceptance passes.

## Profile files

Each directory below `bringup/config/profiles/<PROFILE>/` contains:

- `robot_profile.yaml`: geometry, layout, sensor defaults, motor mapping, and
  hardware safety settings;
- `robot_controllers.yaml`: controller joints, kinematics, rate, limits, and
  timeout.

The schema examples are in `bringup/config/profile_template/`. Runtime launch
validates values before building the robot description.

## Create a profile

Use a workspace variable instead of a machine-specific path:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/create_profile.sh my_robot_4wd --from-profile class_4wd
```

Names may contain letters, numbers, underscores, and hyphens. A name should
describe physical hardware, not a developer group or temporary experiment.

Review every copied number. Cloning a profile does not prove that its dimensions,
motor channels, signs, encoder scale, or temperature limit are correct for a new
robot.

## Physical geometry and calibrated kinematics

Set measured physical values under `xacro` and `drive`:

```yaml
xacro:
  base_length: <METRES>
  base_width: <METRES>
  base_height: <METRES>
  ground_clearance: <METRES>
  wheelbase: <FRONT_TO_REAR_WHEEL_CENTRES_METRES>
  wheel_track: <LEFT_TO_RIGHT_WHEEL_CENTRES_METRES>
  overall_length: <OUTER_ENVELOPE_METRES>
  overall_width: <OUTER_ENVELOPE_METRES>
  use_chassis_mesh: false
drive:
  wheel_layout: diff_4wd
  controller_name: robot_base_controller
  controller_type: diff_drive_controller/DiffDriveController
  wheel_radius_m: <MEASURED_METRES>
```

`base_link` is the ground-plane origin centred between the four wheel contact
points. `chassis_link` is centred inside the measured body: X points forward,
Y left, and Z up. Wheelbase and wheel track are independent of body length and
width. This matters when wheels sit inside the body ends or outside its sides.

Set `use_chassis_mesh: false` when the bundled visual mesh is not the measured
robot. The generated box then uses the same dimensions as collision and
inertia. Enable a custom mesh only after its native scale matches the physical
body.

LiDAR, camera, and IMU XYZ/RPY values are measured relative to `chassis_link`.
Angles use radians. `laser_pos_z` identifies the scan plane; any retained
`laser_frame_z` is a visual-model-to-scan offset and should be zero when the
scan plane itself was measured. Simulation sensors attach to the same URDF
links used by hardware so one TF tree describes both modes.

`drive.wheel_radius_m` is the single source for URDF geometry, hardware encoder
conversion, and injected controller kinematics. Do not add `wheel_radius` or
`kinematics.wheels_radius` to `robot_controllers.yaml`.

For differential-drive floor calibration, retain the measured physical radius
and adjust the controller's existing `left_wheel_radius_multiplier` and
`right_wheel_radius_multiplier`. Start with equal values; use different values
only when repeated forward and reverse measurements prove a side-specific
scale error. These dimensionless corrections do not create a second physical
wheel-radius setting.

Likewise, physical `xacro.wheel_track` places the wheel links, while a
differential controller's `wheel_separation` is its effective turning value. A
four-wheel skid-steer platform can require an effective value larger than its
measured track because the tyres scrub during a turn. Do not overwrite a
validated controller value merely to make those numbers look equal.

Measure the loaded wheel at several orientations. Leave
`hardware.wheel_radius_calibrated: false` until an instructor reviews the value.
Hardware motion must remain unavailable while false.

## Layout and controller pairing

Valid combinations are:

- `diff` or `diff_4wd` with
  `diff_drive_controller/DiffDriveController`;
- `mecanum` or `omni` with
  `mecanum_drive_controller/MecanumDriveController`.

The omni URDF uses an X-drive wheel orientation. A holonomic controller needs
all four joint names and
`kinematics.sum_of_robot_center_projection_on_X_Y_axis`, equal to the absolute
wheel-joint X offset plus its absolute Y offset.

Controller files must keep a finite command timeout and conservative velocity
and acceleration limits. Launch supplies the public `/cmd_vel` adapter and
`/odom` alias; profile authors must not introduce another command publisher.

## Hardware mapping

For a physical profile, review:

- Titan CAN ID and motor frequency;
- encoder ticks per rotation;
- active motor channels from `0` through `3`;
- motor and encoder inversion flags;
- maximum wheel angular velocity;
- MCV2 PID type, sensitivity, and required-support flag;
- encoder feedback warning/error timeouts;
- temperature limit and freshness timeout;
- LiDAR type and sensor mounting transforms.

Use `-1` only for a genuinely unused channel. At least one left and one right
motor are required. `diff_4wd`, `mecanum`, and `omni` require all four channels.
Set every inversion value explicitly.

The `class_4wd` hardware profile requires `velocity_pid`, PID type `mcv2`, and
confirmed firmware support. There is no silent open-loop fallback.

## Validate without motion

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
python3 scripts/validate_profiles.py \
  --profiles-dir bringup/config/profiles
./scripts/check_project.sh
```

Then build and start mock mode:

```bash
cd "$STUDICA_WS"
colcon build --symlink-install --packages-select studica_vmxpi_ros2
source install/setup.bash
timeout 15s ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=mock robot_profile:=my_robot_4wd gui:=false
```

Expected: profile validation succeeds, controllers load, standard topics are
created, and no physical driver is opened. A timeout exit after a healthy launch
is expected from this short smoke command.

## Simulation acceptance

Select the new profile explicitly:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim robot_profile:=my_robot_4wd gui:=true
```

Require:

- correct wheel placement and rotation axes;
- active drive, joint-state, and IMU controllers;
- `/cmd_vel` accepts `geometry_msgs/msg/Twist`;
- `/odom`, `/imu`, `/scan`, `/joint_states`, and TF are present;
- straight and rotational command signs are correct;
- timeout stops motion;
- no second publisher commands `/cmd_vel`.

Record the profile name and validation result with the change.

## Physical acceptance

Never move directly from YAML editing to a floor test. Follow the instructor
order in [Supervised hardware](HARDWARE.md): inspect, read-only health checks,
guarded lifted-wheel validation, then measured low-speed floor calibration.

Any change to motor channels, inversion, encoder conversion, PID, safety
timeouts, wheel radius, or wheel separation invalidates the previous physical
baseline and requires a new signed report.
