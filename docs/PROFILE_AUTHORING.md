# Profile Authoring Guide

Profiles define robot geometry, hardware mapping, and controller tuning.

Profile files:

- `bringup/config/profiles/<profile_name>/robot_profile.yaml`
- `bringup/config/profiles/<profile_name>/robot_controllers.yaml`

Template source:

- `bringup/config/profile_template/robot_profile.yaml`
- `bringup/config/profile_template/robot_controllers.yaml`

## 1. Create a new profile

Recommended (scripted):

```bash
cd ~/ros2_ws/src/studica_vmxpi_ros2
scripts/create_profile.sh my_robot_4wd
```

Optional clone from an existing profile:

```bash
scripts/create_profile.sh my_robot_4wd --from-profile training_4wd
```

Manual copy approach:

```bash
cd ~/ros2_ws/src/studica_vmxpi_ros2
PROFILE=my_robot_4wd
mkdir -p bringup/config/profiles/${PROFILE}
cp bringup/config/profile_template/robot_profile.yaml bringup/config/profiles/${PROFILE}/
cp bringup/config/profile_template/robot_controllers.yaml bringup/config/profiles/${PROFILE}/
```

Profile name rules:

- Allowed characters: letters, numbers, `_`, `-`
- Example valid names: `training_4wd`, `my_robot_v1`, `chassis-a`

## 2. Edit `robot_profile.yaml`

Set physical and hardware values:

- `xacro.*`: base dimensions, mass, wheel/caster geometry, laser mounting height.
- `drive.*`: drive layout and controller selection.
  - `wheel_layout`: `diff` | `diff_4wd` | `mecanum` | `omni`
    - `omni` uses X-drive wheel mounting (45 deg wheel yaw in URDF).
  - `controller_name`: name used in `robot_controllers.yaml`
  - `controller_type`:
    - `diff_drive_controller/DiffDriveController` for `wheel_layout: diff` or `wheel_layout: diff_4wd`
    - `mecanum_drive_controller/MecanumDriveController` for `wheel_layout: mecanum` or `wheel_layout: omni`
- `hardware.*`: CAN ID, motor frequency, encoder ticks, wheel radius, speed scaling.
- Motor index mapping:
  - Valid values are `0..3` for active motors.
  - Use `-1` for an unused motor slot.
  - At least one left motor and one right motor must be active.
- For `wheel_layout: diff_4wd`, `wheel_layout: mecanum`, or `wheel_layout: omni`, all four motors must be active (`>= 0`).
- Set every `invert_*` field explicitly as `true` or `false`.

## 3. Edit `robot_controllers.yaml`

Set controller behavior:

- Keep required top-level keys:
  - `controller_manager`
  - the drive controller key from `drive.controller_name`
- For diff drive (`controller_type: diff_drive_controller/DiffDriveController`):
  - tune `wheel_separation`, `wheel_radius`, velocity/acceleration limits, covariance, publish rate
- For holonomic layouts (`controller_type: mecanum_drive_controller/MecanumDriveController`):
  - set wheel joint names (`front_left_*`, `front_right_*`, `rear_left_*`, `rear_right_*`)
  - set `kinematics.wheels_radius`
  - set `kinematics.sum_of_robot_center_projection_on_X_Y_axis` (`lx + ly` from robot center to wheel center)
    - Example from current URDF wheel origin: `abs(x_joint) + abs(y_joint)`
    - `class_mecanum`: `0.35`, `class_omni`: `0.33`

## 4. Validate profile

Build and source:

```bash
cd ~/ros2_ws
colcon build --packages-select studica_vmxpi_ros2
source install/setup.bash
```

Run a short launch validation:

```bash
timeout 10s ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=mock robot_profile:=my_robot_4wd gui:=false
```

Expected result:

- No immediate schema error from `bringup.launch.py`.
- If a key/type/range is wrong, launch exits with a clear error message naming the file and field.

Also run standalone profile lint:

```bash
python3 scripts/validate_profiles.py --profiles-dir bringup/config/profiles
```

Or run the full local checks used by pre-commit:

```bash
scripts/check_project.sh
```

## 5. Classroom workflow

1. Keep one profile per robot build.
2. Commit profile changes with a short calibration note in commit message.
3. Test profile in `mode:=gz_sim` before hardware mode.
