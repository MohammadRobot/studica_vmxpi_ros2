# Profile Template

Copy these files into `bringup/config/profiles/<new_profile_name>/` to create a new robot profile.

Example:

```bash
PROFILE=my_robot_4wd
mkdir -p bringup/config/profiles/${PROFILE}
cp bringup/config/profile_template/robot_profile.yaml bringup/config/profiles/${PROFILE}/
cp bringup/config/profile_template/robot_controllers.yaml bringup/config/profiles/${PROFILE}/
```

Then launch:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=my_robot_4wd gui:=true
```

`robot_profile.yaml` includes a `drive` section:

- `wheel_layout`: `diff`, `diff_4wd`, `mecanum`, or `omni`
- `controller_name`: must match a top-level controller key in `robot_controllers.yaml`
- `controller_type`: `diff_drive_controller/DiffDriveController` or `mecanum_drive_controller/MecanumDriveController`

`robot_profile.yaml` `hardware` section can also include:

- `lidar_type`: default YDLIDAR preset used in hardware mode when launch arg `lidar_type` is not provided.
