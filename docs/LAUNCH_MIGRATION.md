# Launch Migration Guide

Use `bringup.launch.py` as the single runtime entry point.

## Unified launch format

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=<gz_sim|hardware|mock|gazebo_classic> \
  robot_profile:=<profile_name> \
  gui:=<true|false>
```

## Legacy to unified command mapping

| Legacy command | Unified command |
|---|---|
| `ros2 launch studica_vmxpi_ros2 robot_gz_sim.launch.py gui:=true use_gz_sim:=true` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim gui:=true` |
| `ros2 launch studica_vmxpi_ros2 robot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware` |
| `ros2 launch studica_vmxpi_ros2 robot_gazebo_classic.launch.py use_gazebo_classic:=true` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gazebo_classic` |
| `ros2 launch studica_vmxpi_ros2 robot_bringup.launch.py use_hardware:=true` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware` |

## Wrapper status

- `robot_gz_sim.launch.py`: compatibility wrapper for existing scripts and labs.
- `robot_gazebo_classic.launch.py`: compatibility wrapper for Gazebo Classic workflows.
- `robot_bringup.launch.py`: compatibility wrapper for mixed/legacy system behavior.
- `nav2_*` wrappers remain convenience launchers for mapping/navigation workflows.

Deprecation policy dates:

- Since March 3, 2026: wrappers are compatibility only.
- Not planned for removal before January 1, 2027.
- New runtime features should be added only through `bringup.launch.py`.

## Recommended migration order

1. Replace launch command with `bringup.launch.py` and explicit `mode`.
2. Keep `robot_profile` explicit in every class/lab command.
3. Remove old boolean flags (`use_gz_sim`, `use_hardware`, `use_gazebo_classic`) from teaching material.
