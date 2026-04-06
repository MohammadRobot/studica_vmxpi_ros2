# System Architecture

This document explains how `studica_vmxpi_ros2` is composed and how launch modes connect runtime components.

## 1) Layers

- Launch orchestration: `bringup/launch/bringup.launch.py`
- Robot configuration: `bringup/config/profiles/<profile_name>/`
- Robot model: `description/urdf/*.xacro` + `description/ros2_control/*.xacro`
- Control and hardware abstraction: `ros2_control_node` + hardware plugins in `hardware/`
- Topic/API compatibility: `topic_adapter_node` in `src/topic_adapter_node.cpp`
- Optional features: LiDAR, camera, joystick, SLAM, Nav2 launch wrappers

## 2) Runtime Flow

```mermaid
flowchart TD
    U[User Launch Command] --> B[bringup.launch.py]
    B --> V{Validate robot_profile}
    V -->|ok| X[Xacro Build robot_description]
    V -->|error| E[Fail fast with schema/path error]

    X --> M{mode}

    M -->|gz_sim| GZ[robot_gz_sim.launch.py]
    GZ --> GZS[ros_gz_sim world + spawn]
    GZ --> BR[ros_gz_bridge /clock, /scan, /camera/color/*, /camera/depth/*]
    GZ --> SR[topic_adapter scan relay /scan_raw -> /scan]
    GZ --> CM1[controller_manager in gz_ros2_control]

    M -->|hardware| HW[robot_gz_sim.launch.py use_gz_sim false]
    HW --> CM2[ros2_control_node]
    CM2 --> VMX[vmx_system hardware plugin]

    M -->|mock| MK[robot_gz_sim.launch.py use_gz_sim false]
    MK --> CM3[ros2_control_node]
    CM3 --> DF[sim_system mock plugin]

    CM1 --> C[Controllers]
    CM2 --> C
    CM3 --> C

    C --> JSB[joint_state_broadcaster]
    C --> IMU[imu_sensor_broadcaster]
    C --> DBC[drive controller from profile]

    DBC --> ODOM[/<drive_controller>/odom or /<drive_controller>/odometry]
    CMD[/<drive_controller>/cmd_vel or /<drive_controller>/reference (TwistStamped)] --> DBC

    NAV[topic_adapter nav2 bridge] --> CMD
    NAV --> ODOM
    IMUR[topic_adapter imu relay] --> IMU_OUT[/imu]

    LID[optional lidar_hw.launch.py] --> SCAN[/scan]
    CAM[sim camera sensors in robot.gz.xacro] --> CAMTOPICS[/camera/color/* + /camera/depth/*]
    JOY[optional joystick launch] --> CMD
```

## 3) Mode Matrix

| Mode | Simulator | ros2_control host | Hardware plugin | Typical use |
|---|---|---|---|---|
| `gz_sim` | Gazebo Sim | Gazebo (`gz_ros2_control`) | simulation system | Classroom simulation and rapid testing |
| `hardware` | none | local ROS node (`ros2_control_node`) | `vmx_system` | Real Titan/VMX robot |
| `mock` | none | local ROS node (`ros2_control_node`) | `sim_system` | Software-only tests without robot |

## 4) Profile-Based Robot Variants

To support multiple robot builds (2WD/4WD/class/training/custom):

- Copy/create a profile in `bringup/config/profiles/<name>/`
- Set geometry and hardware mapping in `robot_profile.yaml`
- Set drive behavior in `robot_profile.yaml` `drive.*` (`wheel_layout`, `controller_name`, `controller_type`)
  - `wheel_layout: omni` uses X-drive wheel mounting in URDF (45 deg wheel yaw at corners)
- Set controller tuning in `robot_controllers.yaml`
- Launch with `robot_profile:=<name>`

No launch/C++ changes are needed for normal configuration differences.

## 5) Key Interfaces

- Command input:
  - diff profiles: `/<drive_controller>/cmd_vel` (`geometry_msgs/msg/TwistStamped`)
  - holonomic profiles (`mecanum` + `omni`): `/<drive_controller>/reference` (`geometry_msgs/msg/TwistStamped`)
- Odometry output:
  - diff profiles: `/<drive_controller>/odom` (`nav_msgs/msg/Odometry`)
  - holonomic profiles (`mecanum` + `omni`): `/<drive_controller>/odometry` (`nav_msgs/msg/Odometry`)
- Compatibility odom alias: `/odom`
- Laser scan: `/scan` (`sensor_msgs/msg/LaserScan`)
- IMU output alias: `/imu` (`sensor_msgs/msg/Imu`)
- Sim camera color image: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- Sim camera depth image: `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
