# ROS 2 Training

The former all-in-one training page has been replaced by a sequential,
simulation-first course. Start at the [course index](COURSE.md).

## Labs

1. [Terminal, workspace, safety, and vocabulary](labs/01_terminal_and_safety.md)
2. [Simulation and the ROS graph](labs/02_simulation_graph.md)
3. [Topics, teleoperation, and `/cmd_vel`](labs/03_topics_teleop.md)
4. [Sensors, TF, RViz, and QoS](labs/04_sensors_tf_qos.md)
5. [Python nodes, parameters, services, and launch](labs/05_python_nodes.md)
6. [`ros2_control`, odometry, and diagnostics](labs/06_control_odometry_diagnostics.md)
7. [SLAM and map saving](labs/07_slam.md)
8. [Nav2 localization and goals](labs/08_navigation.md)
9. [Supervised hardware readiness](labs/09_supervised_hardware.md)

The labs use only the public `/cmd_vel` `geometry_msgs/msg/Twist` motion API and
the stable `/odom`, `/imu`, `/scan`, `/joint_states`, `/tf`, and `/tf_static`
feedback interfaces. Old profile-specific controller commands are intentionally
not part of the course.

Use the [documentation index](README.md) for installation, troubleshooting,
mapping/navigation background, networking, supervised hardware, and instructor
material. Python TODO starters and separate reference solutions for Lab 5 are
under [`examples/python/`](../examples/python/).
