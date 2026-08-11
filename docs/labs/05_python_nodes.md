# Lab 5 — Python Nodes, Parameters, Services, and Launch

## Learning goals

- Create and build an `ament_python` package from a TODO starter.
- Subscribe to `/odom` and publish a harmless text summary.
- Configure a node with a parameter, provide a read-only service, and launch it.

## Prerequisites

- Labs 1–4 complete.
- Basic Python variables, functions, and classes.
- A text editor; never edit generated workspace directories.

The example deliberately does not publish motion. It observes stable robot data
and publishes `/apps/odom_summary` as `std_msgs/msg/String`.

## Terminals

### Terminal 1 — launch simulation

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

### Terminal 2 — copy and complete the starter

```bash
export STUDICA_WS="$HOME/studica_ws"
STARTER="$STUDICA_WS/src/studica_vmxpi_ros2/examples/python/starters/robot_course_examples"
DESTINATION="$STUDICA_WS/src/robot_course_examples"
if [ -e "$DESTINATION" ]; then
  echo "Using the existing application package; it was not overwritten"
else
  cp -r "$STARTER" "$DESTINATION"
fi
cd "$DESTINATION"
grep -R -n TODO .
```

Complete the TODOs in this order:

1. create the `/apps/odom_summary` publisher;
2. subscribe to `/odom` with sensor-data QoS and save its latest `x` and `y`;
3. declare/read the `robot_name` parameter;
4. create `/apps/report_now` using `std_srvs/srv/Trigger`;
5. publish from the timer and service callback;
6. return a launch description containing the node and parameters.

The starter must stay syntactically valid after every edit. The separate
reference package is under `examples/python/solutions/`; use it only after the
instructor releases it.

### Terminal 2 — build the application package

```bash
cd "$STUDICA_WS"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_course_examples
source install/setup.bash
```

### Terminal 3 — launch the completed node

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch robot_course_examples sensor_reporter.launch.py
```

### Terminal 4 — inspect all four ROS concepts

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /apps/odom_summary --once
ros2 topic info /odom --verbose
ros2 param get /sensor_reporter robot_name
ros2 param set /sensor_reporter robot_name lab_five_robot
ros2 service call /apps/report_now std_srvs/srv/Trigger '{}'
```

## Expected output

- The package builds without Python syntax or missing-dependency errors.
- The node logs that it is waiting for odometry, then publishes summaries such
  as `class_4wd: x=0.000 m, y=0.000 m`.
- Verbose `/odom` information lists the application node as a subscriber.
- The parameter changes to `lab_five_robot`, and later summaries use that name.
- The Trigger response has `success: true` and returns the latest summary.

## Checkpoint

Show the working topic, parameter, and service. In the source, identify the
exact lines that create the publisher, subscription, service, timer, parameter,
and launch `Node`. Explain why the service callback returns a response while a
topic callback does not.

## Cleanup

1. Stop Terminal 3 with `Ctrl+C`.
2. Confirm `/apps/odom_summary` disappears from `ros2 topic list`.
3. Stop Terminal 1 and wait for Gazebo to close.
4. Keep the source package; do not submit `build`, `install`, or `log` copies.

## Challenge

Add a best-effort `/imu` subscription and include angular velocity around Z in
the text report. Add the dependency to `package.xml`, rebuild, and show the
field while the simulated robot turns. Do not change `/cmd_vel` or publish TF.
