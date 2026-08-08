# Lab 5 Python Examples

These are ordinary `ament_python` packages intended to be copied into the
workspace `src` directory. They are not installed as part of
`studica_vmxpi_ros2`.

- `starters/robot_course_examples`: syntactically valid TODO package for students
- `solutions/robot_course_examples_solution`: complete reference package for instructors

Copy only the starter at the beginning of Lab 5:

```bash
cp -r \
  "$STUDICA_WS/src/studica_vmxpi_ros2/examples/python/starters/robot_course_examples" \
  "$STUDICA_WS/src/robot_course_examples"
```

The exercise subscribes to `/odom`, publishes a harmless text summary, exposes
a read-only Trigger service, and demonstrates parameters and launch. Neither
package publishes `/cmd_vel` or TF.

Instructors can syntax-check every example without running a ROS node:

```bash
python3 -m compileall -q \
  "$STUDICA_WS/src/studica_vmxpi_ros2/examples/python"
```

The reference solution should be released only after the Lab 5 checkpoint.
An instructor can copy and build it independently:

```bash
cp -r \
  "$STUDICA_WS/src/studica_vmxpi_ros2/examples/python/solutions/robot_course_examples_solution" \
  "$STUDICA_WS/src/robot_course_examples_solution"
cd "$STUDICA_WS"
colcon build --symlink-install --packages-select robot_course_examples_solution
source install/setup.bash
ros2 launch robot_course_examples_solution sensor_reporter_solution.launch.py
```

The different package/executable names prevent the solution from silently
replacing a student's work.
