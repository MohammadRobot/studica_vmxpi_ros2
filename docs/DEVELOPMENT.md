# Application Development and Deployment

Use the PC as the primary development machine and the VMXPi as the deployment
target. This keeps editing, Git, simulation, RViz, and most tests fast while
ensuring hardware code is built for the VMXPi's arm64 processor.

## Workspace layout

Developers normally edit one application package:

```text
~/studica_ws/src/
├── studica_vmxpi_ros2/   platform, simulation, hardware, and safety
└── studica_robot_apps/   project-specific nodes, launch, config, and tests
```

Other packages in `src` are platform or vendor dependencies. They do not need
to be understood or edited to build an application.

Keep these responsibilities separate:

| Area | Changes belong here |
|---|---|
| `studica_vmxpi_ros2` | reusable robot model, bringup, public interfaces, safety |
| `studica_robot_apps` | behaviors, missions, perception, application configuration |

The package boundary protects the platform and makes application builds fast.
It does not require another workspace.

## Daily development loop

### 1. Edit on the PC

Open this directory in the local editor:

```text
~/studica_ws/src/studica_robot_apps
```

Keep it in Git. Never edit generated files below `build`, `install`, or `log`.

### 2. Build only the application

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
cd "$STUDICA_WS"
colcon build --symlink-install --packages-select studica_robot_apps
source "$STUDICA_WS/install/setup.bash"
```

The first build establishes this package's symlink-install mode. Keep using the
same mode for later builds. Python source edits are then visible immediately;
rebuild after changing package metadata, installed launch files, or dependencies.

### 3. Test with simulation

Terminal 1:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Terminal 2:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_robot_apps robot_observer.launch.py use_sim_time:=true
```

The included observer is read-only. Confirm it receives simulation odometry:

```bash
ros2 topic echo /apps/odom_summary --once
ros2 service call /apps/report_now std_srvs/srv/Trigger '{}'
```

Run package tests before deployment:

```bash
cd "$STUDICA_WS"
colcon test --packages-select studica_robot_apps
colcon test-result --test-result-base build/studica_robot_apps --verbose
```

### 4. Deploy source to the VMXPi

From the PC:

```bash
cd "$STUDICA_WS/src/studica_robot_apps"
./scripts/deploy_to_vmxpi.sh --host vmx@192.168.1.63
```

The helper uses `rsync` over SSH, copies only `studica_robot_apps`, and runs this
build on the VMXPi:

```bash
colcon build --symlink-install --packages-select studica_robot_apps
```

It never copies `build`, `install`, or `log`. PC binaries are amd64 and cannot
run on the arm64 VMXPi. The script does not contain or save an SSH password;
use an SSH key for repeated deployment or enter the password interactively.
Extra remote files are preserved by default. Add `--delete` only after making
the PC copy authoritative and reviewing any VMXPi-only edits.

To use another robot address:

```bash
./scripts/deploy_to_vmxpi.sh --host vmx@<VMXPI_IP>
```

### 5. Run on the VMXPi

Start the supervised robot bringup first. In a separate SSH session:

```bash
ssh vmx@<VMXPI_IP>
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_vmxpi_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_robot_apps robot_observer.launch.py use_sim_time:=false
```

Source the generated VMXPi Ethernet environment when using Ethernet. The PC and
VMXPi must use matching Cyclone DDS domain and peer profiles; see
[Networking and Cyclone DDS](NETWORKING.md).

## When to use VS Code Remote SSH

Use Remote SSH for logs, hardware-only inspection, or a small emergency
diagnostic edit. Do normal coding on the PC because it provides simulation,
faster tools, and the primary Git copy. If a remote edit is unavoidable, commit
or copy it back immediately so the PC and VMXPi do not diverge.

## Application rules

- Use the stable `/cmd_vel`, `/odom`, `/imu`, `/scan`, and TF interfaces.
- Put tunable values in YAML or ROS parameters, not hard-coded constants.
- Keep pure calculations separate from ROS callbacks so unit tests stay fast.
- Launch no automatic physical motion; require an explicit operator action.
- Allow only one `/cmd_vel` publisher unless a command multiplexer is present.
- Disable joystick teleoperation with `use_joystick:=false` while an application
  owns `/cmd_vel`.
- Test in simulation, then at low speed with the hardware safety procedure.

The same application package runs in simulation and hardware. Only
`use_sim_time` and the selected environment change.
