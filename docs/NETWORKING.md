# Networking and Remote Visualization

Use this guide with an instructor. Foxglove Bridge is the recommended remote
dashboard because it exposes a small, read-only WebSocket surface. Native ROS 2
DDS is needed only when a supervised laptop must publish `/cmd_vel` or run other
ROS nodes as part of the same graph.

Never expose the robot, DDS ports, or Foxglove port to the internet.

## Choose one robot network

Connect the robot and laptop to the same trusted LAN. Decide whether the lesson
uses Wi-Fi or Ethernet; avoid switching routes during a run.

On each computer:

```bash
ip -br address
ip route
```

Record placeholders for the lesson:

```text
Robot address: <ROBOT_IP>
Laptop address: <LAPTOP_IP>
Robot interface: <ROBOT_INTERFACE>
Laptop interface: <LAPTOP_INTERFACE>
```

Confirm the selected route:

```bash
ip route get <PEER_IP>
ping -c 3 <PEER_IP>
```

The route output should name the intended Wi-Fi or Ethernet interface.

## Recommended: read-only Foxglove

The beginner hardware launch binds Foxglove to loopback by default. Bind it to
the robot's exact trusted-LAN address only for a supervised dashboard session:

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  foxglove_address:=<ROBOT_IP> foxglove_port:=8765
```

The real launch normally runs through the root command in
[Supervised hardware](HARDWARE.md); the arguments above are appended there.

On the laptop, add a Foxglove WebSocket connection:

```text
ws://<ROBOT_IP>:8765
```

Import `foxglove/class_4wd_robot_health.layout.json`. The bridge whitelists
telemetry, diagnostics, TF, logs, compressed color, and compressed depth. Its
only client capability is the connection graph: remote publishing, services,
and parameter changes are disabled.

Confirm the listener on the robot:

```bash
ss -ltn | grep 8765
```

If Foxglove reports no topics, check the bridge process and URL. If the robot log
reports `handshake failed`, ensure the connection type is **Foxglove WebSocket**,
not ROS 2, HTTP, or a browser page.

## Native ROS 2 DDS discovery

Use the same values in the current terminal on both computers:

```bash
export ROS_DOMAIN_ID=<CLASS_DOMAIN_ID>
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
```

Use an instructor-assigned domain from `0` through `232`; different classroom
teams should use different IDs. Source the local workspace too if that computer
needs Studica message definitions.

Check that both computers use the same middleware:

```bash
printenv RMW_IMPLEMENTATION
```

An empty value means both normally use the Humble default. If one computer sets
an implementation explicitly, install and set the same implementation on the
other. Keep these exports in the lesson terminal, not `.bashrc`.

Restart discovery after changing DDS variables:

```bash
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

## Test multicast before ROS topics

On the receiving computer:

```bash
ros2 multicast receive
```

While it waits, run on the other computer:

```bash
ros2 multicast send
```

Repeat with the computers reversed. Success in only one direction points to a
firewall, access-point isolation, VPN, or route problem rather than a ROS node.

Disable VPN software temporarily for the supervised test. Some school Wi-Fi
networks intentionally block client-to-client multicast; use a dedicated robot
access point or Ethernet instead of weakening the school network.

## Prefer an interface explicitly with Cyclone DDS

Only use this section when the computer has multiple active networks and the
default route is wrong. Install Cyclone DDS on both computers:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

In the current terminal, replace the interface placeholder:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="<INTERFACE>" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
ros2 daemon stop
```

Set a matching interface on the peer. Do not copy a sample interface name: Wi-Fi
and Ethernet names differ by computer. Remove the variables by closing the
terminal.

## Supervised remote driving

Foxglove is observational and cannot drive. For a ROS-enabled laptop, first pass
the two-way multicast test and confirm these robot topics:

```bash
ros2 topic type /cmd_vel
ros2 topic hz /odom
ros2 run studica_robot_monitor robot_check --mode hardware
```

With the instructor holding the emergency stop and the area clear:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Use the lowest speed. Stop teleop before changing Wi-Fi, unplugging Ethernet, or
closing the laptop. The robot command timeout is a backup, not a substitute for
the physical emergency stop.

## Firewall policy

Prefer a trusted isolated LAN. Allow only what the lesson needs:

- Foxglove: TCP port `8765` from `<LAPTOP_IP>` to `<ROBOT_IP>`;
- native DDS: bidirectional UDP discovery/data for the assigned ROS domain;
- SSH, if used: TCP port `22` from the instructor laptop only.

DDS port calculation varies with domain and participant IDs. Have the network
administrator define the classroom rule; do not disable the firewall globally.

## End the session

1. stop keyboard/gamepad teleop;
2. stop hardware bringup and wait for zero targets;
3. close Foxglove;
4. remove temporary firewall rules, if any;
5. close terminals containing DDS interface overrides.
