# Networking and Cyclone DDS

Use this guide with an instructor. Simulation stays local to the PC. Real-robot
sessions use a deliberate peer profile between one PC and one VMXPi over either
Wi-Fi or Ethernet.

Never expose DDS, SSH, or Foxglove ports to the internet. Stop robot motion
before changing a route, address, firewall rule, or DDS profile.

## The three supported modes

| Mode | Computers | Cyclone DDS interface | Generated environment |
|---|---|---|---|
| Simulation | PC only | loopback (`lo`) | `~/.ros/studica_sim.env` |
| Robot over Wi-Fi | PC and VMXPi | each machine's Wi-Fi address | one peer environment per machine |
| Robot over Ethernet | PC and VMXPi | each machine's Ethernet address | one peer environment per machine |

Do not reuse a robot profile for simulation. A profile containing an address
that is no longer assigned makes Cyclone DDS reject every ROS node.

The repository provides:

- `bringup/config/network/cyclonedds_sim.xml`, the loopback-only template;
- `bringup/config/network/cyclonedds_peer.xml.in`, the two-machine template;
- `scripts/configure_cyclonedds.py`, the validated profile generator.

The setup script installs `rmw_cyclonedds_cpp`. Confirm it on both machines:

```bash
ros2 pkg prefix rmw_cyclonedds_cpp
```

## Inspect the selected network first

Connect both machines to one trusted network. Use Wi-Fi on both or Ethernet on
both; avoid changing routes during robot operation.

On the PC and VMXPi:

```bash
ip -br address
ip route
```

Record:

```text
PC address:       <PC_IP>
PC interface:     <PC_INTERFACE>
VMXPi address:    <VMXPI_IP>
VMXPi interface:  <VMXPI_INTERFACE>
ROS domain:       <DOMAIN_ID>
```

Verify both directions:

```bash
ip route get <PEER_IP>
ping -c 3 <PEER_IP>
```

The route must name the intended interface and source address. Fix IP routing
before changing ROS settings.

## Generate the simulation profile

Run this once on the PC:

```bash
export STUDICA_WS="$HOME/studica_ws"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py sim --domain-id 1
```

Activate it in every simulator, joystick, and simulation-inspection terminal:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"
```

After switching an existing terminal, refresh its ROS CLI daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

The generated profile uses only `lo`, disables multicast, and raises Cyclone's
automatic participant limit to `50` for a full Gazebo launch.
It also exports `GZ_VERSION=harmonic`, which makes later `colcon build`
commands select this project's Gazebo Harmonic (`gz-sim8`) compatibility path.

## Generate PC and VMXPi peer profiles

Run the generator separately on each machine. It validates that the local
address is currently assigned to the named interface, writes an XML profile,
and writes a small sourceable `.env` file. It never edits `.bashrc`, changes the
firewall, starts ROS, or starts robot hardware.

The general PC command is:

```bash
cd "$HOME/studica_ws/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py peer \
  --name pc_<LINK> \
  --interface <PC_INTERFACE> \
  --local-address <PC_IP> \
  --peer-address <VMXPI_IP> \
  --domain-id <DOMAIN_ID>
```

The matching VMXPi command is:

```bash
cd "$HOME/studica_ws/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py peer \
  --name vmxpi_<LINK> \
  --interface <VMXPI_INTERFACE> \
  --local-address <VMXPI_IP> \
  --peer-address <PC_IP> \
  --domain-id <DOMAIN_ID>
```

Use the same domain on both machines. Different classroom teams should use
different domain IDs from `0` through `232`.

If a named profile already exists with another address, the tool refuses to
overwrite it. Review the new route, then add `--force` deliberately. This is
normally required after DHCP changes an address.

## Wi-Fi example

This tested example uses:

```text
PC:     wlp0s20f3, 192.168.1.118
VMXPi:  wlan0,     192.168.1.63
Domain: 1
```

On the PC:

```bash
cd "$HOME/studica_ws/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py peer \
  --name pc_wifi \
  --interface wlp0s20f3 \
  --local-address 192.168.1.118 \
  --peer-address 192.168.1.63 \
  --domain-id 1
source "$HOME/.ros/studica_pc_wifi.env"
```

On the VMXPi:

```bash
cd "$HOME/studica_ws/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py peer \
  --name vmxpi_wifi \
  --interface wlan0 \
  --local-address 192.168.1.63 \
  --peer-address 192.168.1.118 \
  --domain-id 1
source "$HOME/.ros/studica_vmxpi_wifi.env"
```

These addresses are examples, not defaults. Re-read `ip -br address` and use
the addresses assigned during the current lesson.

## Direct Ethernet example

Configure a static, private subnet with no internet gateway. A common classroom
pair is:

```text
PC:     172.22.11.10/24
VMXPi:  172.22.11.2/24
Domain: 1
```

Assign those addresses using Ubuntu Network settings or the classroom's
NetworkManager connection profiles. Confirm the link with `ip -br address`,
`ip route get`, and `ping` before generating DDS files.

Example PC profile:

```bash
cd "$HOME/studica_ws/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py peer \
  --name pc_ethernet \
  --interface enp0s31f6 \
  --local-address 172.22.11.10 \
  --peer-address 172.22.11.2 \
  --domain-id 1
source "$HOME/.ros/studica_pc_ethernet.env"
```

Example VMXPi profile:

```bash
cd "$HOME/studica_ws/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py peer \
  --name vmxpi_ethernet \
  --interface eth0 \
  --local-address 172.22.11.2 \
  --peer-address 172.22.11.10 \
  --domain-id 1
source "$HOME/.ros/studica_vmxpi_ethernet.env"
```

Interface names vary. Never copy `enp0s31f6` or `eth0` without checking the
actual machine.

## Firewall rules

The generated peer profiles use explicit unicast peers and participant indices
`0` through `50`; they do not require multicast. The helper prints the exact
incoming UDP range for the selected domain and an address-scoped UFW command.

For domain `1`, the range is `7660:7761`. The Wi-Fi PC example uses:

```bash
sudo ufw allow in on wlp0s20f3 \
  proto udp \
  from 192.168.1.63 \
  to 192.168.1.118 \
  port 7660:7761 \
  comment 'Studica ROS 2 DDS domain 1'
```

Apply the corresponding printed rule on the VMXPi only if its firewall is
active. Confirm rules without disabling the firewall:

```bash
sudo ufw status numbered
```

For a different domain `D`, the helper calculates the destination range as:

```text
first = 7410 + 250 × D
last  = 7511 + 250 × D
```

Allow only the exact peer, destination address, interface, protocol, and port
range. Do not use `ufw disable`, expose all UDP ports, or permit the whole LAN.
SSH, when required, should allow TCP `22` from the instructor PC only.

## Activate a robot session

On the PC, source its selected peer environment before joystick or inspection
nodes:

```bash
source "$HOME/.ros/studica_pc_wifi.env"  # or studica_pc_ethernet.env
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"
```

On the VMXPi:

```bash
source "$HOME/.ros/studica_vmxpi_wifi.env"  # or studica_vmxpi_ethernet.env
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"
```

Check both terminals:

```bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
```

Do not place a link-specific `CYCLONEDDS_URI` in `.bashrc` or `.profile`.
Terminal-scoped environments prevent an absent robot interface from breaking
simulation. The supervised root launch preserves these four selected variables.

Application nodes may run on the PC during a short network test, but the normal
deployment path copies `studica_robot_apps` source to the VMXPi and builds it
there. This avoids making physical operation depend on the PC connection. See
[Application development and deployment](DEVELOPMENT.md).

## Test discovery without robot hardware

Restart the CLI daemon on both machines after changing profiles:

```bash
ros2 daemon stop
ros2 daemon start
```

On the VMXPi, start a harmless text publisher:

```bash
ros2 run demo_nodes_cpp talker
```

On the PC:

```bash
ros2 topic list
ros2 topic echo /chatter std_msgs/msg/String --once
```

Expected: `/chatter` appears and one `Hello World` message arrives. ROS 2 Humble
does not accept a global `ros2 --no-daemon` option. Stop the talker before
hardware bringup.

If discovery fails, check in this order:

```bash
ip route get <PEER_IP>
ping -c 3 <PEER_IP>
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI
sudo ufw status numbered
```

Also confirm that the local address inside the selected XML still appears in
`ip -br address`. If DHCP changed it, regenerate both peer profiles with the
new addresses and `--force`, update narrow firewall rules, and restart both ROS
daemons.

## Keep simulation and robot DDS profiles separate

Switch back to simulation by opening a new terminal and sourcing only:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"
ros2 daemon stop
ros2 daemon start
```

Errors containing `does not match an available interface`, `failed to create
domain`, or `rcl node's rmw handle is invalid` normally mean the selected XML
contains a stale address. They do not identify a failure in the ROS node named
later in the stack trace.

## Recommended read-only Foxglove access

Foxglove is preferable when the PC only needs visualization. The beginner
hardware launch binds its bridge to loopback by default. Bind it to the exact
trusted-LAN VMXPi address for a supervised session:

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  foxglove_address:=<VMXPI_IP> foxglove_port:=8765
```

The real launch normally runs through the root command in
[Supervised hardware](HARDWARE.md); append those arguments there. On the PC,
connect Foxglove to:

```text
ws://<VMXPI_IP>:8765
```

The project bridge allows telemetry, diagnostics, TF, logs, and compressed
camera data. Remote publishing, services, and parameter changes are disabled.
If the VMXPi firewall is active, allow TCP `8765` only from `<PC_IP>`.

## Supervised remote driving

Native DDS is required for the current developer-mode command paths. An
application publishes `/cmd_vel`; PC-side joystick mode publishes both
`/cmd_vel/joy` and `/joy` and requires `robot.launch.py
control_source:=joystick`. Before starting either path, confirm:

```bash
ros2 topic list -t | grep -E '/cmd_vel|/joy'
ros2 topic hz /odom
ros2 run studica_robot_monitor robot_check --mode hardware
```

Follow [Supervised hardware](HARDWARE.md) and [Joystick teleoperation](JOYSTICK.md).
Use one motion publisher, keep the emergency stop reachable, and stop teleop
before changing networks or closing the laptop.

## End the session

1. Center the joystick, release the deadman button, and stop teleop.
2. Verify zero target and measured wheel velocities.
3. Disable motor power and stop hardware bringup.
4. Stop temporary talkers and close Foxglove.
5. Close robot-profile terminals; use `studica_sim.env` for later simulation.
6. Remove temporary firewall rules only when the corresponding link is retired.
