# VMXPi SSH connectivity audit: 2026-08-31

This was a read-only investigation of intermittent TCP port 22 timeouts after
the first inactive release installation. No NetworkManager profile, firewall,
SSH configuration, systemd unit, ROS process, or motor state was changed.

## Observed path

- VMXPi: `192.168.1.173` on `wlan0`, connected to the 5 GHz `Home` network.
- Audit client: `192.168.2.118`, routed through `192.168.1.1`.
- Signal: `-41 dBm`, link quality `69/70`, nominal rate `200 Mb/s`.
- Twelve return pings had zero loss but ranged from 4.4 to 111 ms, with a
  56.8 ms average.
- The persistent SSH connection reported TCP retransmission and reordering,
  although it remained usable.

## Eliminated VMXPi causes

- `ssh.service` was active, had not restarted, and listened on IPv4 and IPv6.
- sshd reported `0 of 10-100` startup slots in use. Effective limits were
  `MaxStartups 10:30:100`, `MaxSessions 10`, and no per-source limit.
- UFW's systemd unit was active, but `ufw status` reported `inactive`; there
  was therefore no active UFW rate-limit or drop rule causing the timeouts.
- No fail2ban or sshguard service was active.
- Kernel journals contained no matching undervoltage, throttling, OOM, MMC,
  thermal, or Wi-Fi disconnect event during the timeout window.
- `vcgencmd get_throttled` returned `0x0`.
- NetworkManager recorded no event during the timeout window.

The sshd journal accepted repeated public-key sessions immediately before and
after the failed client connections. One attempt reached sshd but was closed by
the client during key exchange; the timeout attempts left no sshd record. This
localizes the failure before sshd, in the Wi-Fi/access-point/router path for new
TCP connections.

## Findings

Wi-Fi power management was active at the driver and the `Home` connection used
NetworkManager's ambiguous `default` setting. This is a concrete latency and
reliability risk, but the available evidence does not prove it was the sole
cause of every timeout. The routed cross-subnet path and high latency jitter
also remain contributors.

The production audit additionally confirmed existing security blockers:

- UFW policy disabled even though `ufw.service` is active;
- SSH password authentication enabled;
- root login set to `prohibit-password`, not the product requirement `no`;
- X11 forwarding enabled;
- generic shared `vmx` identity and other development-image findings.

These findings were already expected on the reference POC image and are not an
authorization to harden it before a bootable recovery clone exists.

## Production decision

The production NetworkManager profile must explicitly set Wi-Fi power saving
to `disable`; inheriting `default` is not accepted. Runtime audit profile v1 now
enforces that state. Update and support operations should use Ethernet when
available, otherwise a same-subnet Wi-Fi management path and one persistent SSH
connection with keepalives. Qualification must still test reconnects, router
loss, access-point loss, cross-subnet routing, and update interruption.

After the audit, the installed development release was rechecked across a
VMXPi reboot. It remained intact, all payload checksums passed,
`/opt/studica/current` remained absent, and no ROS/control process was running.
