"""Fail-closed command ownership for PC navigation with joystick takeover."""
import math


class OverrideGate:
    def __init__(self, linear=0.20, angular=0.60, turbo_linear=0.20, turbo_angular=0.60):
        self.speeds = (linear, angular, turbo_linear, turbo_angular)
        self.mode = 'WAITING_FOR_NEW_GOAL'
        self.joy_time = None
        self.released = False
        self.manual = (0.0, 0.0)
        self.nav = (0.0, 0.0)
        self.nav_time = None
        self.seen = set()
        self.blocked = set()
        self.after = 0.0
        self.goal = None

    def halt(self, stamp):
        self.mode = 'WAITING_FOR_NEW_GOAL'
        self.after = stamp
        self.goal = None
        self.nav_time = None
        self.manual = (0.0, 0.0)
        self.blocked.update(self.seen)

    def joy(self, now, stamp, buttons, axes):
        valid = (len(buttons) > 4 and len(axes) > 3
                 and buttons[4] in (0, 1)
                 and all(math.isfinite(x) and abs(x) <= 1.0 for x in axes))
        if not valid:
            self.halt(stamp)
            self.released = False
            return True
        self.joy_time = now
        held = buttons[4] == 1
        if not held:
            was_manual = self.mode == 'MANUAL'
            if was_manual:
                self.halt(stamp)
            self.released = True
            self.manual = (0.0, 0.0)
            return was_manual
        if not self.released:
            self.halt(stamp)
            return True
        takeover = self.mode != 'MANUAL'
        if takeover:
            self.halt(stamp)
            self.mode = 'MANUAL'
        turbo = len(buttons) > 5 and buttons[5] == 1
        linear, angular = self.speeds[2:] if turbo else self.speeds[:2]
        self.manual = (axes[1] * linear, axes[3] * angular)
        return takeover

    def goals(self, statuses, stamp):
        # statuses: (globally unique action/id, ROS acceptance timestamp, status)
        active = {key for key, _, state in statuses if state in (1, 2, 3)}
        old_active = active & self.blocked
        for key, started, state in statuses:
            new = key not in self.seen
            self.seen.add(key)
            if self.mode == 'MANUAL':
                self.blocked.add(key)
            elif (self.mode == 'WAITING_FOR_NEW_GOAL' and new and state in (1, 2)
                  and started > self.after and not old_active):
                self.mode = 'NAVIGATION'
                self.goal = key
                self.nav_time = None
        if self.mode == 'NAVIGATION' and self.goal not in active:
            self.halt(stamp)

    def navigation(self, now, x, yaw):
        if all(math.isfinite(v) for v in (x, yaw)) and abs(x) <= 0.30 and abs(yaw) <= 0.90:
            self.nav = (x, yaw)
            self.nav_time = now
        else:
            self.nav_time = None

    def output(self, now, stamp):
        if self.mode == 'MANUAL':
            if self.joy_time is None or now - self.joy_time > 0.25:
                self.halt(stamp)
                self.released = False
                return (0.0, 0.0)
            return self.manual
        if self.mode == 'NAVIGATION' and self.nav_time is not None:
            if now - self.nav_time <= 0.25:
                return self.nav
            self.halt(stamp)
        return (0.0, 0.0)
