"""Servo command shaping: rate limit + first-order lag + optional backlash.

The MJCF position actuators already behave like stiff position servos; this
wrapper shapes the COMMAND the way a real hobby servo does:
  1. slew-rate limit (120 deg/s class)
  2. first-order lag (electronics + gear train)
  3. backlash deadband (gear slop, ~0.5 deg): the output only moves once the
     command has crossed the dead zone.
"""
from __future__ import annotations

import numpy as np


class Servo:
    def __init__(self, rate_limit_rad_s: float, tau: float,
                 backlash_rad: float = 0.0, x0: float = 0.0):
        self.rate = rate_limit_rad_s
        self.tau = tau
        self.backlash = backlash_rad
        self.y = x0        # lag state
        self.out = x0      # post-backlash output

    def reset(self, x0: float = 0.0):
        self.y = x0
        self.out = x0

    def step(self, cmd: float, dt: float) -> float:
        # rate limit toward command
        dy_max = self.rate * dt
        target = self.y + np.clip(cmd - self.y, -dy_max, dy_max)
        # first-order lag
        self.y += (target - self.y) * min(dt / self.tau, 1.0)
        # backlash: output follows only outside the dead zone
        if self.backlash > 0.0:
            half = self.backlash / 2
            if self.y - self.out > half:
                self.out = self.y - half
            elif self.out - self.y > half:
                self.out = self.y + half
        else:
            self.out = self.y
        return self.out


class ServoBank:
    """9 servos matching the MJCF actuator order:
    [tilt_L, tilt_R, tilt_T, ail_L, ail_R, elev_L, elev_R, rud_L, rud_R]"""

    def __init__(self, p: dict):
        t = p["tilt"]
        s = p["surfaces"]
        rate_t = np.deg2rad(t["servo_rate_limit"])
        rate_s = np.deg2rad(s["servo_rate_limit"])
        lag_t = t["servo_lag"]
        back = np.deg2rad(t["backlash_deg"])
        self.servos = (
            [Servo(rate_t, lag_t, back) for _ in range(3)]
            + [Servo(rate_s, 0.03, back) for _ in range(6)]
        )

    def reset(self, x0=None):
        x0 = np.zeros(9) if x0 is None else x0
        for s, v in zip(self.servos, x0):
            s.reset(v)

    def step(self, cmds, dt: float) -> np.ndarray:
        return np.array([s.step(c, dt) for s, c in zip(self.servos, cmds)])
