"""PID building blocks for the cascaded baseline controller.

Gain philosophy (viva): each loop is a simple proportional (or PI) law whose
closed-loop bandwidth is set by pole placement from the physical parameters
(inertia for the rate loop), with each outer loop 2x slower than the loop it
commands. No hand-twiddled magic numbers.
"""
from __future__ import annotations

import numpy as np


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=None, i_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit if i_limit is not None else out_limit
        self.i = 0.0
        self.prev_e = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def __call__(self, e, dt, e_dot=None):
        if self.ki:
            self.i += e * dt
            if self.i_limit is not None:
                self.i = np.clip(self.i, -abs(self.i_limit / max(self.ki, 1e-9)),
                                 abs(self.i_limit / max(self.ki, 1e-9)))
        d = 0.0
        if self.kd:
            if e_dot is not None:
                d = e_dot
            elif self.prev_e is not None:
                d = (e - self.prev_e) / dt
            self.prev_e = e
        out = self.kp * e + self.ki * self.i + self.kd * d
        if self.out_limit is not None:
            out = np.clip(out, -self.out_limit, self.out_limit)
        return out


def quat_to_euler(q):
    """w,x,y,z quaternion (world<-body) to roll, pitch, yaw in FLU/ENU with
    PITCH POSITIVE = NOSE UP. In FLU a positive rotation about +y drops the
    nose, so the raw ZYX pitch is negated here (and in euler_to_quat), which
    keeps every controller gain in the intuitive nose-up-positive convention.
    Roll positive = roll right (left wing up); yaw positive = nose left (CCW).
    """
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = np.clip(2 * (w * y - z * x), -1, 1)
    pitch = -np.arcsin(sp)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    """Inverse of quat_to_euler: pitch is nose-up-positive (FLU)."""
    pitch = -pitch
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def attitude_rate_cmd(q_now, q_des, kp_att):
    """Quaternion attitude error -> body rate command (shortest rotation)."""
    q_err = quat_mul(quat_conj(q_now), q_des)
    if q_err[0] < 0:
        q_err = -q_err
    return 2.0 * kp_att * q_err[1:4]
