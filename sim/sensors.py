"""Sensor models on top of MuJoCo's ideal sensor readings.

IMU: additive Gaussian noise + slowly-drifting bias (random walk).
Baro: altitude + noise. Pitot: airspeed along body x with first-order lag.
GPS: position/velocity with noise, held at 10 Hz (zero-order hold).
Rangefinders: MJCF rangefinder sensors + noise, -1 when out of range.
"""
from __future__ import annotations

import numpy as np
import mujoco


class SensorSuite:
    def __init__(self, model, p: dict, seed=0):
        s = p["sensors"]
        self.p = s
        self.rng = np.random.default_rng(seed)
        self.model = model
        self._adr = {}
        for name in ("imu_acc", "imu_gyro", "imu_vel", "att_quat",
                     "range_fwd", "range_down"):
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            adr = model.sensor_adr[sid]
            dim = model.sensor_dim[sid]
            self._adr[name] = (adr, adr + dim)
        self.reset(seed)

    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        s = self.p
        self.acc_bias = self.rng.normal(0, s["imu_accel_bias"], 3)
        self.gyro_bias = self.rng.normal(0, s["imu_gyro_bias"], 3)
        self.pitot_state = 0.0
        self.gps_pos = np.zeros(3)
        self.gps_vel = np.zeros(3)
        self.gps_countdown = 0.0

    def _raw(self, data, name):
        a, b = self._adr[name]
        return data.sensordata[a:b].copy()

    def read(self, data, true_airspeed_x: float, dt: float) -> dict:
        s = self.p
        rng = self.rng
        # IMU with bias random-walk
        self.acc_bias += rng.normal(0, s["imu_accel_bias"] * 0.01, 3)
        self.gyro_bias += rng.normal(0, s["imu_gyro_bias"] * 0.01, 3)
        acc = self._raw(data, "imu_acc") + self.acc_bias + rng.normal(0, s["imu_accel_noise"], 3)
        gyro = self._raw(data, "imu_gyro") + self.gyro_bias + rng.normal(0, s["imu_gyro_noise"], 3)
        quat = self._raw(data, "att_quat")   # ideal attitude (estimator assumed)

        baro_alt = data.qpos[2] + rng.normal(0, s["baro_noise"])

        # pitot: first-order lag ~0.1 s + noise
        self.pitot_state += (true_airspeed_x - self.pitot_state) * min(dt / 0.1, 1.0)
        pitot = max(self.pitot_state + rng.normal(0, s["pitot_noise"]), 0.0)

        # GPS at 10 Hz ZOH
        self.gps_countdown -= dt
        if self.gps_countdown <= 0.0:
            self.gps_countdown = 0.1
            self.gps_pos = data.qpos[0:3] + rng.normal(0, s["gps_pos_noise"], 3)
            self.gps_vel = data.qvel[0:3] + rng.normal(0, s["gps_vel_noise"], 3)

        rngf = np.array([self._raw(data, "range_fwd")[0],
                         self._raw(data, "range_down")[0]])
        ok = (rngf >= 0) & (rngf <= s["range_max"])
        rngf = np.where(ok, rngf + rng.normal(0, s["range_noise"], 2), -1.0)

        return dict(acc=acc, gyro=gyro, quat=quat, baro_alt=baro_alt,
                    pitot=pitot, gps_pos=self.gps_pos.copy(),
                    gps_vel=self.gps_vel.copy(), range=rngf)
