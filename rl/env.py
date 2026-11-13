"""Gymnasium environment around sim.vehicle.Vehicle.

Tasks:
  hover    - hold a randomized goal point near the spawn (S1)
  waypoint - visit a sequence of randomized waypoints (S2)
  obstacle - reach a goal ~35 m ahead through a procedural field (S5)

Action modes (Phase 4 experiment, D-15):
  A - direct 9-dim actuator commands in [-1, 1]
  B - residual on the baseline hover controller (scaled, clipped)

Observations (35,): quat(4), body rates/5(3), body vel/20(3), airspeed/20(1),
altitude/20(1), tilt/90deg(3), last action(9), goal vector in yaw frame/20(3),
8 horizontal range rays /10 (1.0 = clear).

RL runs at 50 Hz = 2 vehicle control steps per env step.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402
from sim.vehicle import Vehicle  # noqa: E402
from sim.wind import Wind  # noqa: E402
from control.baseline import BaselineController  # noqa: E402
from control.pid import quat_to_euler  # noqa: E402
from rl.obstacles import obstacles_xml, inject  # noqa: E402
from rl.randomize import DomainRandomizer  # noqa: E402

N_RAYS = 8
RAY_MAX = 10.0


class VTOLEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task="hover", action_mode="B", dr=False,
                 difficulty=0.3, seed=0, episode_s=None):
        super().__init__()
        self.task = task
        self.action_mode = action_mode
        self.rng = np.random.default_rng(seed)
        self.dr = DomainRandomizer(enabled=dr)
        self.difficulty = difficulty
        self.p = load_params()

        self._base_xml = (ROOT / "model" / "vtol.xml").read_text()
        self._make_vehicle()

        self.ctrl_base = BaselineController(self.p)
        self.rl_dt = 2 * self.veh.dt_ctrl              # 50 Hz
        ep = episode_s or {"hover": 15.0, "waypoint": 30.0,
                           "obstacle": 30.0}[task]
        self.max_steps = int(ep / self.rl_dt)

        self.observation_space = spaces.Box(-np.inf, np.inf, (35,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (9,), np.float32)
        self.residual_scale = np.array([0.25] * 3 + [0.25] * 3 + [0.5] * 3)

        self._ray_group = np.zeros(6, dtype=np.uint8)
        self._ray_group[2] = 1                          # obstacles only
        self.last_action = np.zeros(9)
        self.goal = np.zeros(3)
        self.steps = 0

    # ------------------------------------------------------------- #
    def _make_vehicle(self):
        if self.task == "obstacle":
            block = obstacles_xml(self.difficulty, self.rng)
            xml = inject(self._base_xml, block)
            model = mujoco.MjModel.from_xml_string(xml)
        else:
            model = mujoco.MjModel.from_xml_string(self._base_xml)
        self.veh = Vehicle(model=model, params=self.p,
                           wind=Wind(seed=int(self.rng.integers(1 << 30))))
        m = model
        self._obs_geoms = {i for i in range(m.ngeom)
                           if m.geom_group[i] == 2}
        self._drone_geoms = {i for i in range(m.ngeom)
                             if m.geom_bodyid[i] >= 1}
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self._ground_geom = gid

    # ------------------------------------------------------------- #
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if self.task == "obstacle":
            self._make_vehicle()                        # new field
        dr_info = self.dr.apply(self.veh, self.rng)

        spawn = np.array([0.0, 0.0, 6.0])
        self.veh.reset(seed=int(self.rng.integers(1 << 30)),
                       pos=spawn, hover_init=True)

        if self.task == "hover":
            self.goal = spawn + self.rng.uniform([-2, -2, -1], [2, 2, 2])
            self.waypoints = [self.goal]
        elif self.task == "waypoint":
            self.waypoints = [spawn + self.rng.uniform([-12, -12, -2],
                                                       [12, 12, 4])
                              for _ in range(3)]
            self.goal = self.waypoints[0]
        else:
            self.goal = np.array([35.0, self.rng.uniform(-3, 3),
                                  self.rng.uniform(4, 8)])
            self.waypoints = [self.goal]
        self.wp_index = 0

        self.ctrl_base.reset()
        self.ctrl_base.mode = "HOVER"                   # position hold only
        self.last_action = np.zeros(9)
        self.prev_dist = np.linalg.norm(self.goal - spawn)
        self.steps = 0
        self._collided = False
        s = self.veh.state()
        return self._obs(s), {"dr": dr_info}

    # ------------------------------------------------------------- #
    def _baseline_u(self, s):
        """Position-hold baseline toward the current goal (hover regime)."""
        return self.ctrl_base._hover_update(s, self.goal, np.zeros(3),
                                            0.0, self.veh.dt_ctrl, 0.0)

    def _rays(self, s):
        out = np.full(N_RAYS, 1.0)
        if not self._obs_geoms:
            return out
        m, d = self.veh.model, self.veh.data
        pnt = d.qpos[0:3].copy()
        _, _, yaw = quat_to_euler(s["quat"])
        gid = np.zeros(1, dtype=np.int32)
        for k in range(N_RAYS):
            ang = yaw + k * 2 * np.pi / N_RAYS
            vec = np.array([np.cos(ang), np.sin(ang), 0.0])
            dist = mujoco.mj_ray(m, d, pnt, vec, self._ray_group, 1, -1, gid)
            if 0 <= dist <= RAY_MAX:
                out[k] = dist / RAY_MAX
        return out

    def _obs(self, s):
        _, _, yaw = quat_to_euler(s["quat"])
        cy, sy = np.cos(-yaw), np.sin(-yaw)
        g = self.goal - s["pos"]
        g_yaw = np.array([cy * g[0] - sy * g[1],
                          sy * g[0] + cy * g[1], g[2]])
        obs = np.concatenate([
            s["quat"],
            s["omega_b"] / 5.0,
            s["vel_b"] / 20.0,
            [s["airspeed"] / 20.0],
            [s["pos"][2] / 20.0],
            s["tilt"] / np.deg2rad(90),
            self.last_action,
            g_yaw / 20.0,
            self._rays(s),
        ]).astype(np.float32)
        return obs

    def _check_collision(self):
        d = self.veh.data
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if pair & self._obs_geoms and pair & self._drone_geoms:
                return True
            if self._ground_geom in pair and pair & self._drone_geoms:
                return True                              # airborne tasks
        return False

    # ------------------------------------------------------------- #
    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1, 1)
        s = self.veh.state()
        for _ in range(2):                               # 50 Hz -> 100 Hz
            if self.action_mode == "A":
                u = np.concatenate([(action[0:6] + 1) / 2, action[6:9]])
            else:
                u = self._baseline_u(s)
                u = u + self.residual_scale * action
                u[0:6] = np.clip(u[0:6], 0, 1)
                u[6:9] = np.clip(u[6:9], -1, 1)
            s = self.veh.step(u)

        self.steps += 1
        pos, vel, omega = s["pos"], s["vel_w"], s["omega_b"]
        dist = np.linalg.norm(self.goal - pos)
        roll, pitch, _ = quat_to_euler(s["quat"])

        # ---------- reward ----------
        da = action - self.last_action
        r = 1.0
        r -= 0.15 * min(dist, 6.0)
        r -= 0.03 * np.linalg.norm(vel)
        r -= 0.05 * np.linalg.norm(omega)
        r -= 0.10 * np.linalg.norm(da)
        r -= 0.01 * np.linalg.norm(action)
        if self.task == "obstacle":
            r += 2.0 * (self.prev_dist - dist)           # progress shaping
        self.prev_dist = dist
        self.last_action = action.copy()

        terminated = False
        info = {}
        # waypoint advance / success
        if dist < 1.0 and np.linalg.norm(vel) < 1.5:
            if self.wp_index + 1 < len(self.waypoints):
                self.wp_index += 1
                self.goal = self.waypoints[self.wp_index]
                self.prev_dist = np.linalg.norm(self.goal - pos)
                r += 10.0
            elif self.task in ("waypoint", "obstacle"):
                r += 50.0
                terminated = True
                info["is_success"] = True

        collided = self._check_collision()
        crashed = (pos[2] < 0.4 or abs(roll) > np.deg2rad(70)
                   or abs(pitch) > np.deg2rad(70) or dist > 45.0
                   or not np.all(np.isfinite(pos)))
        if collided or crashed:
            r -= 50.0
            terminated = True
            info["is_success"] = False
            info["collision"] = bool(collided)

        truncated = self.steps >= self.max_steps
        if truncated and "is_success" not in info:
            # hover succeeds by surviving on target to the time limit
            info["is_success"] = bool(self.task == "hover" and dist < 1.5)

        return self._obs(s), float(r), terminated, truncated, info
