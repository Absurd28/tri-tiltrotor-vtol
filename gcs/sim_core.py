"""Ground-station flight simulation core (headless, testable).

Wraps the validated physics (sim.vehicle) and baseline controller
(control.baseline) for interactive flying:

  MANUAL / ANGLE     sticks -> roll/pitch angle, yaw rate, collective thrust
                     (like a real "stabilize" mode: the pilot flies the height)
  MANUAL / POSITION  sticks -> velocity setpoints, centred throttle holds
                     altitude; optional RL residual assist; VTOL switch flies
                     the scripted transition to wing-borne flight where the
                     sticks steer heading and altitude
  AUTO               delivery: take off, transition, cruise to the
                     destination, back-transition, precision approach (RL
                     residual), land, release the payload, fly home, land.

Leg frame: the baseline transition logic was written and validated for flight
along world +x. Instead of touching that controller, every fixed-wing leg runs
it in a rotated "leg frame" whose +x points along the leg. The controller sees
the state rotated by -psi; its outputs (throttles, tilts, surfaces) are body
quantities and need no rotation. Physics stays in the world frame.

RL policy: residual on the baseline hover law (mode B, D-15/D-16). Trained in
the hover regime with heading ~0, so its observation is built from the state
rotated into the current heading frame (the hover physics is yaw-invariant
apart from wind). It is used where it was trained: hover-regime flight
(manual POSITION assist, AUTO precision approach). Takeoff, transition, cruise
and landing are flown by the baseline (RL transition was a scoped-out stage).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402
from sim.vehicle import Vehicle  # noqa: E402
from sim.wind import Wind  # noqa: E402
from control.baseline import BaselineController, wrap_pi  # noqa: E402
from control.pid import quat_to_euler, euler_to_quat, attitude_rate_cmd  # noqa: E402

G = 9.81
CRUISE_ALT = 15.0          # the altitude the baseline mission was validated at
MIN_FW_DIST = 300.0        # shorter hops are flown as a multicopter
BACK_DIST = 220.0          # start the back-transition this far from the pad
PAYLOAD_KG = 0.5


# ---------------------------------------------------------------- frames --- #
def _qz(psi):
    return np.array([np.cos(psi / 2), 0.0, 0.0, np.sin(psi / 2)])


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _Rz(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def to_frame(s, origin, psi):
    """Express a Vehicle.state() dict in a frame rotated by psi about z and
    translated to origin. Body-frame entries are unchanged."""
    R = _Rz(-psi)
    out = dict(s)
    out["pos"] = R @ (s["pos"] - origin)
    out["vel_w"] = R @ s["vel_w"]
    out["quat"] = _qmul(_qz(-psi), s["quat"])
    out["R_bw"] = R @ s["R_bw"]
    return out


def from_frame(p, origin, psi):
    return _Rz(psi) @ np.asarray(p) + origin


# ----------------------------------------------------------------- world --- #
# 6x6 m "buildings" (visual only, no collision) for orientation in the video
LANDMARKS = [(round((250 + 150 * (k % 3)) * np.cos(k * np.pi / 6), 1),
              round((250 + 150 * (k % 3)) * np.sin(k * np.pi / 6), 1),
              6 + 4 * (k % 4)) for k in range(12)]


def build_world_xml():
    """The vehicle MJCF plus a large ground, sky, sun and mocap markers."""
    xml = (ROOT / "model" / "vtol.xml").read_text()
    xml = xml.replace('size="12 12 0.1" material="grid"',
                      'size="3000 3000 0.1" material="grid"')
    xml = xml.replace('texrepeat="12 12"', 'texrepeat="1200 1200"')
    xml = xml.replace(
        "<asset>",
        '<asset>\n    <texture type="skybox" builtin="gradient" '
        'rgb1="0.55 0.72 0.92" rgb2="0.93 0.95 0.98" width="512" height="512"/>')
    xml = xml.replace(
        '<global offwidth="1280" offheight="960"/>',
        '<global offwidth="1280" offheight="960"/>\n'
        '    <map znear="0.005" zfar="400"/>')
    xml = xml.replace("<option ", '<statistic extent="2" center="0 0 1"/>\n'
                      "  <option ", 1)
    markers = """
    <light directional="true" dir="-0.4 -0.3 -1" diffuse="0.6 0.6 0.6" castshadow="false"/>
    <body name="pad_home" mocap="true" pos="0 0 0.002">
      <geom type="cylinder" size="1.2 0.002" rgba="0.15 0.55 0.95 1" contype="0" conaffinity="0"/>
      <geom type="box" size="0.7 0.12 0.003" pos="0 0 0.001" rgba="1 1 1 1" contype="0" conaffinity="0"/>
    </body>
    <body name="pad_dest" mocap="true" pos="0 0 -50">
      <geom type="cylinder" size="1.2 0.002" rgba="0.95 0.45 0.1 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" size="0.08 4" pos="0 0 4" rgba="0.95 0.45 0.1 0.5" contype="0" conaffinity="0"/>
    </body>
    <body name="parcel" mocap="true" pos="0 0 -50">
      <geom type="box" size="0.12 0.1 0.08" pos="0 0 0.08" rgba="0.9 0.5 0.1 1" contype="0" conaffinity="0"/>
    </body>
"""
    for (x, y, h) in LANDMARKS:  # distant landmarks for spatial reference
        markers += (f'    <geom type="box" size="3 3 {h/2:.1f}" pos="{x:.1f} '
                    f'{y:.1f} {h/2:.1f}" rgba="0.55 0.58 0.62 1" '
                    f'contype="0" conaffinity="0"/>\n')
    xml = xml.replace("</worldbody>", markers + "  </worldbody>")
    return xml


# ---------------------------------------------------------------- policy --- #
class ResidualPolicy:
    """Loads runs/<stage>/best.onnx (validation-selected, D-17; ONNX as on the
    companion computer), falling back to final/zip/latest checkpoint.
    `available` is False if nothing trained exists yet."""

    SEARCH = ("s2_waypoint", "s1_hover")

    def __init__(self, path=None):
        self.name, self._run = None, None
        cands = [Path(path)] if path else []
        for st in self.SEARCH:
            d = ROOT / "runs" / st
            cands += [d / "best.onnx", d / "best.zip", d / "final.onnx", d / "final.zip"]
            cands += sorted(d.glob("ckpt_*_steps.zip"),
                            key=lambda p: int(p.stem.split("_")[1]))[::-1][:1]
        for c in cands:
            if c.exists():
                self._load(c)
                break

    def _load(self, p):
        if p.suffix == ".onnx":
            import onnxruntime as ort
            sess = ort.InferenceSession(str(p), providers=["CPUExecutionProvider"])
            self._run = lambda o: sess.run(None, {"obs": o[None].astype(np.float32)})[0][0]
        else:
            from stable_baselines3 import PPO
            m = PPO.load(str(p), device="cpu")
            self._run = lambda o: m.predict(o, deterministic=True)[0]
        self.name = f"{p.parent.name}/{p.name}"

    @property
    def available(self):
        return self._run is not None

    def __call__(self, obs):
        return np.clip(np.asarray(self._run(obs), dtype=float), -1, 1)


# ------------------------------------------------------------------- sim --- #
class FlightSim:
    def __init__(self, policy_path=None, seed=0):
        self.p = load_params()
        self.model = mujoco.MjModel.from_xml_string(build_world_xml())
        self.wind = Wind(seed=seed)
        self.veh = Vehicle(model=self.model, params=self.p, wind=self.wind,
                           seed=seed)
        self.ctrl = BaselineController(self.p)
        self.policy = ResidualPolicy(policy_path)
        # observation builder shared with training (rl/env.py)
        from rl.env import VTOLEnv
        self._env = VTOLEnv(task="hover", action_mode="B")
        self._env.veh = self.veh
        self._env._obs_geoms = set()
        self.residual_scale = self._env.residual_scale

        m = self.model
        self.bid_base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.gid_payload = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "payload")
        self.mocap = {n: m.body_mocapid[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, n)] for n in ("pad_home", "pad_dest", "parcel")}
        self._mass0 = m.body_mass[self.bid_base]
        self._ipos0 = m.body_ipos[self.bid_base].copy()
        self._inert0 = m.body_inertia[self.bid_base].copy()
        self._payload_pos = m.geom_pos[self.gid_payload].copy()
        self._mass_now = self._mass0
        self.reset()

    # ------------------------------------------------------------ setup --- #
    def reset(self, wind_ms=None):
        if wind_ms is not None:
            ang = np.deg2rad(30.0)
            self.wind.steady = np.array([wind_ms * np.cos(ang),
                                         wind_ms * np.sin(ang), 0.0])
            self.wind.sigma = 0.2 * wind_ms
        self._set_payload(True)
        self.veh.reset(seed=0, pos=(0, 0, 0.147))
        self.ctrl.reset()
        self.ctrl.m = self.p["mass"]["total"]
        self.armed = False
        self.flight_mode = "MANUAL"     # MANUAL | AUTO
        self.manual_mode = "POSITION"   # POSITION | ANGLE
        self.fw = False                 # wing-borne (manual VTOL switch)
        self.rl_assist = True
        self.rl_active = False
        self.phase = "DISARMED"
        self.message = "Ready. Arm to fly."
        self.crashed = False
        self.sticks = dict(thr=0.0, yaw=0.0, pitch=0.0, roll=0.0)
        self.sp = np.array([0.0, 0.0, 0.0])
        self.yaw_sp = 0.0
        self.leg = None
        self.mission = None
        self.last_action = np.zeros(9)
        self._tick = 0
        self.track = []
        self.home = np.zeros(3)
        d = self.veh.data
        d.mocap_pos[self.mocap["pad_home"]] = [0, 0, 0.002]
        d.mocap_pos[self.mocap["pad_dest"]] = [0, 0, -50]
        d.mocap_pos[self.mocap["parcel"]] = [0, 0, -50]
        mujoco.mj_forward(self.model, d)
        self.s = self.veh.state()

    def _set_payload(self, attached):
        m = self.model
        if attached:
            m.body_mass[self.bid_base] = self._mass0
            m.body_ipos[self.bid_base] = self._ipos0
            m.body_inertia[self.bid_base] = self._inert0
            m.geom_rgba[self.gid_payload][3] = 1.0
        else:
            M = self._mass0
            m.body_mass[self.bid_base] = M - PAYLOAD_KG
            m.body_ipos[self.bid_base] = (M * self._ipos0
                                          - PAYLOAD_KG * self._payload_pos) / (M - PAYLOAD_KG)
            # first-order: the 0.5 kg box sits 6 cm under the CG, its own
            # inertia contribution is small -> scale the tensor by mass ratio
            m.body_inertia[self.bid_base] = self._inert0 * (M - PAYLOAD_KG) / M
            m.geom_rgba[self.gid_payload][3] = 0.0
        # MuJoCo caches subtree masses at compile time and divides by them in
        # subtree_com; leaving them stale scaled the computed CG toward the
        # world origin (40 m off at a 300 m pad -> aero lever arms exploded)
        dm = m.body_mass[self.bid_base] - self._mass_now
        for b in (self.bid_base, 0):
            m.body_subtreemass[b] += dm
        self._mass_now = m.body_mass[self.bid_base]
        self.payload_attached = attached

    # ---------------------------------------------------------- commands --- #
    def arm(self, on=True):
        if on and self.crashed:
            self.message = "Crashed - reset first."
            return
        if on and self.s["pos"][2] > 0.4:
            return
        self.armed = on
        if on:
            self.sp = self.s["pos"].copy()
            self.yaw_sp = quat_to_euler(self.s["quat"])[2]
            self.ctrl.rate_i[:] = 0
            self.ctrl.vel_i[:] = 0
            self.phase = "ARMED"
            self.message = "Armed."
        else:
            self.phase = "DISARMED"
            self.flight_mode = "MANUAL"
            self.fw = False
            self.mission = None
            self.message = "Disarmed."

    def set_mode(self, mode):
        if mode == self.flight_mode:
            return
        if mode == "MANUAL":
            # pilot takes over: hold the current point (hover regime only)
            self.flight_mode = "MANUAL"
            self.mission = None
            if self.leg is not None and self.ctrl.mode in ("TRANSITION", "CRUISE"):
                self.fw = True               # still wing-borne: keep FW handling
                self._fw_heading = quat_to_euler(self.s["quat"])[2]
                self._fw_alt = max(self.s["pos"][2], 10.0)
            else:
                self.leg = None
                self.fw = False
                self.sp = self.s["pos"].copy()
                self.yaw_sp = quat_to_euler(self.s["quat"])[2]
            self.message = "Manual control."
        elif mode == "AUTO":
            self.message = "Auto: set a destination and press START."
            self.flight_mode = "AUTO"
            self.mission = None
            self.sp = self.s["pos"].copy()
            self.yaw_sp = quat_to_euler(self.s["quat"])[2]

    def start_delivery(self, dest_xy):
        dest = np.array([float(dest_xy[0]), float(dest_xy[1]), 0.0])
        pos = self.s["pos"]
        if self.crashed:
            self.message = "Crashed - reset first."
            return False
        if pos[2] > 0.4:
            self.message = "Land first: deliveries start on the ground."
            return False
        dist = np.linalg.norm(dest[:2] - pos[:2])
        if dist < 5.0:
            self.message = "Destination is less than 5 m away."
            return False
        if dist > 2500.0:
            self.message = "Destination beyond 2.5 km (battery reserve)."
            return False
        self.flight_mode = "AUTO"
        if not self.armed:
            self.arm(True)
        self.home = np.array([pos[0], pos[1], 0.0])
        self.mission = dict(dest=dest, home=self.home.copy(), leg_no=0,
                            fw=dist >= MIN_FW_DIST)
        self.veh.data.mocap_pos[self.mocap["pad_dest"]] = [dest[0], dest[1], 0.002]
        self._begin_leg(self.home, dest)
        return True

    def _begin_leg(self, a, b):
        """Fly from point a to point b (both on the ground)."""
        msn = self.mission
        psi = float(np.arctan2(b[1] - a[1], b[0] - a[0]))
        dist = float(np.linalg.norm((b - a)[:2]))
        self.leg = dict(origin=np.array([a[0], a[1], 0.0]), psi=psi, dist=dist,
                        target=b.copy())
        self.ctrl.reset()
        if msn["fw"]:
            self.ctrl.mission = dict(home=np.array([0.0, 0.0, CRUISE_ALT]),
                                     cruise_wp=np.array([dist, 0.0, CRUISE_ALT]),
                                     land=np.array([dist, 0.0, 0.0]))
        self.phase = "CLIMB"
        self.sp = np.array([a[0], a[1], CRUISE_ALT if msn["fw"] else 10.0])
        # heading is held through the climb (yawing near the ground flips a
        # tilt-rotor: the yaw actuator is differential nacelle tilt); the turn
        # to the leg heading happens in the ALIGN hover
        self.yaw_sp = quat_to_euler(self.s["quat"])[2]
        self.message = (f"Leg {msn['leg_no'] + 1}/2: "
                        f"{'to destination' if msn['leg_no'] == 0 else 'return home'}"
                        f" ({dist:.0f} m, {'VTOL fixed-wing' if msn['fw'] else 'multicopter'})")

    def release_payload(self):
        if not self.payload_attached:
            return
        self._set_payload(False)
        self.ctrl.m = self.p["mass"]["total"] - PAYLOAD_KG
        pos = self.s["pos"]
        R = self.s["R_bw"]
        side = R @ np.array([0.0, 0.7, 0.0])           # drop beside the skid
        self.veh.data.mocap_pos[self.mocap["parcel"]] = [pos[0] + side[0],
                                                         pos[1] + side[1], 0.0]
        self.message = "Payload released."

    # ------------------------------------------------------------- RL obs --- #
    def _rl_residual(self, s_world, goal_world, psi_ref):
        """Residual action from the trained policy for a hover-regime goal."""
        if not (self.rl_assist and self.policy.available):
            self.rl_active = False
            return np.zeros(9)
        origin = np.zeros(3)
        s_r = to_frame(s_world, origin, psi_ref)
        g = goal_world - s_world["pos"]
        if np.linalg.norm(g) > 10.0:                   # carrot inside training range
            g = g / np.linalg.norm(g) * 10.0
        env = self._env
        env.goal = s_r["pos"] + _Rz(-psi_ref) @ g
        env.last_action = self.last_action
        obs = env._obs(s_r)
        a = self.policy(obs)
        self.rl_active = True
        return a

    # --------------------------------------------------------------- step --- #
    def step(self):
        """One RL period (50 Hz) = 2 controller/physics periods."""
        dt = self.veh.dt_ctrl
        rl_a = None
        for k in range(2):
            u, rl_goal = self._control(self.s, dt)
            if rl_goal is not None:
                if rl_a is None:
                    rl_a = self._rl_residual(self.s, *rl_goal)
                u = u + self.residual_scale * rl_a
                u[0:6] = np.clip(u[0:6], 0, 1)
                u[6:9] = np.clip(u[6:9], -1, 1)
            self.s = self.veh.step(u)
        if rl_a is None:
            self.rl_active = False
            rl_a = np.zeros(9)
        self.last_action = rl_a
        self._tick += 1
        if self._tick % 10 == 0:
            self.track.append(self.s["pos"][:2].round(1).tolist())
            self.track = self.track[-3000:]
        self._check_crash()
        return self.s

    def _check_crash(self):
        if self.crashed:
            return
        s = self.s
        roll, pitch, _ = quat_to_euler(s["quat"])
        on_ground = s["pos"][2] < 0.3
        hard = on_ground and (np.linalg.norm(s["vel_w"]) > 4.0)
        flipped = abs(roll) > np.deg2rad(85) or abs(pitch) > np.deg2rad(85)
        if hard or (flipped and s["pos"][2] < 2.0) or not np.all(np.isfinite(s["pos"])):
            self.crashed = True
            self.armed = False
            self.phase = "CRASHED"
            self.message = "CRASH - press RESET."

    # ------------------------------------------------------------ control --- #
    def _control(self, s, dt):
        """Returns (u9, rl_goal or None). rl_goal = (goal_world, psi_ref)."""
        if not self.armed or self.crashed:
            return np.zeros(9), None
        if self.flight_mode == "AUTO" and self.mission is not None:
            return self._auto(s, dt)
        if self.flight_mode == "AUTO":                # auto, no mission: hold
            if s["pos"][2] < 0.3:
                return np.zeros(9), None
            return self._hold(s, self.sp, dt), (self.sp, self.yaw_sp)
        if self.fw:
            return self._manual_fw(s, dt), None
        if self.manual_mode == "ANGLE":
            return self._manual_angle(s, dt), None
        return self._manual_position(s, dt)

    def _hold(self, s, p, dt, v_ff=None):
        self.ctrl.mode = "HOVER"
        self.ctrl.yaw_cmd = self.yaw_sp
        return self.ctrl._hover_update(s, np.asarray(p, float),
                                       np.zeros(3) if v_ff is None else v_ff,
                                       0.0, dt, 0.0)

    # ---- manual: angle (stabilize) ---- #
    def _manual_angle(self, s, dt):
        st = self.sticks
        c = self.ctrl
        roll_cmd = np.deg2rad(30.0) * st["roll"]
        pitch_cmd = -np.deg2rad(30.0) * st["pitch"]     # stick forward = nose down
        self.yaw_sp = wrap_pi(self.yaw_sp - np.deg2rad(90.0) * st["yaw"] * dt)
        thr = (st["thr"] + 1) / 2                       # 0..1, mid-stick ~hover
        if thr < 0.05 and s["pos"][2] < 0.3:
            self.phase = "LANDED"
            return np.zeros(9)
        self.phase = "ANGLE"
        F = np.clip(thr * 2.0 * c.m * G, 0.0, 3 * c.mixer.T_max)
        roll, pitch, yaw = quat_to_euler(s["quat"])
        q_des = euler_to_quat(roll_cmd, pitch_cmd, self.yaw_sp)
        w_cmd = attitude_rate_cmd(s["quat"], q_des, c.kp_att)
        w_cmd[2] *= 0.4
        M = c._rate_loop(w_cmd, s["omega_b"], dt)
        V = max(float(s["v_air_b"][0]), 0.0)
        return c.mixer.mix(F, M, 0.0, V, v_scale=s["v_batt"] / 22.2)

    # ---- manual: position (assisted, RL residual) ---- #
    def _manual_position(self, s, dt):
        st = self.sticks
        vmax = self.p["control"]["max_vel_xy"]
        vz = 2.0 * st["thr"] if abs(st["thr"]) > 0.08 else 0.0
        self.yaw_sp = wrap_pi(self.yaw_sp - np.deg2rad(60.0) * st["yaw"] * dt)
        fwd, left = vmax * st["pitch"], -vmax * st["roll"]
        cy, sy = np.cos(self.yaw_sp), np.sin(self.yaw_sp)
        v_cmd = np.array([cy * fwd - sy * left, sy * fwd + cy * left, vz])
        on_ground = s["pos"][2] < 0.3
        if on_ground and vz <= 0.0:
            self.phase = "LANDED"
            self.sp = s["pos"].copy()
            return np.zeros(9), None
        self.phase = "POSITION"
        # integrate the setpoint, leashed to the vehicle so it cannot run away
        self.sp = self.sp + v_cmd * dt
        lead = self.sp - s["pos"]
        if np.linalg.norm(lead) > 3.0:
            self.sp = s["pos"] + lead / np.linalg.norm(lead) * 3.0
        self.sp[2] = max(self.sp[2], -0.5 if vz < 0 else 0.0)
        u = self._hold(s, self.sp, dt, v_ff=v_cmd)
        rl = (self.sp, self.yaw_sp) if s["pos"][2] > 1.5 else None
        return u, rl

    # ---- manual: VTOL switch ---- #
    def vtol_switch(self, to_fw):
        if self.flight_mode != "MANUAL" or self.manual_mode != "POSITION":
            self.message = "VTOL switch works in MANUAL / POSITION."
            return
        s = self.s
        if to_fw and not self.fw:
            if s["pos"][2] < 8.0:
                self.message = "Climb above 8 m before transitioning."
                return
            psi = quat_to_euler(s["quat"])[2]
            self.leg = dict(origin=s["pos"] * [1, 1, 0], psi=psi, dist=1e4,
                            target=None)
            self.ctrl.reset()
            alt = s["pos"][2]
            self.ctrl.mission = dict(home=np.array([0, 0, alt]),
                                     cruise_wp=np.array([1e4, 0, alt]),
                                     land=np.zeros(3))
            self.ctrl.mode = "TRANSITION"
            self._fw_heading, self._fw_alt = psi, alt
            self.fw = True
            self.message = "Transition to wing-borne flight..."
        elif not to_fw and self.fw:
            if self.ctrl.mode in ("TRANSITION", "CRUISE"):
                self.ctrl.mode, self.ctrl.timer = "BACK", 0.0
                self.message = "Back-transition to hover..."

    def _manual_fw(self, s, dt):
        st = self.sticks
        leg = self.leg
        sl = to_frame(s, leg["origin"], leg["psi"])
        c = self.ctrl
        if c.mode in ("CRUISE", "TRANSITION"):
            # right stick: heading rate / climb; target point far ahead
            self._fw_heading = wrap_pi(self._fw_heading - np.deg2rad(25) * st["roll"] * dt)
            # pull back (or throttle up) = climb, like flying a plane
            climb = 3.0 * np.clip(st["thr"] - st["pitch"], -1, 1)
            self._fw_alt = float(np.clip(self._fw_alt + climb * dt, 8.0, 120.0))
            h = self._fw_heading - leg["psi"]
            ahead = sl["pos"] + 400.0 * np.array([np.cos(h), np.sin(h), 0.0])
            c.mission["cruise_wp"] = np.array([ahead[0], ahead[1], self._fw_alt])
            if c.mode == "TRANSITION":
                c.mission["home"][2] = self._fw_alt
        elif c.mode == "BACK":
            # stop roughly 60 m ahead along the current track
            if not hasattr(self, "_back_set") or not self._back_set:
                v = sl["vel_w"][:2]
                d = v / max(np.linalg.norm(v), 1e-3) * 60.0
                c.mission["cruise_wp"] = np.array([sl["pos"][0] + d[0],
                                                   sl["pos"][1] + d[1],
                                                   c.mission["home"][2]])
                c.mission["home"][2] = sl["pos"][2]
                self._back_set = True
        u = c.update(sl, dt)
        self.phase = "FW_" + c.mode
        if c.mode in ("HOVER2", "LAND", "DONE"):
            # hover regained: hand back to the pilot in POSITION mode
            self.fw = False
            self._back_set = False
            self.leg = None
            self.sp = s["pos"].copy()
            self.yaw_sp = quat_to_euler(s["quat"])[2]
            self.message = "Hovering. Manual POSITION control."
        return u

    # ---- auto delivery ---- #
    def _auto(self, s, dt):
        msn, leg = self.mission, self.leg
        c = self.ctrl
        target = leg["target"]
        ph = self.phase

        if ph == "CLIMB":
            alt = self.sp[2]
            v_ff = np.array([0, 0, self.p["control"]["climb_rate"]])
            if s["pos"][2] > alt - 0.5:
                self.phase = "ALIGN"
            return self._hold(s, self.sp, dt, v_ff=v_ff), None

        if ph == "ALIGN":
            # yaw authority in hover is differential nacelle tilt, saturating
            # at ~+-7 deg: a fast turn excites a yaw oscillation the saturated
            # loop cannot damp (seen: divergence after a 30 deg/s turn). Turn
            # slowly, and pause the setpoint while the vehicle lags behind it.
            err = wrap_pi(leg["psi"] - self.yaw_sp)
            yaw = quat_to_euler(s["quat"])[2]
            lag = abs(wrap_pi(self.yaw_sp - yaw))
            step = np.deg2rad(10.0) * dt if lag < np.deg2rad(8) else 0.0
            self.yaw_sp = wrap_pi(self.yaw_sp + np.clip(err, -step, step))
            u = self._hold(s, self.sp, dt)
            if (abs(err) < 1e-3 and abs(wrap_pi(leg["psi"] - yaw)) < np.deg2rad(4)
                    and np.linalg.norm(s["vel_w"]) < 0.7):
                self.phase, self._t_settle = "SETTLE", 0.0
            return u, None

        if ph == "SETTLE":
            # the validated mission always hovered 3 s before the transition;
            # starting it while still yawing lets the yaw run away as the
            # nacelles tilt (seen in testing: 60 deg/s spin)
            yaw = quat_to_euler(s["quat"])[2]
            u = self._hold(s, self.sp, dt)
            # hover has an inherent +-2 deg / 5 deg/s yaw limit cycle (servo
            # backlash), so "calm" means: no turn in progress
            calm = (abs(s["omega_b"][2]) < np.deg2rad(6)
                    and abs(wrap_pi(leg["psi"] - yaw)) < np.deg2rad(5)
                    and np.linalg.norm(s["vel_w"]) < 0.6)
            self._t_settle += dt
            if self._t_settle > 3.0 and calm:
                if msn["fw"]:
                    # hand over to the validated mission controller in the
                    # leg frame, straight into the transition
                    c.rate_i[:] = 0
                    c.vel_i[:] = 0
                    c.yaw_cmd = 0.0
                    c.mode, c.timer = "TRANSITION", 0.0
                    for k in ("_tilt_ramp", "_vff_hold"):
                        if hasattr(c, k):
                            delattr(c, k)
                    self.phase = "TRANSITION"
                else:
                    self.phase = "MC_CRUISE"
            return u, None

        if ph in ("TRANSITION", "CRUISE", "BACK") and msn["fw"]:
            sl = to_frame(s, leg["origin"], leg["psi"])
            # the mission's 140 m back-transition trigger was tuned at full
            # mass; the light (delivered) aircraft keeps flying on the wing
            # down to ~10.7 m/s and overshot by 170 m in testing. Start the
            # deceleration earlier; the approach phase closes any gap.
            if c.mode == "CRUISE" and leg["dist"] - sl["pos"][0] < BACK_DIST:
                c.mode, c.timer = "BACK", 0.0
            u = c.update(sl, dt)
            self.phase = c.mode if c.mode in ("TRANSITION", "CRUISE", "BACK") else ph
            if c.mode in ("HOVER2", "LAND", "DONE"):
                self.phase = "APPROACH"
                self.sp = np.array([target[0], target[1], CRUISE_ALT])
                self.yaw_sp = quat_to_euler(s["quat"])[2]
                self.message = "Precision approach to the pad."
            return u, None

        if ph == "MC_CRUISE":
            # carrot along the leg at the hover speed limit
            to = target[:2] - s["pos"][:2]
            d = np.linalg.norm(to)
            carrot = s["pos"][:2] + to / max(d, 1e-6) * min(d, 6.0)
            self.sp = np.array([carrot[0], carrot[1], 10.0])
            if d < 6.0:
                self.phase = "APPROACH"
                self.sp = np.array([target[0], target[1], 10.0])
            return self._hold(s, self.sp, dt), (self.sp, self.yaw_sp)

        if ph == "APPROACH":
            # hover-regime flight to the pad: baseline + RL residual.
            # Far from the pad, fly NOSE-FIRST: tail-first flight lets the
            # fins fight the weak (differential-tilt) yaw authority and the
            # yaw diverged in testing. Turn on the spot first, slowly.
            goal = self.sp.copy()
            to = goal - s["pos"]
            d_xy = np.linalg.norm(to[:2])
            yaw = quat_to_euler(s["quat"])[2]
            if d_xy > 6.0:
                brg = float(np.arctan2(to[1], to[0]))
                err = wrap_pi(brg - self.yaw_sp)
                lag = abs(wrap_pi(self.yaw_sp - yaw))
                step_y = np.deg2rad(10.0) * dt if lag < np.deg2rad(8) else 0.0
                self.yaw_sp = wrap_pi(self.yaw_sp + np.clip(err, -step_y, step_y))
                aligned = abs(wrap_pi(brg - yaw)) < np.deg2rad(20)
                if not aligned:
                    if not hasattr(self, "_turn_pt") or self._turn_pt is None:
                        self._turn_pt = s["pos"].copy()
                    p_cmd = self._turn_pt.copy()
                    p_cmd[2] = goal[2]
                    return self._hold(s, p_cmd, dt), None
            self._turn_pt = None
            step = to.copy()
            if d_xy > 6.0:                                # carrot, not a leap
                step[:2] = step[:2] / d_xy * 6.0
            p_cmd = s["pos"] + step
            u = self._hold(s, p_cmd, dt)
            if np.linalg.norm(to[:2]) < 0.6 and np.linalg.norm(s["vel_w"]) < 0.6:
                self.phase = "DESCEND"
                self.message = "Landing."
            return u, (p_cmd, self.yaw_sp)

        if ph == "DESCEND":
            p = np.array([target[0], target[1], 0.0])
            c.mode = "LAND"
            c.land_pt = p
            c.yaw_cmd = self.yaw_sp
            v_ff = np.array([0, 0, -self.p["control"]["descent_rate"]])
            u = c._hover_update(s, p, v_ff, 0.0, dt, 0.0)
            if s["pos"][2] < 0.20 and abs(s["vel_w"][2]) < 0.4:
                self.phase = "ON_PAD"
                self._t_pad = 0.0
            return u, None

        if ph == "ON_PAD":
            self._t_pad += dt
            if msn["leg_no"] == 0 and self.payload_attached and self._t_pad > 1.0:
                self.release_payload()
                err = np.linalg.norm(s["pos"][:2] - target[:2])
                msn["delivery_err"] = float(err)
                self.message = f"Delivered ({err:.2f} m from the pad centre). Returning in 3 s."
            if self._t_pad > 4.0:
                if msn["leg_no"] == 0:
                    msn["leg_no"] = 1
                    self._begin_leg(target.copy(), msn["home"].copy())
                else:
                    err = np.linalg.norm(s["pos"][:2] - msn["home"][:2])
                    msn["home_err"] = float(err)
                    self.phase = "COMPLETE"
                    self.armed = False
                    self.mission = None
                    self.leg = None
                    self.message = (f"Mission complete. Landed {err:.2f} m from home. "
                                    "Disarmed.")
            return np.zeros(9), None

        return np.zeros(9), None

    # ------------------------------------------------------------ telemetry --- #
    def telemetry(self):
        s = self.s
        roll, pitch, yaw = quat_to_euler(s["quat"])
        msn = self.mission
        dist_to = None
        if self.leg is not None and self.leg.get("target") is not None:
            dist_to = float(np.linalg.norm(self.leg["target"][:2] - s["pos"][:2]))
        return dict(
            t=round(float(s["time"]), 2),
            pos=[round(float(v), 2) for v in s["pos"]],
            vel=[round(float(v), 2) for v in s["vel_w"]],
            roll=round(np.degrees(roll), 1), pitch=round(np.degrees(pitch), 1),
            yaw=round(np.degrees(yaw), 1),
            airspeed=round(float(max(s["v_air_b"][0], 0.0)), 1),
            gs=round(float(np.linalg.norm(s["vel_w"][:2])), 1),
            vz=round(float(s["vel_w"][2]), 2),
            tilt=[round(float(np.degrees(v)), 0) for v in s["tilt"]],
            thrust=[round(float(v), 1) for v in s["thrusts"]],
            batt_v=round(float(s["v_batt"]), 2),
            soc=round(100 * float(s["soc"]), 1),
            armed=self.armed, mode=self.flight_mode, manual_mode=self.manual_mode,
            fw=self.fw, phase=self.phase, message=self.message,
            rl_assist=self.rl_assist, rl_active=self.rl_active,
            rl_policy=self.policy.name, payload=self.payload_attached,
            crashed=self.crashed,
            home=[round(float(v), 1) for v in self.home[:2]],
            dest=None if msn is None else [round(float(v), 1) for v in msn["dest"][:2]],
            dist_to=None if dist_to is None else round(dist_to, 1),
            wind=[round(float(v), 1) for v in self.wind.steady[:2]],
        )
