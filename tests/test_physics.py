"""Phase 2 acceptance tests: motor, servo, aero, battery, vehicle.

Brief thresholds:
  hover thrust 1.18 kgf/motor within 5 %
  stall speed 11.5 +/- 1 m/s
  level trim at 17 m/s with L/D 8 +/- 1
  deterministic with a fixed seed
"""
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from params import load_params  # noqa: E402
from sim.motor import Motor, Battery  # noqa: E402
from sim.servo import Servo  # noqa: E402
from sim.aero import WingAero  # noqa: E402
from sim.wind import Wind  # noqa: E402
from sim.vehicle import Vehicle  # noqa: E402

P = load_params()
G = 9.81
WEIGHT = P["mass"]["total"] * G
DT = P["simulation"]["physics_dt"]


# --------------------------- motor --------------------------------------- #
def test_motor_hover_thrust_within_5pct():
    """At the hover operating point the motor must produce 1.18 kgf +/- 5 %."""
    m = Motor(P, +1)
    w_hover = np.sqrt(P["propulsion"]["hover_thrust_per_motor"]
                      / P["propulsion"]["kT"])
    thr = w_hover / P["propulsion"]["max_omega"]
    T = 0.0
    for _ in range(int(1.0 / DT)):          # 1 s to settle the lag
        T, _ = m.step(thr, v_axial=0.0, dt=DT)
    target = P["propulsion"]["hover_thrust_per_motor"]
    assert abs(T - target) / target < 0.05, f"T={T:.2f} vs {target:.2f} N"


def test_motor_first_order_lag():
    """63 % of a step reached after ~one time constant (0.07 s)."""
    m = Motor(P, +1)
    w_cmd = P["propulsion"]["max_omega"]
    n = int(P["propulsion"]["motor_lag"] / DT)
    for _ in range(n):
        m.step(1.0, 0.0, DT)
    assert 0.55 < m.omega / w_cmd < 0.70


def test_motor_thrust_decays_with_inflow():
    m = Motor(P, +1)
    for _ in range(500):
        m.step(0.8, 0.0, DT)
    T0, _ = m.step(0.8, 0.0, DT)
    T10, _ = m.step(0.8, 10.0, DT)
    T17, _ = m.step(0.8, 17.0, DT)
    assert T10 < T0
    assert T17 < T10
    # zero thrust at pitch speed
    v_pitch = m.omega / (2 * np.pi) * m.pitch_m
    T_vp, _ = m.step(0.8, v_pitch, DT)
    assert T_vp == pytest.approx(0.0, abs=0.3)


def test_motor_torque_ratio_and_sign():
    m_ccw = Motor(P, +1)
    m_cw = Motor(P, -1)
    for _ in range(800):
        T, Q1 = m_ccw.step(0.7, 0.0, DT)
        _, Q2 = m_cw.step(0.7, 0.0, DT)
    assert Q1 == pytest.approx(-Q2)            # opposite reaction
    assert abs(Q1) / T == pytest.approx(P["propulsion"]["kQ_over_kT"], rel=0.01)
    # magnitude in the briefed hover band 0.17-0.23 N*m near hover thrust
    m = Motor(P, +1)
    for _ in range(800):
        T, Q = m.step(np.sqrt(11.57 / 2.6e-5) / 889.0, 0.0, DT)
    assert 0.15 < abs(Q) < 0.25


# --------------------------- servo --------------------------------------- #
def test_servo_rate_limit():
    rate = np.deg2rad(120)
    s = Servo(rate, tau=0.05)
    prev = 0.0
    max_slope = 0.0
    for _ in range(200):
        out = s.step(np.deg2rad(90), DT)
        max_slope = max(max_slope, (out - prev) / DT)
        prev = out
    assert max_slope <= rate * 1.01


def test_servo_backlash_deadband():
    s = Servo(np.deg2rad(500), tau=0.01, backlash_rad=np.deg2rad(0.5))
    for _ in range(300):
        s.step(np.deg2rad(10), DT)
    up = s.out
    # reverse by less than the backlash: output should barely move
    for _ in range(300):
        s.step(np.deg2rad(10) - np.deg2rad(0.4), DT)
    assert abs(s.out - up) < np.deg2rad(0.15)


# --------------------------- battery ------------------------------------- #
def test_battery_sags_and_drains():
    b = Battery(P)
    v0, _ = b.step([0, 0, 0], DT)
    v1, scale = b.step([11.57, 11.57, 11.57], DT)
    assert v1 < v0                      # sag under load
    assert b.soc < 1.0                  # energy drained
    assert 0.9 < scale <= 25.2 / 22.2


# --------------------------- aero ---------------------------------------- #
def _level_flight_forces(aero, V, alpha):
    """Body-frame net force for straight flight at speed V, AoA alpha."""
    v_air_b = np.array([V * np.cos(alpha), 0.0, -V * np.sin(alpha)])
    hinge = dict(aileron_left=0.0, aileron_right=0.0, elevator=0.0, rudder=0.0)
    forces, _ = aero.forces(v_air_b, np.zeros(3), hinge, slip=[],
                            notch_open=False)
    F = np.sum([f for f, _ in forces], axis=0)
    # rotate to wind axes: lift perpendicular to flight path, drag along it
    ca, sa = np.cos(alpha), np.sin(alpha)
    lift = F[2] * ca - F[0] * sa * -1.0  # project onto "up" perpendicular
    lift = F[0] * np.sin(alpha) + F[2] * np.cos(alpha)
    drag = -(F[0] * np.cos(alpha) - F[2] * np.sin(alpha))
    return lift, drag


def test_trim_17ms_L_over_D():
    """Find alpha with lift = weight at 17 m/s; L/D must be 8 +/- 1."""
    aero = WingAero(P)
    from scipy.optimize import brentq
    f = lambda a: _level_flight_forces(aero, 17.0, a)[0] - WEIGHT
    alpha = brentq(f, np.deg2rad(-5), np.deg2rad(10))
    lift, drag = _level_flight_forces(aero, 17.0, alpha)
    ld = lift / drag
    assert 0 < np.rad2deg(alpha) < 6, f"trim alpha {np.rad2deg(alpha):.1f} deg"
    assert 7.0 <= ld <= 9.0, f"L/D = {ld:.2f}"


def test_stall_speed():
    """Slowest speed with lift = weight (sweep) must be 11.5 +/- 1 m/s."""
    aero = WingAero(P)
    speeds = np.linspace(8, 16, 81)
    v_min = None
    for V in speeds:
        alphas = np.linspace(np.deg2rad(-2), np.deg2rad(14), 60)
        lifts = [_level_flight_forces(aero, V, a)[0] for a in alphas]
        if max(lifts) >= WEIGHT:
            v_min = V
            break
    assert v_min is not None, "never reached lift = weight"
    assert abs(v_min - P["aero"]["stall_speed"]) <= 1.0, f"stall at {v_min:.2f}"


def test_restoring_moments_signs():
    """Static stability: pitch-up disturbance -> nose-down moment; sideslip ->
    restoring yaw. Checked via moments about the CG from the force set."""
    aero = WingAero(P)
    hinge = dict(aileron_left=0.0, aileron_right=0.0, elevator=0.0, rudder=0.0)

    def moment(v_air_b):
        forces, tq = aero.forces(v_air_b, np.zeros(3), hinge, [], False)
        M = tq.copy()
        for F, r in forces:
            M += np.cross(r, F)
        return M

    # pitch: in FLU, +M_y rotates the nose DOWN (R_y maps x to -z). Static
    # stability therefore requires dM_y/d(alpha) > 0: higher AoA produces a
    # stronger nose-down (restoring) moment.
    M_lo = moment(np.array([17.0, 0, -17.0 * np.tan(np.deg2rad(2))]))
    M_hi = moment(np.array([17.0, 0, -17.0 * np.tan(np.deg2rad(6))]))
    assert M_hi[1] > M_lo[1], "not statically stable in pitch"
    # yaw: wind from the right (v_y negative velocity? drone slips right ->
    # v_air_b_y < 0) -> restoring moment must yaw nose right (M_z < 0)
    M_slip = moment(np.array([15.0, -2.0, 0.0]))
    assert M_slip[2] < 0, "fin does not weathervane"


def test_aileron_roll_sign():
    aero = WingAero(P)
    hinge = dict(aileron_left=-0.2, aileron_right=+0.2,  # +cmd mapping
                 elevator=0.0, rudder=0.0)
    forces, tq = aero.forces(np.array([15.0, 0, 0]), np.zeros(3), hinge, [],
                             False)
    M = tq.copy()
    for F, r in forces:
        M += np.cross(r, F)
    assert M[0] > 0, "+aileron command must give +roll moment (roll right)"


# --------------------------- wind ----------------------------------------- #
def test_wind_deterministic():
    w1 = Wind(steady=(2, 0, 0), gust_sigma=1.0, seed=42)
    w2 = Wind(steady=(2, 0, 0), gust_sigma=1.0, seed=42)
    s1 = [w1.step(DT) for _ in range(100)]
    s2 = [w2.step(DT) for _ in range(100)]
    assert np.allclose(s1, s2)


# --------------------------- vehicle -------------------------------------- #
@pytest.fixture(scope="module")
def veh():
    return Vehicle(seed=7)


def test_vehicle_hover_balance(veh):
    """At hover throttle with tilt 0 the net vertical accel is near zero for
    the first instants (open loop, so only a short window is meaningful)."""
    w_h = np.sqrt(P["propulsion"]["hover_thrust_per_motor"]
                  / P["propulsion"]["kT"])
    thr_h = w_h / P["propulsion"]["max_omega"]
    veh.reset(seed=7, pos=(0, 0, 5), hover_init=True)
    u = np.array([thr_h, thr_h, thr_h, 0, 0, 0, 0, 0, 0])
    s = None
    for _ in range(20):                      # 0.2 s
        s = veh.step(u, wind_override=(0, 0, 0))
    assert abs(s["vel_w"][2]) < 0.4, f"vz={s['vel_w'][2]:.3f} m/s"


def test_vehicle_tail_torque_induces_yaw(veh):
    """With all three motors at hover and no compensation, the uncancelled
    tail reaction torque must slowly yaw the vehicle (D-04)."""
    w_h = np.sqrt(P["propulsion"]["hover_thrust_per_motor"]
                  / P["propulsion"]["kT"])
    thr_h = w_h / P["propulsion"]["max_omega"]
    veh.reset(seed=7, pos=(0, 0, 5), hover_init=True)
    u = np.array([thr_h, thr_h, thr_h, 0, 0, 0, 0, 0, 0])
    for _ in range(50):                      # 0.5 s
        s = veh.step(u, wind_override=(0, 0, 0))
    assert abs(s["omega_b"][2]) > 0.05, "expected yaw drift from tail torque"


def test_vehicle_deterministic():
    v1 = Vehicle(seed=3)
    v2 = Vehicle(seed=3)
    u = np.array([0.6, 0.6, 0.6, 0, 0, 0, 0.1, -0.1, 0.05])
    v1.reset(seed=3, pos=(0, 0, 5), hover_init=True)
    v2.reset(seed=3, pos=(0, 0, 5), hover_init=True)
    for _ in range(50):
        s1 = v1.step(u)
        s2 = v2.step(u)
    assert np.allclose(s1["pos"], s2["pos"])
    assert np.allclose(s1["omega_b"], s2["omega_b"])


def test_vehicle_sensors(veh):
    veh.reset(seed=7, pos=(0, 0, 5), hover_init=True)
    veh.step(np.array([0.6, 0.6, 0.6, 0, 0, 0, 0, 0, 0]))
    obs = veh.read_sensors()
    for k in ("acc", "gyro", "quat", "baro_alt", "pitot", "gps_pos", "range"):
        assert k in obs
    assert obs["baro_alt"] == pytest.approx(5.0, abs=1.0)


def test_benchmark_steps_per_second(veh, capsys):
    veh.reset(seed=1, pos=(0, 0, 5), hover_init=True)
    u = np.array([0.6, 0.6, 0.6, 0.2, 0.2, 0.1, 0, 0, 0])
    n = 300
    t0 = time.perf_counter()
    for _ in range(n):
        veh.step(u)
    dt_wall = time.perf_counter() - t0
    ctrl_hz = n / dt_wall
    phys_hz = ctrl_hz * veh.n_sub
    rt = ctrl_hz * veh.dt_ctrl
    with capsys.disabled():
        print(f"\n[benchmark] control steps/s: {ctrl_hz:,.0f} | physics steps/s:"
              f" {phys_hz:,.0f} | realtime factor: {rt:.1f}x")
    assert rt > 1.0, "simulation slower than realtime"
