"""Ground-station simulation core (gcs/sim_core.py)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from gcs.sim_core import FlightSim, to_frame, from_frame, _qz  # noqa: E402
from control.pid import quat_to_euler, euler_to_quat  # noqa: E402


def test_leg_frame_roundtrip():
    q = euler_to_quat(0.1, -0.05, 0.9)
    s = dict(pos=np.array([100.0, 50.0, 15.0]), vel_w=np.array([3.0, 4.0, 0.5]),
             quat=q, R_bw=np.eye(3), omega_b=np.zeros(3))
    psi, origin = 0.7, np.array([10.0, 20.0, 0.0])
    sl = to_frame(s, origin, psi)
    r, p, y = quat_to_euler(sl["quat"])
    assert np.allclose([r, p, y], [0.1, -0.05, 0.9 - 0.7], atol=1e-9)
    assert np.allclose(from_frame(sl["pos"], origin, psi), s["pos"])
    assert np.isclose(np.linalg.norm(sl["vel_w"]), np.linalg.norm(s["vel_w"]))


def test_payload_release_keeps_physics_consistent():
    """Dropping 0.5 kg must update the cached subtree mass (a stale value put
    the computed CG tens of metres away and crashed the return leg)."""
    sim = FlightSim()
    sim.veh.reset(seed=0, pos=(300.0, 300.0, 0.147))
    sim.s = sim.veh.state()
    m0 = sim.model.body_subtreemass[sim.bid_base]
    sim.release_payload()
    assert np.isclose(sim.model.body_subtreemass[sim.bid_base], m0 - 0.5)
    for _ in range(20):
        sim.step()
    com = sim.veh.data.subtree_com[sim.bid_base]
    assert np.linalg.norm(com[:2] - sim.s["pos"][:2]) < 0.1
    sim._set_payload(True)
    assert np.isclose(sim.model.body_subtreemass[sim.bid_base], m0)


def test_manual_position_takeoff_and_hold():
    sim = FlightSim()
    sim.rl_assist = False
    sim.arm(True)
    sim.sticks["thr"] = 0.8                      # climb
    for _ in range(50 * 6):
        sim.step()
    sim.sticks["thr"] = 0.0                      # hold
    for _ in range(50 * 4):
        sim.step()
    z = sim.s["pos"][2]
    assert 4.0 < z < 12.0 and not sim.crashed
    assert np.linalg.norm(sim.s["vel_w"]) < 1.0


def test_arm_refused_when_airborne_or_crashed():
    sim = FlightSim()
    sim.crashed = True
    sim.arm(True)
    assert not sim.armed


@pytest.mark.slow
def test_multicopter_delivery_round_trip():
    sim = FlightSim()
    assert sim.start_delivery((40.0, -30.0))
    delivered = None
    for _ in range(50 * 200):
        sim.step()
        if sim.mission and "delivery_err" in sim.mission:
            delivered = sim.mission["delivery_err"]
        if sim.phase in ("COMPLETE", "CRASHED"):
            break
    assert sim.phase == "COMPLETE", sim.message
    assert delivered is not None and delivered < 1.0
    assert not sim.payload_attached
    assert np.linalg.norm(sim.s["pos"][:2]) < 1.0
