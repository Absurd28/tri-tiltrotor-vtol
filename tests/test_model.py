"""Phase 1 acceptance tests: MJCF model vs the design budget.

Thresholds come from the project brief:
  mass within 2 %, CG within 1 cm, inertia within 10 %,
  full tilt sweep without self-collision,
  prop clearance >= 10 % of prop radius at every tilt angle.
"""
import sys
from pathlib import Path

import numpy as np
import pytest
import mujoco

sys.path.insert(0, str(Path(__file__).parent.parent))

from sim.model_check import (  # noqa: E402
    MODEL_XML, load_model, mass_report, clearance_sweep,
)


@pytest.fixture(scope="module")
def model_data():
    return load_model()


@pytest.fixture(scope="module")
def report(model_data):
    return mass_report(*model_data)


@pytest.fixture(scope="module")
def sweep(model_data):
    return clearance_sweep(*model_data)


def test_model_loads(model_data):
    model, _ = model_data
    assert model.nbody == 11          # world, base, 3 tilt pods, 6 surfaces
    assert model.njnt == 10           # free + 3 tilt + 6 surface hinges
    assert model.nu == 9              # 3 tilt + 6 surface position servos


def test_build_script_regenerates(tmp_path):
    """build_mjcf.build() must emit XML MuJoCo can compile."""
    from model.build_mjcf import build
    xml = build(write_info=False)
    m = mujoco.MjModel.from_xml_string(xml)
    assert m.nu == 9


def test_total_mass_within_2pct(report):
    assert abs(report["mass_err_pct"]) <= 2.0, (
        f"mass {report['mass']:.4f} vs budget {report['mass_budget']}"
    )


def test_cg_within_1cm(report):
    assert report["cg_norm"] <= 0.010, f"CG offset {report['cg']*100} cm"


def test_inertia_within_10pct(report):
    for ax, err in zip("xyz", report["inertia_err_pct"]):
        assert abs(err) <= 10.0, f"I{ax}{ax} error {err:+.1f}%"


def test_tilt_sweep_no_self_collision(sweep):
    assert sweep["min_struct_distance"] > 0.0, (
        f"structural interpenetration: {sweep['min_struct_info']}"
    )


def test_prop_clearance(sweep):
    assert sweep["min_prop_clearance"] >= sweep["clearance_target"], (
        f"min prop clearance {sweep['min_prop_clearance']*100:.2f} cm "
        f"< target {sweep['clearance_target']*100:.2f} cm "
        f"at {sweep['min_prop_info']}"
    )


def test_tilt_joint_ranges(model_data):
    model, _ = model_data
    for jn, hi_deg in (("j_nacelle_left", 90), ("j_nacelle_right", 90),
                       ("j_tail_pod", 30)):
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        lo, hi = model.jnt_range[j]
        assert lo == pytest.approx(0.0)
        assert hi == pytest.approx(np.deg2rad(hi_deg), rel=1e-3)


def test_thrust_sites_and_sensors(model_data):
    model, _ = model_data
    for s in ("thrust_left", "thrust_right", "thrust_tail", "imu",
              "pitot", "range_fwd", "range_down"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s) >= 0, s
    for sn in ("imu_acc", "imu_gyro", "imu_vel", "att_quat",
               "range_fwd", "range_down"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sn) >= 0, sn


def test_actuator_names_and_order(model_data):
    model, _ = model_data
    expected = ["tilt_left", "tilt_right", "tilt_tail",
                "aileron_left", "aileron_right",
                "elevator_left", "elevator_right",
                "rudder_left", "rudder_right"]
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
             for i in range(model.nu)]
    assert names == expected


def test_sim_steps_stably(model_data):
    """200 steps parked on the ground: no NaN, no blow-up."""
    model, _ = model_data
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "parked")
    mujoco.mj_resetDataKeyframe(model, data, key)
    for _ in range(200):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert abs(data.qpos[2]) < 0.5, "drone should rest near the ground"
