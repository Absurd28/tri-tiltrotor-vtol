"""Model acceptance checks: mass, CG, inertia, tilt-sweep clearances.

Used by scripts/check_model.py (human report) and tests/test_model.py
(pytest acceptance). All physics queries go through MuJoCo itself, so the
numbers are exactly what the simulation will use.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402

MODEL_XML = ROOT / "model" / "vtol.xml"

# Prop geoms and the airframe geoms they must clear at every tilt angle.
PROP_GEOMS = ("prop_left", "prop_right", "prop_tail")
AIRFRAME_GEOMS = (
    "nose_box", "mid_box", "rear_box", "bulkhead",
    "wing_main", "wing_te_center", "aileron_left", "aileron_right",
    "boom_left", "boom_right", "fin_left", "fin_right",
    "stab_left", "stab_right", "elevator_left", "elevator_right",
    "rudder_left", "rudder_right",
    "arm_left", "arm_right", "skid_left", "skid_right", "skid_nose",
)

# Structural pairs checked for interpenetration during the tilt sweep.
# Mechanically-connected pairs (motor on its own axle/arm tip, axle through
# its boom bearings) are intentionally excluded.
STRUCT_PAIRS = (
    ("motor_left", "wing_main"), ("motor_left", "nose_box"),
    ("motor_right", "wing_main"), ("motor_right", "nose_box"),
    ("motor_tail", "bulkhead"), ("motor_tail", "boom_left"),
    ("motor_tail", "boom_right"), ("motor_tail", "rear_box"),
)


def load_model(xml_path: Path | str = MODEL_XML):
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "parked")
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data


def mass_report(model, data) -> dict:
    """Total mass, CG in body frame, composite inertia about the CG."""
    p = load_params()
    total = float(mujoco.mj_getTotalmass(model))

    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    cg_world = data.subtree_com[base].copy()
    cg_body = cg_world - data.xpos[base]  # base orientation = identity

    # Composite inertia about the CG: parallel-axis sum over all bodies.
    inertia = np.zeros((3, 3))
    eye = np.eye(3)
    for i in range(1, model.nbody):
        m = model.body_mass[i]
        if m <= 0:
            continue
        R = data.ximat[i].reshape(3, 3)
        I_body = R @ np.diag(model.body_inertia[i]) @ R.T
        r = data.xipos[i] - cg_world
        inertia += I_body + m * (np.dot(r, r) * eye - np.outer(r, r))

    budget = p["inertia"]
    report = {
        "mass": total,
        "mass_budget": p["mass"]["total"],
        "mass_err_pct": 100 * (total - p["mass"]["total"]) / p["mass"]["total"],
        "cg": cg_body,
        "cg_norm": float(np.linalg.norm(cg_body)),
        "inertia": np.diag(inertia).copy(),
        "inertia_budget": np.array([budget["Ixx"], budget["Iyy"], budget["Izz"]]),
    }
    report["inertia_err_pct"] = 100 * (report["inertia"] - report["inertia_budget"]) / report["inertia_budget"]
    return report


def _gid(model, name):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid < 0:
        raise KeyError(f"geom not found: {name}")
    return gid


def _set_tilt(model, data, wing_angle, tail_angle):
    for jn, ang in (("j_nacelle_left", wing_angle),
                    ("j_nacelle_right", wing_angle),
                    ("j_tail_pod", tail_angle)):
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        data.qpos[model.jnt_qposadr[j]] = ang
    mujoco.mj_forward(model, data)


def clearance_sweep(model, data, n_wing=19, n_tail=7) -> dict:
    """Minimum prop-to-airframe distance and structural interpenetration
    across the full tilt envelope. Uses mj_geomDistance, which ignores
    collision filtering, so disabled-contact props are still measured."""
    p = load_params()
    prop_r = p["propulsion"]["prop_diameter"] / 2.0
    wing_max = np.deg2rad(p["tilt"]["wing_max_deg"])
    tail_max = np.deg2rad(p["tilt"]["tail_max_deg"])

    props = {n: _gid(model, n) for n in PROP_GEOMS}
    frame = {n: _gid(model, n) for n in AIRFRAME_GEOMS}
    struct = [(_gid(model, a), _gid(model, b), a, b) for a, b in STRUCT_PAIRS]

    fromto = np.zeros(6)
    min_clear = np.inf
    min_info = None
    min_struct = np.inf
    min_struct_info = None

    for wa in np.linspace(0, wing_max, n_wing):
        for ta in np.linspace(0, tail_max, n_tail):
            _set_tilt(model, data, wa, ta)
            for pname, pg in props.items():
                for fname, fg in frame.items():
                    d = mujoco.mj_geomDistance(model, data, pg, fg, 1.0, fromto)
                    if d < min_clear:
                        min_clear = d
                        min_info = (pname, fname, np.rad2deg(wa), np.rad2deg(ta))
            for ga, gb, na, nb in struct:
                d = mujoco.mj_geomDistance(model, data, ga, gb, 1.0, fromto)
                if d < min_struct:
                    min_struct = d
                    min_struct_info = (na, nb, np.rad2deg(wa), np.rad2deg(ta))

    _set_tilt(model, data, 0.0, 0.0)
    return {
        "prop_radius": prop_r,
        "clearance_target": 0.10 * prop_r,
        "min_prop_clearance": float(min_clear),
        "min_prop_info": min_info,
        "min_struct_distance": float(min_struct),
        "min_struct_info": min_struct_info,
    }
