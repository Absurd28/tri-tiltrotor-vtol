#!/usr/bin/env python3
"""Generate the MJCF model of the tilt-tri-rotor VTOL from params.yaml.

Design notes (viva):
- Every mass-budget line item is represented by one or more geoms with explicit
  mass, placed at its physical location. MuJoCo computes body inertia from the
  geoms, so the composite inertia emerges from the layout -- no hand-typed
  inertia tensor that could disagree with the geometry.
- The battery x-position is SOLVED so the x-CG lands on the origin (the battery
  is the balance weight, exactly as in real aircraft). The payload z-position
  is solved the same way for the z-CG, both clamped to their bay limits.
- Tilt convention: hinge about +y (spanwise). 0 rad = thrust up (hover),
  +pi/2 = thrust forward (cruise). R_y(theta) maps body +z to (sin,0,cos).
- Props are thin cylinders with collisions DISABLED (contype=0): real props are
  handled by the clearance check, not by contact physics.
- Elevator/rudder servos sit in the rear fuselage driving pushrods (D-07):
  putting 48 g at x=-0.55 would push the battery balance point out of its bay.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402

PI = 3.14159265358979


def solve_balance(components: list[dict], p: dict) -> tuple[float, float]:
    """Solve battery x and payload z so the CG sits at the origin.

    Both are independent single-variable linear solves:
        sum(m_i * x_i) + m_bat * x_bat = 0   ->  x_bat
        sum(m_i * z_i) + m_pay * z_pay = 0   ->  z_pay
    Clamped to the physical bay limits from params.yaml; a residual CG offset
    after clamping is reported by the caller.
    """
    lay = p["layout"]
    m_bat = p["mass"]["battery"]
    m_pay = p["mass"]["payload"]

    sum_mx = sum(c["mass"] * c["pos"][0] for c in components
                 if c["name"] not in ("battery", "payload"))
    sum_mz = sum(c["mass"] * c["pos"][2] for c in components
                 if c["name"] not in ("battery", "payload"))

    x_bat = -sum_mx / m_bat
    lo, hi = lay["battery_x_range"]
    x_bat_clamped = min(max(x_bat, lo), hi)

    # payload z solve includes the battery at its final position
    bat_z = lay["battery_z"]
    z_pay = -(sum_mz + m_bat * bat_z) / m_pay
    lo_z, hi_z = lay["payload_z_range"]
    z_pay_clamped = min(max(z_pay, lo_z), hi_z)

    return x_bat_clamped, z_pay_clamped


def component_table(p: dict) -> list[dict]:
    """All mass-budget items with absolute positions (tilt = 0).

    pos = component CG in body frame (origin = design CG).
    Entries for the tilting bodies carry parent="nacelle_left" etc.; their pos
    is still absolute (used for the balance solve), converted to local when the
    XML is assembled.
    """
    g = p["geometry"]
    lay = p["layout"]
    m = p["mass"]

    wz = lay["wing_z"]
    hub_z = lay["nacelle_hub_z"]
    boom_y = lay["boom_y"]
    rsx = lay["rear_servo_x"]
    mx, my = g["motor_left_x"], g["motor_left_y"]       # 0.20, 0.40
    tx = g["motor_tail_x"]                               # -0.40
    m_fc = lay["avionics_fc_mass"]
    m_av_nose = m["avionics_sensors"] - m_fc

    C = []  # noqa: N806

    def add(name, mass, pos, geom, parent="base", collide=True, group=0,
            rgba=None):
        C.append(dict(name=name, mass=mass, pos=pos, geom=geom, parent=parent,
                      collide=collide, group=group, rgba=rgba))

    # ---- fuselage structure (0.35 of fuselage_skids; 0.05 goes to skids) ----
    add("nose_box", 0.08, (0.35, 0, 0.01), ("box", 0.10, 0.04, 0.045))
    add("mid_box", 0.19, (0.05, 0, -0.005), ("box", 0.20, 0.055, 0.075))
    add("rear_box", 0.05, (-0.18, 0, 0.01), ("box", 0.03, 0.045, 0.05))
    # Notch front wall: its notch-facing (rear) surface must sit exactly
    # 0.19 m ahead of the tail hub (x = -0.21), per the design spec.
    add("bulkhead", 0.03, (-0.198, 0, 0.02), ("box", 0.012, 0.17, 0.06))

    # ---- payload and battery (positions solved later) ----
    add("payload", m["payload"], (0.0, 0, -0.03), ("box", 0.05, 0.04, 0.03),
        group=3, rgba="0.9 0.5 0.1 1")
    add("battery", m["battery"], (0.075, 0, lay["battery_z"]),
        ("box", 0.075, 0.025, 0.025), group=3, rgba="0.2 0.7 0.3 1")

    # ---- avionics: FC+IMU at CG, the rest in the nose bay (D-08) ----
    add("avionics_fc", m_fc, (0.0, 0, 0.01), ("box", 0.03, 0.03, 0.012),
        group=3, rgba="0.2 0.3 0.8 1")
    add("avionics_nose", m_av_nose, (lay["avionics_nose_x"], 0, 0.0),
        ("box", 0.045, 0.035, 0.035), group=3, rgba="0.2 0.3 0.8 1")

    # ---- wing: main box + fixed TE centre strip; ailerons hinged (0.48) ----
    add("wing_main", 0.41, (-0.015, 0, wz), ("box", 0.085, 0.80, 0.012))
    add("wing_te_center", 0.04, (-0.125, 0, wz), ("box", 0.025, 0.40, 0.008))
    add("aileron_left", 0.015, (-0.125, 0.60, wz),
        ("box", 0.025, 0.18, 0.006), parent="aileron_left")
    add("aileron_right", 0.015, (-0.125, -0.60, wz),
        ("box", 0.025, 0.18, 0.006), parent="aileron_right")

    # ---- booms, fins, stabs (0.25 incl. hinged rudders/elevators) ----
    add("boom_left", 0.08, (-0.385, boom_y, 0), ("box", 0.235, 0.015, 0.015))
    add("boom_right", 0.08, (-0.385, -boom_y, 0), ("box", 0.235, 0.015, 0.015))
    add("fin_left", 0.014, (-0.56, boom_y, 0.10), ("box", 0.06, 0.004, 0.085))
    add("fin_right", 0.014, (-0.56, -boom_y, 0.10), ("box", 0.06, 0.004, 0.085))
    add("rudder_left", 0.006, (-0.64, boom_y, 0.10),
        ("box", 0.02, 0.004, 0.085), parent="rudder_left")
    add("rudder_right", 0.006, (-0.64, -boom_y, 0.10),
        ("box", 0.02, 0.004, 0.085), parent="rudder_right")
    add("stab_left", 0.017, (-0.53, 0.2975, 0.02), ("box", 0.08, 0.0875, 0.004))
    add("stab_right", 0.017, (-0.53, -0.2975, 0.02), ("box", 0.08, 0.0875, 0.004))
    add("elevator_left", 0.008, (-0.63, 0.2975, 0.02),
        ("box", 0.02, 0.0875, 0.004), parent="elevator_left")
    add("elevator_right", 0.008, (-0.63, -0.2975, 0.02),
        ("box", 0.02, 0.0875, 0.004), parent="elevator_right")

    # ---- skids (0.05 of fuselage_skids) ----
    skid_z = lay["skid_z"]
    add("skid_left", 0.015, (-0.375, boom_y, skid_z),
        ("capsule_x", 0.175, 0.012))
    add("skid_right", 0.015, (-0.375, -boom_y, skid_z),
        ("capsule_x", 0.175, 0.012))
    for sx, tag in ((-0.25, "f"), (-0.50, "b")):
        for sy, side in ((boom_y, "l"), (-boom_y, "r")):
            add(f"strut_{side}{tag}", 0.0025, (sx, sy, (skid_z - 0.015) / 2),
                ("capsule_z", (abs(skid_z) - 0.015) / 2, 0.005))
    add("skid_nose", 0.010, (0.25, 0, -0.105), ("capsule_x", 0.05, 0.010))

    # ---- ESCs (0.09) ----
    add("esc_left", 0.03, (0.09, my, hub_z + 0.015), ("box", 0.015, 0.012, 0.006),
        group=3)
    add("esc_right", 0.03, (0.09, -my, hub_z + 0.015), ("box", 0.015, 0.012, 0.006),
        group=3)
    add("esc_tail", 0.03, (rsx, 0, 0.055), ("box", 0.015, 0.012, 0.006), group=3)

    # ---- tilt servos (0.135) ----
    add("tiltservo_left", 0.045, (0.13, my, hub_z + 0.015),
        ("box", 0.015, 0.010, 0.015), group=3)
    add("tiltservo_right", 0.045, (0.13, -my, hub_z + 0.015),
        ("box", 0.015, 0.010, 0.015), group=3)
    add("tiltservo_tail", 0.045, (-0.17, 0, 0.02),
        ("box", 0.015, 0.010, 0.015), group=3)

    # ---- surface servos (0.072); elev/rud in fuselage with pushrods D-07 ----
    add("ailservo_left", 0.012, (-0.08, 0.55, wz + 0.005),
        ("box", 0.010, 0.010, 0.008), group=3)
    add("ailservo_right", 0.012, (-0.08, -0.55, wz + 0.005),
        ("box", 0.010, 0.010, 0.008), group=3)
    add("elevservo_left", 0.012, (rsx, 0.05, 0.0),
        ("box", 0.010, 0.010, 0.008), group=3)
    add("elevservo_right", 0.012, (rsx, -0.05, 0.0),
        ("box", 0.010, 0.010, 0.008), group=3)
    add("rudservo_left", 0.012, (rsx, 0.08, 0.035),
        ("box", 0.010, 0.010, 0.008), group=3)
    add("rudservo_right", 0.012, (rsx, -0.08, 0.035),
        ("box", 0.010, 0.010, 0.008), group=3)

    # ---- tail axle + linkage (0.07): axle tilts with the pod ----
    add("tail_axle", 0.04, (tx, 0, lay["tail_hub_z"]),
        ("capsule_y", 0.19, 0.006), parent="tail_pod")
    add("linkage", 0.03, (-0.30, 0, 0.0), ("capsule_x", 0.09, 0.004), group=3)

    # ---- nacelle arms (nacelle_hardware 0.08) ----
    add("arm_left", 0.04, (0.1275, my, hub_z), ("capsule_x", 0.0575, 0.012))
    add("arm_right", 0.04, (0.1275, -my, hub_z), ("capsule_x", 0.0575, 0.012))

    # ---- motors and props (tilting bodies) ----
    po_w = lay["prop_offset_wing"]
    po_t = lay["prop_offset_tail"]
    prop_r = p["propulsion"]["prop_diameter"] / 2.0
    for side, sy in (("left", my), ("right", -my)):
        add(f"motor_{side}", 0.14, (mx, sy, hub_z + 0.02),
            ("cylinder", 0.021, 0.02), parent=f"nacelle_{side}",
            rgba="0.15 0.15 0.15 1")
        add(f"prop_{side}", 0.02, (mx, sy, hub_z + po_w),
            ("cylinder", prop_r, 0.004), parent=f"nacelle_{side}",
            collide=False, group=1, rgba="0.1 0.1 0.1 0.45")
    add("motor_tail", 0.14, (tx, 0, lay["tail_hub_z"] + 0.025),
        ("cylinder", 0.021, 0.02), parent="tail_pod", rgba="0.15 0.15 0.15 1")
    add("prop_tail", 0.02, (tx, 0, lay["tail_hub_z"] + po_t),
        ("cylinder", prop_r, 0.004), parent="tail_pod",
        collide=False, group=1, rgba="0.1 0.1 0.1 0.45")

    return C


# Hinge-body definitions: name -> (body origin xyz, hinge axis, range lo, hi)
def hinge_bodies(p):
    g = p["geometry"]
    lay = p["layout"]
    wz = lay["wing_z"]
    boom_y = lay["boom_y"]
    my = g["motor_left_y"]
    sr = p["surfaces"]
    a_ail = sr["aileron_range_deg"] * PI / 180
    a_ele = sr["elevator_range_deg"] * PI / 180
    a_rud = sr["rudder_range_deg"] * PI / 180
    t = p["tilt"]
    return {
        "nacelle_left": ((g["motor_left_x"], my, lay["nacelle_hub_z"]),
                         "0 1 0", 0.0, t["wing_max_deg"] * PI / 180),
        "nacelle_right": ((g["motor_right_x"], -my, lay["nacelle_hub_z"]),
                          "0 1 0", 0.0, t["wing_max_deg"] * PI / 180),
        "tail_pod": ((g["motor_tail_x"], 0, lay["tail_hub_z"]),
                     "0 1 0", 0.0, t["tail_max_deg"] * PI / 180),
        "aileron_left": ((-0.10, 0.60, wz), "0 1 0", -a_ail, a_ail),
        "aileron_right": ((-0.10, -0.60, wz), "0 1 0", -a_ail, a_ail),
        "elevator_left": ((-0.61, 0.2975, 0.02), "0 1 0", -a_ele, a_ele),
        "elevator_right": ((-0.61, -0.2975, 0.02), "0 1 0", -a_ele, a_ele),
        "rudder_left": ((-0.62, boom_y, 0.10), "0 0 1", -a_rud, a_rud),
        "rudder_right": ((-0.62, -boom_y, 0.10), "0 0 1", -a_rud, a_rud),
    }


def geom_xml(c, body_origin=(0, 0, 0)):
    """Emit one <geom> element; pos is relative to the owning body."""
    px = c["pos"][0] - body_origin[0]
    py = c["pos"][1] - body_origin[1]
    pz = c["pos"][2] - body_origin[2]
    kind = c["geom"][0]
    rgba = f' rgba="{c["rgba"]}"' if c["rgba"] else ""
    coll = "" if c["collide"] else ' contype="0" conaffinity="0"'
    grp = f' group="{c["group"]}"' if c["group"] else ""
    common = (f'name="{c["name"]}" mass="{c["mass"]}"'
              f' pos="{px:.4f} {py:.4f} {pz:.4f}"{rgba}{coll}{grp}')
    if kind == "box":
        _, hx, hy, hz = c["geom"]
        return f'<geom {common} type="box" size="{hx} {hy} {hz}"/>'
    if kind == "cylinder":
        _, r, hh = c["geom"]
        return f'<geom {common} type="cylinder" size="{r} {hh}"/>'
    if kind in ("capsule_x", "capsule_y", "capsule_z"):
        _, hl, r = c["geom"]
        ax = {"capsule_x": (1, 0, 0), "capsule_y": (0, 1, 0),
              "capsule_z": (0, 0, 1)}[kind]
        f1 = [px - ax[0] * hl, py - ax[1] * hl, pz - ax[2] * hl]
        f2 = [px + ax[0] * hl, py + ax[1] * hl, pz + ax[2] * hl]
        fr = " ".join(f"{v:.4f}" for v in f1 + f2)
        return (f'<geom name="{c["name"]}" mass="{c["mass"]}" type="capsule"'
                f' fromto="{fr}" size="{r}"{rgba}{coll}{grp}/>')
    raise ValueError(kind)


def build(write_info: bool = True) -> str:
    p = load_params()
    comps = component_table(p)

    # --- balance solve: battery x, payload z ---
    x_bat, z_pay = solve_balance(comps, p)
    for c in comps:
        if c["name"] == "battery":
            c["pos"] = (x_bat, 0, p["layout"]["battery_z"])
        if c["name"] == "payload":
            c["pos"] = (0.0, 0, z_pay)

    total = sum(c["mass"] for c in comps)
    cgx = sum(c["mass"] * c["pos"][0] for c in comps) / total
    cgy = sum(c["mass"] * c["pos"][1] for c in comps) / total
    cgz = sum(c["mass"] * c["pos"][2] for c in comps) / total

    hb = hinge_bodies(p)
    lay = p["layout"]
    g = p["geometry"]
    po_w = lay["prop_offset_wing"]
    po_t = lay["prop_offset_tail"]

    by_parent: dict[str, list] = {}
    for c in comps:
        by_parent.setdefault(c["parent"], []).append(c)

    def body_block(name, kp_pos=None):
        origin, axis, lo, hi = hb[name]
        damping = 0.2 if name.startswith(("nacelle", "tail")) else 0.02
        lines = [f'<body name="{name}" pos="{origin[0]} {origin[1]} {origin[2]}">',
                 f'  <joint name="j_{name}" type="hinge" axis="{axis}"'
                 f' range="{lo:.5f} {hi:.5f}" damping="{damping}" armature="0.001"/>']
        for c in by_parent.get(name, []):
            lines.append("  " + geom_xml(c, origin))
        if name == "nacelle_left":
            lines.append(f'  <site name="thrust_left" pos="0 0 {po_w}" size="0.008"/>')
        if name == "nacelle_right":
            lines.append(f'  <site name="thrust_right" pos="0 0 {po_w}" size="0.008"/>')
        if name == "tail_pod":
            lines.append(f'  <site name="thrust_tail" pos="0 0 {po_t}" size="0.008"/>')
        lines.append("</body>")
        return "\n        ".join(lines)

    base_geoms = "\n        ".join(
        geom_xml(c) for c in by_parent.get("base", []))
    tilt_bodies = "\n        ".join(
        body_block(n) for n in ("nacelle_left", "nacelle_right", "tail_pod"))
    surf_bodies = "\n        ".join(
        body_block(n) for n in ("aileron_left", "aileron_right",
                                "elevator_left", "elevator_right",
                                "rudder_left", "rudder_right"))

    t = p["tilt"]
    wing_max = t["wing_max_deg"] * PI / 180
    tail_max = t["tail_max_deg"] * PI / 180
    nose_x = g["nose_x"]

    # parked root height: skid bottom at skid_z - skid radius
    park_z = -(lay["skid_z"] - 0.012)

    xml = f"""<mujoco model="vtol_tilt_trirotor">
  <!-- GENERATED by model/build_mjcf.py; edit params.yaml, not this file.
       Solved balance: battery_x={x_bat:.4f} m, payload_z={z_pay:.4f} m
       Analytic point-mass CG: ({cgx:.4f}, {cgy:.4f}, {cgz:.4f}) m -->
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{p['simulation']['physics_dt']}" integrator="implicitfast"
          density="0" viscosity="0"/>

  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
    <quality shadowsize="4096"/>
    <global offwidth="1280" offheight="960"/>
  </visual>

  <asset>
    <texture type="2d" name="grid" builtin="checker" width="256" height="256"
             rgb1="0.22 0.26 0.30" rgb2="0.30 0.34 0.38"/>
    <material name="grid" texture="grid" texrepeat="12 12" reflectance="0.1"/>
  </asset>

  <default>
    <geom rgba="0.82 0.83 0.86 1" friction="0.8 0.005 0.0001"/>
    <position forcerange="-5 5"/>
  </default>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.7 0.7 0.7"/>
    <geom name="ground" type="plane" size="12 12 0.1" material="grid"/>
    <body name="base" pos="0 0 {park_z:.3f}">
      <freejoint name="root"/>
      <site name="imu" pos="0 0 0" size="0.006"/>
      <site name="pitot" pos="{nose_x} 0 0.03" size="0.005"/>
      <site name="range_fwd" pos="{nose_x} 0 0" zaxis="1 0 0" size="0.005"/>
      <site name="range_down" pos="0 0 -0.08" zaxis="0 0 -1" size="0.005"/>
        {base_geoms}
        {tilt_bodies}
        {surf_bodies}
    </body>
  </worldbody>

  <actuator>
    <position name="tilt_left" joint="j_nacelle_left" kp="10" kv="1"
              ctrlrange="0 {wing_max:.5f}"/>
    <position name="tilt_right" joint="j_nacelle_right" kp="10" kv="1"
              ctrlrange="0 {wing_max:.5f}"/>
    <position name="tilt_tail" joint="j_tail_pod" kp="10" kv="1"
              ctrlrange="0 {tail_max:.5f}"/>
    <position name="aileron_left" joint="j_aileron_left" kp="2" kv="0.05"
              forcerange="-1 1"/>
    <position name="aileron_right" joint="j_aileron_right" kp="2" kv="0.05"
              forcerange="-1 1"/>
    <position name="elevator_left" joint="j_elevator_left" kp="2" kv="0.05"
              forcerange="-1 1"/>
    <position name="elevator_right" joint="j_elevator_right" kp="2" kv="0.05"
              forcerange="-1 1"/>
    <position name="rudder_left" joint="j_rudder_left" kp="2" kv="0.05"
              forcerange="-1 1"/>
    <position name="rudder_right" joint="j_rudder_right" kp="2" kv="0.05"
              forcerange="-1 1"/>
  </actuator>

  <sensor>
    <accelerometer name="imu_acc" site="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <velocimeter name="imu_vel" site="imu"/>
    <framequat name="att_quat" objtype="site" objname="imu"/>
    <rangefinder name="range_fwd" site="range_fwd"/>
    <rangefinder name="range_down" site="range_down"/>
  </sensor>

  <keyframe>
    <key name="parked" qpos="0 0 {park_z:.3f} 1 0 0 0  0 0 0  0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
"""
    out = ROOT / "model" / "vtol.xml"
    out.write_text(xml, encoding="utf-8")

    if write_info:
        info = (f"# Generated by build_mjcf.py\n"
                f"battery_x: {x_bat:.4f}\n"
                f"payload_z: {z_pay:.4f}\n"
                f"analytic_mass: {total:.4f}\n"
                f"analytic_cg: [{cgx:.5f}, {cgy:.5f}, {cgz:.5f}]\n")
        (ROOT / "model" / "model_info.yaml").write_text(info, encoding="utf-8")

    print(f"Wrote {out}")
    print(f"  point-mass total = {total:.4f} kg (budget {p['mass']['total']})")
    print(f"  solved battery_x = {x_bat:.4f} m  (range {lay['battery_x_range']})")
    print(f"  solved payload_z = {z_pay:.4f} m  (range {lay['payload_z_range']})")
    print(f"  analytic CG      = ({cgx:.4f}, {cgy:.4f}, {cgz:.4f}) m")
    return xml


if __name__ == "__main__":
    build()
