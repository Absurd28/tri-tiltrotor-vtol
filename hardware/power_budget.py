#!/usr/bin/env python3
"""Electrical power budget for the VTOL, derived from params/params.yaml and
hardware/bom.csv. Writes hardware/power_budget.md (tables used in the
README) and returns a dict for tests/test_hardware.py.

Method (viva):
  hover power per motor  P = T^1.5 / sqrt(2 rho A) / (FM * eta_motor)
      momentum theory ideal power, divided by a figure of merit FM (prop
      efficiency vs ideal, 0.6 for a hobby 12" prop) and motor/ESC efficiency
  cruise power           P = (W / (L/D)) * V / (eta_prop * eta_motor)
  current                I = P / V_nominal, flight time = 0.8 * Wh / P
      (80 % usable capacity: LiPo reserve)
All results are ESTIMATES until bench thrust-stand tests (Phase 9).
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402

FM = 0.60            # [assumed] figure of merit, 12" fixed-pitch prop
ETA_PROP_CRUISE = 0.60
USABLE = 0.80        # LiPo usable fraction (reserve)
LD_CRUISE = 8.0      # Phase 2 acceptance value at 17 m/s
AVIONICS_W = 9.0     # FC 2.5 W + GPS 0.5 + Pi Zero 2 W 3 + radios 1.5 + cam/VTX 1.5
SERVO_HOVER_A = 1.5  # 10 servos mostly holding, at 7.4 V
SERVO_PEAK_A = 6.0   # all tilt servos slewing + surfaces (stall ~1.5 A each tilt)

# copper resistance-based gauge choice for <= ~3 % voltage drop at 0.3 m
AWG = [(10, 60.0), (12, 41.0), (14, 32.0), (16, 22.0), (18, 16.0), (20, 11.0)]


def read_bom():
    with open(ROOT / "hardware" / "bom.csv", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def budget():
    p = load_params()
    pr, g = p["propulsion"], 9.81
    rho = p["simulation"]["air_density"]
    A = math.pi * (pr["prop_diameter"] / 2) ** 2
    eta_m = pr["motor_efficiency"]
    V = pr["battery_voltage_nominal"]
    Wh = pr["battery_capacity_wh"]
    W = p["mass"]["total"] * g

    def motor_power(T):
        return T ** 1.5 / math.sqrt(2 * rho * A) / (FM * eta_m)

    T_h = pr["hover_thrust_per_motor"]
    T_max = pr["max_thrust_per_motor"]
    # hover: wing motors carry 2*T_h, tail the rest (equal split by design)
    P_hover = 3 * motor_power(W / 3) + AVIONICS_W + SERVO_HOVER_A * 7.4
    P_motor_max = motor_power(T_max)
    I_motor_max = P_motor_max / (V * 0.95)       # sagged pack voltage
    v_cruise = p["aero"]["cruise_speed"] if "cruise_speed" in p["aero"] else 17.0
    D = W / LD_CRUISE
    P_cruise = D * v_cruise / (ETA_PROP_CRUISE * eta_m) + AVIONICS_W + 0.5 * 7.4
    P_peak = 3 * P_motor_max + AVIONICS_W + SERVO_PEAK_A * 7.4

    out = dict(
        hover_power_W=P_hover, hover_current_A=P_hover / V,
        hover_time_min=USABLE * Wh / P_hover * 60,
        cruise_power_W=P_cruise, cruise_current_A=P_cruise / V,
        cruise_time_min=USABLE * Wh / P_cruise * 60,
        cruise_range_km=USABLE * Wh / P_cruise * 3600 * v_cruise / 1000,
        motor_max_power_W=P_motor_max, motor_max_current_A=I_motor_max,
        peak_current_A=P_peak / V,
        thrust_to_weight=3 * T_max / W,
        hover_thrust_check_N=T_h,
    )
    bom = read_bom()
    esc = next(r for r in bom if r["budget_line"] == "escs")
    out["esc_rating_A"] = float(esc["item"].split()[1].rstrip("A"))
    out["esc_margin"] = out["esc_rating_A"] / I_motor_max
    out["servo_bec_A"] = 10.0
    out["servo_bec_margin"] = out["servo_bec_A"] / SERVO_PEAK_A
    out["battery_burst_A"] = 5.0 * 45                # 5 Ah * 45 C
    out["battery_margin"] = out["battery_burst_A"] / out["peak_current_A"]

    def gauge(I):
        return next((a for a, cap in reversed(AWG) if cap >= I * 1.25), 10)
    out["awg_battery"] = gauge(out["peak_current_A"])
    out["awg_esc"] = gauge(I_motor_max)

    # mass check against the params budget lines
    lines = {}
    for r in bom:
        lines.setdefault(r["budget_line"], 0.0)
        lines[r["budget_line"]] += int(r["qty"]) * float(r["unit_mass_g"]) / 1000
    out["mass_lines"] = {k: (v, p["mass"][k]) for k, v in lines.items()}
    return out


def write_md(b):
    rows = [
        ("Hover (all 3 motors + avionics + servos)", f"{b['hover_power_W']:.0f} W",
         f"{b['hover_current_A']:.1f} A", f"{b['hover_time_min']:.1f} min"),
        ("Cruise 17 m/s (wing motors, L/D 8)", f"{b['cruise_power_W']:.0f} W",
         f"{b['cruise_current_A']:.1f} A", f"{b['cruise_time_min']:.0f} min "
         f"(~{b['cruise_range_km']:.0f} km)"),
        ("One motor at max thrust (2.1 kgf)", f"{b['motor_max_power_W']:.0f} W",
         f"{b['motor_max_current_A']:.1f} A", "—"),
        ("Peak: 3 motors max + servos slewing", f"{b['peak_current_A'] * 22.2:.0f} W",
         f"{b['peak_current_A']:.1f} A", "—"),
    ]
    md = ["| Condition | Power | Battery current | Endurance (80 % usable) |",
          "|---|---|---|---|"]
    md += [f"| {a} | {b_} | {c} | {d} |" for a, b_, c, d in rows]
    md += ["", "| Check | Value | Requirement | Result |", "|---|---|---|---|"]
    checks = [
        ("ESC rating / max motor current", f"{b['esc_rating_A']:.0f} A / {b['motor_max_current_A']:.1f} A",
         "≥ 1.5×", b["esc_margin"] >= 1.5, f"{b['esc_margin']:.2f}×"),
        ("Servo UBEC / peak servo current", f"{b['servo_bec_A']:.0f} A / {SERVO_PEAK_A:.0f} A",
         "≥ 1.5×", b["servo_bec_margin"] >= 1.5, f"{b['servo_bec_margin']:.2f}×"),
        ("Battery burst (45C) / peak current", f"{b['battery_burst_A']:.0f} A / {b['peak_current_A']:.0f} A",
         "≥ 2×", b["battery_margin"] >= 2, f"{b['battery_margin']:.1f}×"),
        ("Thrust-to-weight", f"{b['thrust_to_weight']:.2f}", "≥ 1.6 for VTOL", b["thrust_to_weight"] >= 1.6, ""),
    ]
    md += [f"| {n} | {v} | {req} | {'✓' if ok else '✗'} {m} |" for n, v, req, ok, m in checks]
    md += ["", f"Wire gauge: battery/PM **{b['awg_battery']} AWG**, ESC leads **{b['awg_esc']} AWG** "
           "(≥ 25 % ampacity margin).", "",
           "| Mass budget line | BOM total | params.yaml budget | Δ |", "|---|---|---|---|"]
    for k, (got, want) in b["mass_lines"].items():
        md.append(f"| {k} | {got * 1000:.0f} g | {want * 1000:.0f} g | {100 * (got - want) / want:+.1f} % |")
    (ROOT / "hardware" / "power_budget.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    return "\n".join(md)


if __name__ == "__main__":
    print(write_md(budget()))
