"""Electrical design acceptance: margins and mass budget (hardware/)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from hardware.power_budget import budget  # noqa: E402


def test_esc_servo_battery_margins():
    b = budget()
    assert b["esc_margin"] >= 1.5, b["esc_margin"]
    assert b["servo_bec_margin"] >= 1.5
    assert b["battery_margin"] >= 2.0


def test_thrust_to_weight_and_endurance():
    b = budget()
    assert b["thrust_to_weight"] >= 1.6
    assert b["hover_time_min"] > 5.0        # enough for 2 takeoffs + 2 landings
    assert b["cruise_time_min"] > 20.0


def test_bom_mass_matches_airframe_budget():
    """Every BOM budget line within 10 % of params.yaml (Phase 1 tolerance)."""
    for line, (got, want) in budget()["mass_lines"].items():
        assert abs(got - want) / want <= 0.10, (line, got, want)


def test_actuator_count_matches_sim_action_space():
    import csv
    rows = list(csv.DictReader(open(Path(__file__).parent.parent / "hardware" / "bom.csv",
                                    encoding="utf-8")))
    qty = {r["budget_line"]: 0 for r in rows}
    for r in rows:
        qty[r["budget_line"]] += int(r["qty"])
    assert qty["motors"] == 3 and qty["escs"] == 3
    assert qty["tilt_servos"] == 3 and qty["surface_servos"] == 6
