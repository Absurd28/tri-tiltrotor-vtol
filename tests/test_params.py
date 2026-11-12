"""Tests for the params module."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from params import load_params, get_param


def test_load_returns_dict():
    p = load_params()
    assert isinstance(p, dict)


def test_total_mass():
    total = get_param("mass.total")
    assert abs(total - 3.527) < 1e-6, f"Expected 3.527, got {total}"


def test_mass_budget_sums():
    p = load_params()
    m = p["mass"]
    components = [
        "payload", "battery", "wing", "fuselage_skids", "booms_fins_stab",
        "motors", "props", "escs", "tilt_servos", "surface_servos",
        "nacelle_hardware", "tail_axle_linkage", "avionics_sensors",
    ]
    total_components = sum(m[k] for k in components)
    assert abs(total_components - m["total"]) < 0.001, (
        f"Mass budget doesn't sum: components={total_components:.3f}, total={m['total']}"
    )


def test_physics_dt():
    dt = get_param("simulation.physics_dt")
    assert dt == 0.002, f"Expected 0.002, got {dt}"
    hz = 1.0 / dt
    assert hz == 500.0


def test_tilt_range():
    wing_max = get_param("tilt.wing_max_deg")
    tail_max = get_param("tilt.tail_max_deg")
    assert wing_max == 90.0
    assert tail_max == 30.0


def test_missing_key_returns_default():
    result = get_param("nonexistent.key", default=42)
    assert result == 42


def test_inertia_diagonal_positive():
    p = load_params()
    inertia = p["inertia"]
    for axis in ["Ixx", "Iyy", "Izz"]:
        assert inertia[axis] > 0, f"{axis} must be positive"


def test_wing_area_consistent():
    span = get_param("geometry.wing_span")
    chord = get_param("geometry.wing_chord")
    area = get_param("geometry.wing_area")
    computed = span * chord
    assert abs(computed - area) < 0.001, (
        f"wing_area mismatch: span*chord={computed:.4f}, area={area}"
    )