#!/usr/bin/env python3
"""Phase 0 smoke test: verifies imports, params loader, MuJoCo model load."""
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))


def test_imports():
    import numpy as np
    import scipy
    import matplotlib
    import mujoco
    import gymnasium
    print(f"  numpy {np.__version__} | scipy {scipy.__version__} | "
          f"mujoco {mujoco.__version__} | gymnasium {gymnasium.__version__}")


def test_params_loader():
    from params import load_params, get_param
    p = load_params()
    total_mass = get_param("mass.total")
    assert abs(total_mass - 3.527) < 1e-6, f"Wrong mass: {total_mass}"
    sim_dt = get_param("simulation.physics_dt")
    assert abs(sim_dt - 0.002) < 1e-9, f"Wrong dt: {sim_dt}"
    print(f"  total_mass={total_mass} kg, physics_dt={sim_dt} s")


def test_mujoco_load():
    import mujoco
    xml_path = ROOT / "model" / "smoke_test.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    assert model.nbody >= 2, "Expected at least 2 bodies"
    mujoco.mj_step(model, data)
    print(f"  model bodies={model.nbody}, nq={model.nq}, nu={model.nu}")
    print(f"  one step OK, time={data.time:.4f} s")


def test_torch():
    try:
        import torch
        cuda_ok = torch.cuda.is_available()
        device = torch.cuda.get_device_name(0) if cuda_ok else "CPU"
        x = torch.ones(3, 3)
        print(f"  torch {torch.__version__} | CUDA: {cuda_ok} ({device})")
    except ImportError:
        print("  torch not installed yet (Phase 0 acceptable)")


if __name__ == "__main__":
    tests = [test_imports, test_params_loader, test_mujoco_load, test_torch]
    failures = []
    for t in tests:
        try:
            print(f"[RUN] {t.__name__}")
            t()
            print(f"[PASS] {t.__name__}")
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failures.append(t.__name__)
    print()
    if failures:
        print(f"FAILED: {failures}")
        sys.exit(1)
    else:
        print("ALL SMOKE TESTS PASSED")