#!/usr/bin/env python3
"""Human-readable model report: mass, CG, inertia, clearances vs budget."""
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from sim.model_check import load_model, mass_report, clearance_sweep  # noqa: E402


def main():
    model, data = load_model()
    print(f"model: nbody={model.nbody} njnt={model.njnt} nu={model.nu} "
          f"ngeom={model.ngeom}")

    r = mass_report(model, data)
    print("\n=== Mass / CG / Inertia ===")
    print(f"mass     {r['mass']:.4f} kg  (budget {r['mass_budget']})  "
          f"err {r['mass_err_pct']:+.2f}%  [limit 2%]")
    cg = r["cg"]
    print(f"CG       ({cg[0]*100:+.2f}, {cg[1]*100:+.2f}, {cg[2]*100:+.2f}) cm  "
          f"norm {r['cg_norm']*100:.2f} cm  [limit 1 cm]")
    for ax, i, b, e in zip("xyz", r["inertia"], r["inertia_budget"],
                           r["inertia_err_pct"]):
        print(f"I{ax}{ax}      {i:.4f} kg.m2 (budget {b})  err {e:+.1f}%  "
              f"[limit 10%]")

    c = clearance_sweep(model, data)
    print("\n=== Tilt-sweep clearances ===")
    print(f"prop radius          {c['prop_radius']*100:.2f} cm")
    print(f"min prop clearance   {c['min_prop_clearance']*100:.2f} cm  "
          f"[target >= {c['clearance_target']*100:.2f} cm]")
    print(f"  at: {c['min_prop_info']}")
    print(f"min struct distance  {c['min_struct_distance']*100:.2f} cm  "
          f"[must be > 0]")
    print(f"  at: {c['min_struct_info']}")

    ok = (abs(r["mass_err_pct"]) <= 2.0
          and r["cg_norm"] <= 0.01
          and all(abs(e) <= 10.0 for e in r["inertia_err_pct"])
          and c["min_prop_clearance"] >= c["clearance_target"]
          and c["min_struct_distance"] > 0.0)
    print("\nRESULT:", "ALL CHECKS PASS" if ok else "CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
