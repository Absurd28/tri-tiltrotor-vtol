#!/usr/bin/env python3
"""Offscreen renders of the model: top / side / bottom / front / iso,
plus iso views at 45 and 90 degree wing tilt. Output: docs/renders/*.png
Set MUJOCO_GL=egl (or osmesa) when running headless.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from sim.model_check import load_model, _set_tilt  # noqa: E402

OUT = ROOT / "docs" / "renders"
OUT.mkdir(parents=True, exist_ok=True)

# (name, azimuth, elevation, wing_tilt_deg, tail_tilt_deg)
VIEWS = [
    ("top", 90, -89, 0, 0),
    ("side", 90, 0, 0, 0),
    ("bottom", 90, 89, 0, 0),
    ("front", 180, -10, 0, 0),
    ("iso_hover", 135, -25, 0, 0),
    ("iso_mid", 135, -25, 45, 15),
    ("iso_cruise", 135, -25, 90, 30),
]


def main():
    model, data = load_model()
    renderer = mujoco.Renderer(model, height=960, width=1280)
    cam = mujoco.MjvCamera()
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")

    for name, az, el, wing_deg, tail_deg in VIEWS:
        _set_tilt(model, data, np.deg2rad(wing_deg), np.deg2rad(tail_deg))
        cam.lookat[:] = data.xpos[base] + [0, 0, 0.02]
        cam.distance = 2.1
        cam.azimuth = az
        cam.elevation = el
        renderer.update_scene(data, camera=cam)
        px = renderer.render()
        if px.std() < 1.0:
            print(f"WARNING: {name} render looks blank (std={px.std():.2f})")
        path = OUT / f"view_{name}.png"
        plt.imsave(path, px)
        print(f"saved {path.relative_to(ROOT)}")

    renderer.close()
    print("DONE")


if __name__ == "__main__":
    main()
