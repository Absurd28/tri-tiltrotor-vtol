"""Procedural obstacle fields, injected into the MJCF before compilation.

Obstacles are static worldbody geoms in GROUP 2 so the range-ray caster can
filter for them. difficulty in [0,1] scales count and tightness of the field
between the start area and the goal corridor.
"""
from __future__ import annotations

import numpy as np

GROUP = 2
RGBA = "0.75 0.35 0.3 1"


def obstacles_xml(difficulty: float, rng: np.random.Generator,
                  x_range=(8.0, 32.0), y_half=8.0) -> str:
    """Return <geom .../> lines for a field between x_range along +x."""
    difficulty = float(np.clip(difficulty, 0.0, 1.0))
    n = int(round(2 + 10 * difficulty))
    lines = []
    for i in range(n):
        kind = rng.choice(["cylinder", "sphere", "wall"])
        x = rng.uniform(*x_range)
        y = rng.uniform(-y_half, y_half)
        # all shapes span the 4-8 m flight band, or they are decoration the
        # horizontal range rays can never see
        if kind == "cylinder":
            r = rng.uniform(0.3, 0.8)
            h = rng.uniform(6.0, 10.0)
            lines.append(
                f'<geom name="obs{i}" type="cylinder" size="{r:.2f} {h/2:.2f}"'
                f' pos="{x:.2f} {y:.2f} {h/2:.2f}" group="{GROUP}"'
                f' rgba="{RGBA}"/>')
        elif kind == "sphere":
            r = rng.uniform(0.6, 1.3)
            z = rng.uniform(4.0, 7.5)
            lines.append(
                f'<geom name="obs{i}" type="sphere" size="{r:.2f}"'
                f' pos="{x:.2f} {y:.2f} {z:.2f}" group="{GROUP}"'
                f' rgba="{RGBA}"/>')
        else:  # wall segment
            w = rng.uniform(1.5, 3.5)
            h = rng.uniform(5.5, 9.0)
            lines.append(
                f'<geom name="obs{i}" type="box" size="0.15 {w/2:.2f} {h/2:.2f}"'
                f' pos="{x:.2f} {y:.2f} {h/2:.2f}" group="{GROUP}"'
                f' rgba="{RGBA}"/>')
    return "\n    ".join(lines)


def inject(xml: str, obstacle_block: str) -> str:
    assert "</worldbody>" in xml
    return xml.replace("</worldbody>",
                       f"    {obstacle_block}\n  </worldbody>")
