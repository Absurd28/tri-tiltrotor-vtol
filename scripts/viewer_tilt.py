#!/usr/bin/env python3
"""Interactive viewer with keyboard tilt control (needs a display / WSLg).

Keys:
  Q / A : left nacelle tilt  +5 / -5 deg
  W / S : right nacelle tilt +5 / -5 deg
  E / D : tail tilt          +5 / -5 deg
  Z     : all tilts to 0 (hover)
  X     : wing tilts to 90, tail 30 (cruise)
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "vtol.xml"))
data = mujoco.MjData(model)
key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "parked")
mujoco.mj_resetDataKeyframe(model, data, key)

STEP = np.deg2rad(5)
WING_MAX = np.deg2rad(90)
TAIL_MAX = np.deg2rad(30)


def key_cb(keycode):
    c = chr(keycode) if 0 <= keycode < 256 else ""
    if c == "Q":
        data.ctrl[0] = min(data.ctrl[0] + STEP, WING_MAX)
    elif c == "A":
        data.ctrl[0] = max(data.ctrl[0] - STEP, 0)
    elif c == "W":
        data.ctrl[1] = min(data.ctrl[1] + STEP, WING_MAX)
    elif c == "S":
        data.ctrl[1] = max(data.ctrl[1] - STEP, 0)
    elif c == "E":
        data.ctrl[2] = min(data.ctrl[2] + STEP, TAIL_MAX)
    elif c == "D":
        data.ctrl[2] = max(data.ctrl[2] - STEP, 0)
    elif c == "Z":
        data.ctrl[0:3] = 0
    elif c == "X":
        data.ctrl[0] = data.ctrl[1] = WING_MAX
        data.ctrl[2] = TAIL_MAX
    print(f"tilt L/R/T = {np.rad2deg(data.ctrl[0]):.0f} / "
          f"{np.rad2deg(data.ctrl[1]):.0f} / {np.rad2deg(data.ctrl[2]):.0f} deg")


print(__doc__)
with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
