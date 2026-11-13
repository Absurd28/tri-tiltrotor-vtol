#!/usr/bin/env python3
"""Render one seeded episode of a policy (or the baseline) to MP4.

Usage (MUJOCO_GL=egl):
  python rl/render_episode.py --task obstacle --policy runs/s5_obstacle/final.zip \
      --seed 10003 --out docs/videos/rl_obstacle.mp4
  python rl/render_episode.py --task obstacle --baseline --seed 10003 --out ...
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from rl.env import VTOLEnv  # noqa: E402

FFMPEG = "/home/code_hack_absurd/miniconda3/envs/drone311/bin/ffmpeg"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--policy", default=None)
    ap.add_argument("--baseline", action="store_true")
    ap.add_argument("--seed", type=int, default=10_000)
    ap.add_argument("--difficulty", type=float, default=0.4)
    ap.add_argument("--dr", action="store_true")
    ap.add_argument("--mode", default="B")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import mujoco
    import matplotlib.pyplot as plt

    policy = None
    if not args.baseline:
        from stable_baselines3 import PPO
        policy = PPO.load(args.policy, device="cpu")

    env = VTOLEnv(task=args.task, action_mode=args.mode, dr=args.dr,
                  difficulty=args.difficulty, seed=99)
    obs, _ = env.reset(seed=args.seed)
    renderer = mujoco.Renderer(env.veh.model, height=480, width=640)
    cam = mujoco.MjvCamera()
    frames, done, trunc, info = [], False, False, {}
    while not (done or trunc):
        a = (np.zeros(env.action_space.shape, np.float32) if policy is None
             else policy.predict(obs, deterministic=True)[0])
        obs, _, done, trunc, info = env.step(a)
        if env.steps % 2 == 0:                       # 25 fps
            s = env.veh.state()
            cam.lookat[:] = s["pos"]
            cam.distance, cam.azimuth, cam.elevation = 9.0, 210, -25
            renderer.update_scene(env.veh.data, camera=cam)
            frames.append(renderer.render().copy())

    tmp = Path(tempfile.mkdtemp())
    for i, fr in enumerate(frames):
        plt.imsave(tmp / f"f{i:05d}.png", fr)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-framerate", "25",
                    "-i", str(tmp / "f%05d.png"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", str(out)], check=True)
    print(f"wrote {out} | steps {env.steps} | success {info.get('is_success')}"
          f" | collision {info.get('collision', False)}")


if __name__ == "__main__":
    main()
