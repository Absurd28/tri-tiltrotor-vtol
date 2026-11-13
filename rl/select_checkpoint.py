#!/usr/bin/env python3
"""Pick the best checkpoint of a run on VALIDATION seeds (20000+), disjoint
from the TEST seeds (10000+) used by rl/evaluate.py, and copy it to
runs/<name>/best.zip. Writes runs/<name>/selection.csv.

Usage: python rl/select_checkpoint.py --name s1_hover --task hover -n 20
Selection metric: success rate, ties broken by mean reward (D-17).
"""
from __future__ import annotations

import argparse
import csv
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from rl.env import VTOLEnv  # noqa: E402
from rl.evaluate import run_episodes  # noqa: E402

VAL_SEED = 20_000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--task", required=True)
    ap.add_argument("-n", type=int, default=20)
    ap.add_argument("--difficulty", type=float, default=0.3)
    args = ap.parse_args()
    import torch
    torch.set_num_threads(1)
    from stable_baselines3 import PPO

    d = ROOT / "runs" / args.name
    cands = sorted(d.glob("ckpt_*_steps.zip"), key=lambda p: int(p.stem.split("_")[1]))
    if (d / "final.zip").exists():
        cands.append(d / "final.zip")
    env = VTOLEnv(task=args.task, action_mode="B", dr=True,
                  difficulty=args.difficulty, seed=99)
    rows = []
    base = run_episodes(env, None, args.n, base_seed=VAL_SEED)
    rows.append(dict(ckpt="baseline (zero residual)", success=base["success"] / args.n,
                     collision=base["collision"] / args.n,
                     reward=float(np.mean(base["reward"]))))
    print(rows[-1])
    for c in cands:
        st = run_episodes(env, PPO.load(c, device="cpu"), args.n, base_seed=VAL_SEED)
        rows.append(dict(ckpt=c.name, success=st["success"] / args.n,
                         collision=st["collision"] / args.n,
                         reward=float(np.mean(st["reward"]))))
        print(rows[-1], flush=True)
    best = max(rows[1:], key=lambda r: (r["success"], -r["collision"], r["reward"]))
    shutil.copy(d / best["ckpt"], d / "best.zip")
    with open(d / "selection.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("best:", best, "-> best.zip")


if __name__ == "__main__":
    main()
