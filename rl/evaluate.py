#!/usr/bin/env python3
"""Evaluate a policy (or the baseline) on seeded episodes.

Usage:
  python rl/evaluate.py --task hover --policy runs/s1_hover/final.zip -n 50
  python rl/evaluate.py --task hover --baseline -n 50     # zero-residual
Appends one row to eval/rl_metrics.csv and prints the summary.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from rl.env import VTOLEnv  # noqa: E402


def run_episodes(env, policy, n, base_seed=10_000):
    stats = dict(success=0, collision=0, crash=0, reward=[], track_err=[],
                 act_smooth=[], energy=[])
    for ep in range(n):
        obs, _ = env.reset(seed=base_seed + ep)
        done = trunc = False
        ep_r, errs, das = 0.0, [], []
        prev_a = np.zeros(9)
        info = {}
        while not (done or trunc):
            if policy is None:
                a = np.zeros(9, dtype=np.float32)
            else:
                a, _ = policy.predict(obs, deterministic=True)
            obs, r, done, trunc, info = env.step(a)
            ep_r += r
            s = env.veh.state()
            errs.append(np.linalg.norm(env.goal - s["pos"]))
            das.append(np.linalg.norm(np.asarray(a) - prev_a))
            prev_a = np.asarray(a).copy()
        stats["reward"].append(ep_r)
        stats["track_err"].append(float(np.mean(errs)))
        stats["act_smooth"].append(float(np.mean(das)))
        stats["energy"].append(100 * (1 - env.veh.battery.soc))
        if info.get("is_success"):
            stats["success"] += 1
        if info.get("collision"):
            stats["collision"] += 1
        elif done and not info.get("is_success"):
            stats["crash"] += 1
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--policy", default=None)
    ap.add_argument("--baseline", action="store_true")
    ap.add_argument("--mode", default="B")
    ap.add_argument("--dr", action="store_true")
    ap.add_argument("--difficulty", type=float, default=0.3)
    ap.add_argument("-n", type=int, default=50)
    ap.add_argument("--label", default=None)
    args = ap.parse_args()

    policy = None
    if not args.baseline:
        from stable_baselines3 import PPO
        policy = PPO.load(args.policy, device="cpu")

    env = VTOLEnv(task=args.task, action_mode=args.mode, dr=args.dr,
                  difficulty=args.difficulty, seed=99)
    st = run_episodes(env, policy, args.n)

    label = args.label or ("baseline" if args.baseline
                           else Path(args.policy).parent.name)
    row = dict(
        label=label, task=args.task, n=args.n,
        success_rate=st["success"] / args.n,
        collision_rate=st["collision"] / args.n,
        crash_rate=st["crash"] / args.n,
        mean_reward=np.mean(st["reward"]),
        mean_track_err=np.mean(st["track_err"]),
        act_smoothness=np.mean(st["act_smooth"]),
        mean_energy_pct=np.mean(st["energy"]),
    )
    print(" | ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                     for k, v in row.items()))

    out = ROOT / "eval" / "rl_metrics.csv"
    new = not out.exists()
    with open(out, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    print("appended to", out.relative_to(ROOT))


if __name__ == "__main__":
    main()
