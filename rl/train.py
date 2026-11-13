#!/usr/bin/env python3
"""PPO training for the VTOL tasks.

Usage:
  python rl/train.py --task hover --mode B --steps 4000000 --name s1_hover
  python rl/train.py --task waypoint --mode B --steps 4000000 \
      --name s2_waypoint --init runs/s1_hover/final.zip
  python rl/train.py --task obstacle ... --dr

Checkpoints every 100k steps into runs/<name>/; resume with --resume.
TensorBoard: runs/tb/.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))


def make_env_fn(task, mode, dr, difficulty, seed, rank):
    def _f():
        from rl.env import VTOLEnv
        return VTOLEnv(task=task, action_mode=mode, dr=dr,
                       difficulty=difficulty, seed=seed + 1000 * rank)
    return _f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="hover",
                    choices=["hover", "waypoint", "obstacle"])
    ap.add_argument("--mode", default="B", choices=["A", "B"])
    ap.add_argument("--steps", type=int, default=4_000_000)
    ap.add_argument("--name", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-envs", type=int, default=6)
    ap.add_argument("--dr", action="store_true")
    ap.add_argument("--difficulty", type=float, default=0.3)
    ap.add_argument("--log-std-init", type=float, default=-1.6,
                    help="initial log std of the Gaussian policy (-1.6 -> 0.2)")
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--init", type=str, default=None,
                    help="warm start from this policy zip")
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
    from stable_baselines3.common.callbacks import CheckpointCallback

    name = args.name or f"{args.task}_{args.mode}"
    run_dir = ROOT / "runs" / name
    run_dir.mkdir(parents=True, exist_ok=True)

    env = SubprocVecEnv([make_env_fn(args.task, args.mode, args.dr,
                                     args.difficulty, args.seed, r)
                         for r in range(args.n_envs)])
    env = VecMonitor(env, filename=str(run_dir / "monitor"))

    device = "cuda" if torch.cuda.is_available() else "cpu"
    ckpt_files = sorted(run_dir.glob("ckpt_*_steps.zip"))
    if args.resume and ckpt_files:
        model = PPO.load(ckpt_files[-1], env=env, device=device)
        print(f"resumed from {ckpt_files[-1].name} "
              f"at {model.num_timesteps} steps")
    elif args.init:
        model = PPO.load(args.init, env=env, device=device)
        model.num_timesteps = 0
        print(f"warm start from {args.init}")
    else:
        model = PPO(
            "MlpPolicy", env, device=device, seed=args.seed,
            n_steps=1024, batch_size=1536, n_epochs=10,
            learning_rate=args.lr, gamma=0.99, gae_lambda=0.95,
            clip_range=0.2, ent_coef=0.003,
            policy_kwargs=dict(net_arch=[128, 128], log_std_init=args.log_std_init),
            tensorboard_log=str(ROOT / "runs" / "tb"),
            verbose=1,
        )
        if args.mode == "B":
            # residual starts at exactly zero = the baseline controller (D-16)
            with torch.no_grad():
                model.policy.action_net.weight.zero_()
                model.policy.action_net.bias.zero_()

    ckpt = CheckpointCallback(save_freq=max(100_000 // args.n_envs, 1),
                              save_path=str(run_dir), name_prefix="ckpt")
    model.learn(total_timesteps=args.steps, callback=ckpt,
                tb_log_name=name, reset_num_timesteps=not args.resume,
                progress_bar=False)
    model.save(run_dir / "final.zip")
    print(f"saved {run_dir / 'final.zip'}")


if __name__ == "__main__":
    main()
