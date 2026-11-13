#!/usr/bin/env python3
"""Export an SB3 PPO policy to ONNX and verify numerical parity.

Usage: python rl/export_onnx.py runs/s1_hover/final.zip
Writes <name>.onnx next to the zip and checks onnxruntime output matches
the torch policy to 1e-5.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))


class PolicyWrapper(torch.nn.Module):
    """Deterministic actor: obs -> action mean, tanh-squashed to [-1,1]
    bounds the same way SB3 predict(deterministic=True) clips."""

    def __init__(self, policy):
        super().__init__()
        self.extractor = policy.mlp_extractor
        self.features = policy.features_extractor
        self.action_net = policy.action_net

    def forward(self, obs):
        feat = self.features(obs)
        latent_pi = self.extractor.forward_actor(feat)
        return torch.clamp(self.action_net(latent_pi), -1.0, 1.0)


def main():
    zip_path = Path(sys.argv[1])
    from stable_baselines3 import PPO
    model = PPO.load(zip_path, device="cpu")
    wrapper = PolicyWrapper(model.policy).eval()

    obs_dim = model.observation_space.shape[0]
    dummy = torch.zeros(1, obs_dim)
    out_path = zip_path.with_suffix(".onnx")
    torch.onnx.export(wrapper, (dummy,), str(out_path),
                      input_names=["obs"], output_names=["action"],
                      dynamo=False)

    import onnxruntime as ort
    sess = ort.InferenceSession(str(out_path),
                                providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(20):
        obs = rng.standard_normal((1, obs_dim)).astype(np.float32)
        with torch.no_grad():
            ref = wrapper(torch.from_numpy(obs)).numpy()
        got = sess.run(None, {"obs": obs})[0]
        worst = max(worst, float(np.abs(ref - got).max()))
    assert worst < 1e-5, f"ONNX parity error {worst}"
    print(f"wrote {out_path} | parity max err {worst:.2e}")


if __name__ == "__main__":
    main()
