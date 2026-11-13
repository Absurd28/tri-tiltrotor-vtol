"""Phase 4 acceptance: check_env passes, baseline completes hover through the
env interface, rollout speed and memory reported."""
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from rl.env import VTOLEnv  # noqa: E402


def test_check_env():
    from gymnasium.utils.env_checker import check_env
    env = VTOLEnv(task="hover", action_mode="B", seed=1)
    check_env(env, skip_render_check=True)


def test_baseline_completes_hover_through_env():
    """Mode B with zero residual = pure baseline controller: must survive the
    full hover episode near the goal (Phase 4 acceptance)."""
    env = VTOLEnv(task="hover", action_mode="B", seed=3)
    obs, _ = env.reset(seed=3)
    terminated = truncated = False
    info = {}
    while not (terminated or truncated):
        obs, r, terminated, truncated, info = env.step(np.zeros(9))
    assert truncated and not terminated, "baseline crashed in hover task"
    assert info.get("is_success", False), "baseline did not hold the goal"


def test_mode_a_steps():
    env = VTOLEnv(task="hover", action_mode="A", seed=2)
    env.reset(seed=2)
    for _ in range(20):
        obs, r, term, trunc, _ = env.step(env.action_space.sample() * 0.1)
        assert np.all(np.isfinite(obs))
        if term:
            break


def test_obstacle_env_rays_and_collision():
    env = VTOLEnv(task="obstacle", action_mode="B", seed=5, difficulty=0.8)
    obs, _ = env.reset(seed=5)
    rays = obs[-8:]
    assert np.all(rays >= 0) and np.all(rays <= 1.0)
    # dense field at difficulty 0.8 should return at least one finite ray
    # within a few steps of flying forward
    seen = rays.min() < 1.0
    hit = False
    for _ in range(400):
        obs, r, term, trunc, info = env.step(np.zeros(9))
        seen = seen or obs[-8:].min() < 1.0
        hit = hit or info.get("collision", False)
        if term or trunc:
            break
    assert seen or hit, "rays never detected any obstacle"


def test_determinism():
    e1 = VTOLEnv(task="hover", seed=7)
    e2 = VTOLEnv(task="hover", seed=7)
    o1, _ = e1.reset(seed=7)
    o2, _ = e2.reset(seed=7)
    assert np.allclose(o1, o2)
    a = np.ones(9) * 0.1
    for _ in range(30):
        o1, r1, t1, tr1, _ = e1.step(a)
        o2, r2, t2, tr2, _ = e2.step(a)
    assert np.allclose(o1, o2) and r1 == r2


def test_parked_motors_off_is_finite():
    """Motors stopped at zero airspeed: the slipstream model must not divide
    0/0 (it used to NaN and make MuJoCo reset the vehicle)."""
    from sim.vehicle import Vehicle
    veh = Vehicle()
    s = veh.reset(seed=0, pos=(3.0, 4.0, 0.147))
    for _ in range(50):
        s = veh.step(np.zeros(9))
    assert np.all(np.isfinite(s["pos"]))
    assert np.allclose(s["pos"][:2], [3.0, 4.0], atol=0.05)


def test_untrained_residual_policy_is_baseline():
    """D-16: a fresh mode-B policy must output exactly zero residual, so the
    untrained agent flies the baseline controller."""
    from stable_baselines3 import PPO
    from rl.train import zero_residual_head
    env = VTOLEnv(task="hover", action_mode="B", seed=4)
    model = PPO("MlpPolicy", env, device="cpu", seed=0,
                policy_kwargs=dict(net_arch=[128, 128], log_std_init=-2.3))
    zero_residual_head(model)
    obs, _ = env.reset(seed=4)
    for _ in range(5):
        a, _ = model.predict(obs, deterministic=True)
        assert np.allclose(a, 0.0)
        obs, *_ = env.step(a)
    std = model.policy.log_std.exp().detach().numpy()
    assert np.allclose(std, np.exp(-2.3), rtol=1e-3)


def test_env_speed_and_memory(capsys):
    import resource
    env = VTOLEnv(task="hover", action_mode="B", seed=1)
    env.reset(seed=1)
    n = 300
    t0 = time.perf_counter()
    for _ in range(n):
        _, _, term, trunc, _ = env.step(np.zeros(9))
        if term or trunc:
            env.reset()
    dt = time.perf_counter() - t0
    sps = n / dt
    rss_gb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6
    with capsys.disabled():
        print(f"\n[env benchmark] {sps:.0f} env steps/s single process "
              f"(~{sps*6:.0f} on 6 procs) | RSS {rss_gb:.2f} GB")
    assert sps > 20
    assert rss_gb < 2.0
