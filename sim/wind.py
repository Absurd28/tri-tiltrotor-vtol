"""Wind model: steady vector + first-order (Ornstein-Uhlenbeck) gusts.

The OU process is a cheap stand-in for a Dryden gust spectrum: colored noise
with correlation time tau_gust, standard deviation sigma per axis. Seeded for
reproducibility.
"""
from __future__ import annotations

import numpy as np


class Wind:
    def __init__(self, steady=(0.0, 0.0, 0.0), gust_sigma=0.0,
                 tau_gust=2.0, seed=0):
        self.steady = np.asarray(steady, dtype=float)
        self.sigma = float(gust_sigma)
        self.tau = tau_gust
        self.rng = np.random.default_rng(seed)
        self.gust = np.zeros(3)

    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.gust = np.zeros(3)

    def step(self, dt: float) -> np.ndarray:
        """Advance gusts; returns wind velocity in WORLD frame (m/s)."""
        if self.sigma > 0.0:
            a = dt / self.tau
            noise = self.rng.standard_normal(3)
            self.gust += -a * self.gust + self.sigma * np.sqrt(2 * a) * noise
            # vertical gusts weaker near ground; keep simple 50 %
            self.gust[2] *= 1.0 - 0.5 * a
        return self.steady + self.gust
