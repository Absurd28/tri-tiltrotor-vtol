"""Light domain randomization (reduced S6 per the compressed plan).

Applied per episode:
  mass +-15 %  (scales every body mass and inertia consistently)
  kT +-10 %, motor lag +-30 %
  aero CL_alpha/CD0 +-20 %
  steady wind up to 3 m/s with gusts
"""
from __future__ import annotations

import numpy as np


class DomainRandomizer:
    def __init__(self, enabled=True):
        self.enabled = enabled

    def apply(self, veh, rng: np.random.Generator):
        """Mutate a Vehicle in place for this episode. Returns a dict of the
        factors used (for logging)."""
        if not self.enabled:
            return {}
        f_mass = rng.uniform(0.85, 1.15)
        f_kt = rng.uniform(0.90, 1.10)
        f_lag = rng.uniform(0.70, 1.30)
        f_cla = rng.uniform(0.80, 1.20)
        f_cd0 = rng.uniform(0.80, 1.20)

        m = veh.model
        if not hasattr(veh, "_nominal_mass"):
            veh._nominal_mass = m.body_mass.copy()
            veh._nominal_inertia = m.body_inertia.copy()
        m.body_mass[:] = veh._nominal_mass * f_mass
        m.body_inertia[:] = veh._nominal_inertia * f_mass

        base_p = veh.p
        for mtr in veh.motors:
            mtr.kT = base_p["propulsion"]["kT"] * f_kt
            mtr.kQ = base_p["propulsion"]["kQ_over_kT"] * mtr.kT
            mtr.tau = base_p["propulsion"]["motor_lag"] * f_lag
        veh.aero.CLa = base_p["aero"]["CL_alpha"] * f_cla
        veh.aero.CD0_wing = 0.02 * f_cd0
        veh.aero.CD0_other = (base_p["aero"]["CD0"] - 0.02) * f_cd0

        wind_mag = rng.uniform(0.0, 3.0)
        ang = rng.uniform(0, 2 * np.pi)
        veh.wind.steady = np.array([wind_mag * np.cos(ang),
                                    wind_mag * np.sin(ang), 0.0])
        veh.wind.sigma = wind_mag * 0.25
        return dict(mass=f_mass, kT=f_kt, lag=f_lag, CLa=f_cla,
                    CD0=f_cd0, wind=wind_mag)
