"""Motor + propeller model (per motor).

Physics (viva notes):
- ESC/motor speed dynamics: first-order lag, d(omega)/dt = (omega_cmd - omega)/tau.
- Static thrust T = kT * omega^2, torque Q = kQ * omega^2 (kQ = kQ/kT ratio * kT).
- Axial-inflow thrust decay: a fixed-pitch prop loses thrust as axial airspeed
  approaches its "pitch speed" V_p = omega/(2*pi) * pitch. Linear model:
      T = kT*omega^2 * (1 - V_ax / V_p),  clamped to [0, 1.1].
  This captures why the 12x6.5 prop that hovers well cannot push past ~20 m/s.
- Battery sag: V_oc falls linearly with state of charge; terminal voltage
  V = V_oc - I*R_int. Max motor speed scales with V/V_nominal (KV model),
  so available thrust drops as the pack drains or under high load.
- Reaction torque on the airframe: -spin * Q about the prop axis.
"""
from __future__ import annotations

import numpy as np


class Motor:
    def __init__(self, p: dict, spin: int, rng=None):
        pr = p["propulsion"]
        self.kT = pr["kT"]
        self.kQ = pr["kQ_over_kT"] * pr["kT"]
        self.tau = pr["motor_lag"]
        self.omega_max_nom = pr["max_omega"]
        self.pitch_m = pr["prop_pitch_in"] * 0.0254
        self.spin = spin  # +1 CCW (reaction torque -z), -1 CW
        self.omega = 0.0

    def reset(self, omega0: float = 0.0):
        self.omega = omega0

    def step(self, throttle: float, v_axial: float, dt: float,
             v_scale: float = 1.0) -> tuple[float, float]:
        """Advance one physics step.

        throttle: 0..1 command
        v_axial:  airspeed component along the thrust axis (+ = into the prop)
        v_scale:  battery voltage ratio V/V_nom (sag reduces top speed)
        returns (thrust N, reaction torque N*m signed about thrust axis)
        """
        throttle = float(np.clip(throttle, 0.0, 1.0))
        omega_cmd = throttle * self.omega_max_nom * v_scale
        self.omega += (omega_cmd - self.omega) * dt / self.tau

        T_static = self.kT * self.omega**2
        if self.omega > 1.0:
            v_pitch = self.omega / (2 * np.pi) * self.pitch_m
            inflow = np.clip(1.0 - v_axial / v_pitch, 0.0, 1.1)
        else:
            inflow = 1.0
        thrust = T_static * inflow
        q_reaction = -self.spin * self.kQ * self.omega**2
        return thrust, q_reaction

    def induced_velocity(self, thrust: float, rho: float, disk_area: float) -> float:
        """Momentum-theory induced velocity at the disc (hover formula)."""
        return float(np.sqrt(max(thrust, 0.0) / (2 * rho * disk_area)))


class Battery:
    """6S pack: linear open-circuit voltage vs state of charge + IR sag."""

    def __init__(self, p: dict):
        pr = p["propulsion"]
        self.v_full = pr["battery_voltage_full"]
        self.v_empty = pr["battery_voltage_empty"]
        self.v_nom = pr["battery_voltage_nominal"]
        self.r_int = pr["battery_internal_R"]
        self.capacity_J = pr["battery_capacity_wh"] * 3600.0
        self.eta = pr["motor_efficiency"]
        self.fm = pr["figure_of_merit"]
        self.energy_J = self.capacity_J
        self.rho = p["simulation"]["air_density"]
        prop_r = p["propulsion"]["prop_diameter"] / 2
        self.disk_area = np.pi * prop_r**2

    @property
    def soc(self) -> float:
        return self.energy_J / self.capacity_J

    def v_oc(self) -> float:
        return self.v_empty + (self.v_full - self.v_empty) * self.soc

    def step(self, thrusts, dt: float) -> tuple[float, float]:
        """Drain energy for the given per-motor thrusts.
        Returns (terminal voltage, scale factor V/V_nom for motor top speed)."""
        # Ideal induced power per motor / figure of merit / motor efficiency
        p_total = 0.0
        for T in thrusts:
            p_ideal = max(T, 0.0)**1.5 / np.sqrt(2 * self.rho * self.disk_area)
            p_total += p_ideal / self.fm / self.eta
        v = self.v_oc()
        current = p_total / max(v, 1.0)
        v_term = v - current * self.r_int
        self.energy_J = max(self.energy_J - p_total * dt, 0.0)
        return v_term, float(np.clip(v_term / self.v_nom, 0.3, 25.2 / 22.2))
