| Condition | Power | Battery current | Endurance (80 % usable) |
|---|---|---|---|
| Hover (all 3 motors + avionics + servos) | 638 W | 28.7 A | 8.4 min |
| Cruise 17 m/s (wing motors, L/D 8) | 176 W | 7.9 A | 30 min (~31 km) |
| One motor at max thrust (2.1 kgf) | 491 W | 23.3 A | — |
| Peak: 3 motors max + servos slewing | 1528 W | 68.8 A | — |

| Check | Value | Requirement | Result |
|---|---|---|---|
| ESC rating / max motor current | 40 A / 23.3 A | ≥ 1.5× | ✓ 1.72× |
| Servo UBEC / peak servo current | 10 A / 6 A | ≥ 1.5× | ✓ 1.67× |
| Battery burst (45C) / peak current | 225 A / 69 A | ≥ 2× | ✓ 3.3× |
| Thrust-to-weight | 1.79 | ≥ 1.6 for VTOL | ✓  |

Wire gauge: battery/PM **10 AWG**, ESC leads **14 AWG** (≥ 25 % ampacity margin).

| Mass budget line | BOM total | params.yaml budget | Δ |
|---|---|---|---|
| battery | 700 g | 700 g | +0.0 % |
| motors | 420 g | 420 g | +0.0 % |
| escs | 90 g | 90 g | +0.0 % |
| tilt_servos | 135 g | 135 g | +0.0 % |
| surface_servos | 72 g | 72 g | +0.0 % |
| avionics_sensors | 274 g | 270 g | +1.5 % |
