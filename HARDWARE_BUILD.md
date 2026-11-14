# Hardware build guide — tri-tiltrotor VTOL

How to turn the simulated aircraft into a real one: what to buy, how to build it, and how to
bring it up safely. The electrical design (block diagram, pin map, wiring rules, power budget)
is in [hardware/README.md](hardware/README.md); this guide is the practical companion.

> **Status: not yet built.** Every number below comes from the simulation and the parameter file
> (`params/params.yaml`), where most values are estimates (±30 %). The real aircraft must be
> measured and bench-tested step by step (§6) before any free flight. Nothing in this repository
> shows that a real aircraft built from it is safe to fly.

---

## 1. What you are building

| | Value |
|---|---|
| Layout | conventional wing + twin tail booms; two tilting wing motors + one tilting tail motor |
| Take-off mass | 3.53 kg including a 0.5 kg payload |
| Wing | 1.60 m span, 0.22 m chord (0.352 m², aspect ratio 7.3), leading edge 7 cm ahead of the CG |
| Length | 1.10 m (nose at +0.45 m, boom tips at −0.62 m from the CG) |
| Wing motors | hubs at x = +0.20 m, y = ±0.40 m, nacelles tilt 0° (up) → 90° (forward) |
| Tail motor | hub at x = −0.40 m between the booms, tilts 0–30°, in a notch behind the fuselage |
| Booms | 0.42 m apart (outer edges), carrying the stabilisers (at x = −0.55 m) and twin fins |
| Props | 12 × 6.5 in; left wing CCW, right wing CW (reaction torques cancel), tail CCW |
| Speeds | stall 11.5 m/s, cruise 17 m/s, transition 4 → 14 m/s |
| Endurance (estimate) | hover 8.4 min, cruise ≈ 30 min (`hardware/power_budget.py`) |

Pictures of the target geometry: [docs/renders/](docs/renders/) (front, side, top, iso hover/cruise).

## 2. Shopping list

Links point to the manufacturer's page where one exists, otherwise to a retailer. They were
checked in November 2026; prices are approximate and change, and stock varies by country — the
same part numbers are sold by regional drone shops. Buy one or two spares of props, servos and
connectors.

### Electronics and propulsion

| # | Part | Qty | Recommended part | Where to buy | ≈ Price (USD) |
|---|---|---|---|---|---|
| 1 | Flight controller (gyro/accel IMUs, baro) | 1 | Holybro Pixhawk 6C Mini | [holybro.com](https://holybro.com/products/pixhawk-6c-mini) | 180–200 |
| 2 | GNSS + compass + safety switch | 1 | Holybro M10 GPS (10-pin, SKU 12040) | [holybro.com](https://holybro.com/products/m10-gps) | 44 |
| 3 | Airspeed sensor + pitot | 1 | Holybro MS4525DO digital airspeed sensor | [holybro.com](https://holybro.com/products/digital-air-speed-sensor-ms4525do) | — |
| 4 | Power module | 1 | Holybro PM02 V3 (12S; 60 A cont. / 100 A burst) — **replace its 30 A XT60 lead with 10 AWG + XT90** | [holybro.com](https://holybro.com/products/pm02-v3-12s-power-module) | 19 |
| 5 | Telemetry radio pair | 1 | Holybro SiK Telemetry Radio V3 — **433 or 915 MHz, whichever is legal where you fly** | [Holybro products](https://holybro.com/collections/all-product) · [docs](https://docs.holybro.com/radio/sik-telemetry-radio-v3) | from 59 |
| 6 | Motors | 3 | T-Motor MN4014 400KV (4–8S, 150 g) | [T-Motor store](https://store.tmotor.com/) · [IR-Lock](https://irlock.com/products/t-motor-mn4014-kv400) | — |
| 7 | ESCs (no BEC) | 3 | T-Motor AIR 40A 2–6S | [T-Motor store](https://store.tmotor.com/goods.php?id=368) · [IR-Lock](https://irlock.com/products/t-motor-air-40a-2-6s-multi-rotor-uav-drone-esc) | 40 each |
| 8 | Propellers | 2 + 1 (+spares) | APC 12×6E (tractor) and 12×6EP (pusher) — one of each wing side + tail; check the rotation marking before fitting | [12×6E](https://www.readymaderc.com/products/details/apc-12x6e-prop) · [12×6EP](https://www.readymaderc.com/products/details/apc-12x6ep-pusher-prop) | 5 each |
| 9 | Battery | 1 (+1) | 6S 5000 mAh LiPo, ≥ 45C, XT90 (e.g. Turnigy nano-tech 5000 6S 45–90C) | [HobbyKing](https://hobbyking.com/turnigy-nano-tech-5000mah-6s-45-90c-lipo-pack-xt90.html) · [CNHL](https://chinahobbyline.com/products/cnhl-g-plus-5000mah-22-2v-6s-100c-lipo-battery-with-xt90-plug) | 70–110 |
| 10 | Power distribution board | 1 | Matek FCHUB-12S V2 (440 A current sensor, 5 V/12 V BEC) | [Unmanned Tech](https://www.unmannedtechshop.co.uk/products/matek-fchub-12s-v2-pdb-with-440a-current-sensor-dual-bec-for-x-class-quad) | 25–35 |
| 11 | Servo regulator (UBEC) | 1 | Hobbywing UBEC 10A HV (3–14S in, 6/7.4/8.4 V out, 35 g) | [hobbywing.com](https://www.hobbywing.com/en/products/ubec-10a-hv289) | — |
| 12 | Companion computer | 1 | Raspberry Pi Zero 2 W | [raspberrypi.com](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/) | 15 |
| 13 | Camera | 1 | Raspberry Pi Camera Module 3 **+ Zero camera cable** (the standard cable does not fit the Zero) | [raspberrypi.com](https://www.raspberrypi.com/products/camera-module-3/) | 25 |
| 14 | Companion supply | 1 | 5 V 3 A UBEC (e.g. Matek UBEC Duo or Pololu 5 V 3 A) | regional drone shop | 10 |
| 15 | Lidar rangefinders | 2 | Benewake TF-Luna (0.2–8 m, UART/I2C) — one down, one forward | [benewake.com](https://en.benewake.com/TFLuna/index.html) | 20 each |
| 16 | RC receiver | 1 | RadioMaster RP1 V2 ExpressLRS 2.4 GHz | [radiomasterrc.com](https://radiomasterrc.com/products/rp1-expresslrs-2-4ghz-nano-receiver) | 19 |
| 17 | RC transmitter | 1 | RadioMaster Boxer (ELRS, EdgeTX) — also works as a USB joystick in the simulator | [radiomasterrc.com](https://radiomasterrc.com/products/boxer-radio-transparent-version) | 160 |
| 18 | FPV camera + VTX (optional) | 1 | Caddx Ant nano camera + 5.8 GHz VTX (legal power for your region) | [GetFPV](https://www.getfpv.com/fpv/cameras/caddx-ant-1200tvl-1-8mm-fpv-nano-camera-14x14-black.html) | 20 + VTX |
| 19 | Tilt servos | 3 | ~45 g digital HV metal-gear, ≥ 15 kg·cm, ≈ 0.10 s/60° — **size from the measured hinge torque** | regional RC shop | 30–60 each |
| 20 | Surface servos | 6 (+2) | 12–15 g digital metal-gear (Emax ES08MD II / [ES09MD HV](https://speedyfpv.com/collections/servos/products/emax-es09md-hv-metal-gear-digital-servo-high-voltage-all-purpose) class) | RC shop | 10–15 each |
| 21 | Payload release | 1 | MG90S metal-gear micro servo + 3D-printed hook | any electronics shop | 4 |
| 22 | Ground telemetry | 1 | second SiK radio (comes in the pair) + USB cable | — | — |

**Why these choices** (details in [docs/DECISIONS.md](docs/DECISIONS.md), D-19): PX4-class
flight controller holds all safety-critical loops; the RL policy (`policies/s1_hover.onnx`) runs
on the 11 g Pi Zero 2 W so the aircraft keeps flying on the baseline controller if it fails;
separate regulators stop servo current spikes from resetting the computers.

**Motor note:** at 400 KV on 6S, a 12 × 6.5 prop may not reach the simulated 2.1 kgf maximum
thrust per motor (needed for a 1.79 thrust-to-weight). Measure it on a thrust stand first (§6);
if short, use a 13-inch prop or a ~460 KV 6S-rated motor and update `params.yaml`.

### Airframe materials and hardware

| Item | Suggestion |
|---|---|
| Wing | EPP/EPO foam core with a carbon spar (two 8–10 mm carbon tubes or a 10 × 1 mm carbon strip), laminated or covered; or a hot-wire-cut foam core with fibreglass skin |
| Fuselage | 3 mm plywood/balsa box or 3D-printed (PETG/ASA, 3–4 perimeters) with removable top hatch; belly hatch for the payload |
| Tail booms | 2 × 10–12 mm carbon tubes, ~0.75 m long |
| Stabilisers / fins | 3–5 mm depron or balsa, carbon-rod reinforced |
| Nacelle hinges | 3D-printed nacelle with an 8 mm carbon tube axle in two flanged bearings (e.g. 8 × 12 × 3.5 mm) per nacelle |
| Tilt linkage | ball-link pushrods (M2/M3), servo arm ≥ 20 mm |
| Fasteners | M2/M2.5/M3 nylon and steel screws, threaded inserts, zip ties, Velcro battery strap |
| Wiring | 10 AWG silicone (battery/PDB), 14 AWG (ESCs), 22–26 AWG servo extensions, XT90 + XT60 connectors, heat-shrink, 25 V 470 µF low-ESR capacitors (one per ESC) |
| Tools | soldering iron (≥ 80 W for XT90), hot-wire cutter or sharp knives, epoxy (5- and 30-minute), CA glue, 3D printer, multimeter, digital scale, LiPo charger + fireproof bag |
| Test gear | thrust stand with load cell and current meter (e.g. Tyto/RCbenchmark series 1580/1585) |

## 3. Build the airframe

1. **Wing.** Cut the 1.60 m × 0.22 m wing (two halves), install the spar through both halves and
   join them with a 30-minute epoxy joiner. Leave the motor-boom pockets at y = ±0.40 m, the
   aileron cut-outs and wire channels. Mount it so the leading edge sits 7 cm ahead of the CG mark.
2. **Fuselage.** Box from nose (+0.45 m) to the tail bulkhead (−0.21 m). Inside: battery bay
   6–10 cm ahead of the CG on a sliding tray (the battery is the balance weight), flight
   controller exactly at the CG on soft mounts, avionics bay in the nose, payload hatch under the
   CG, rear servo bay at −0.18 m for the elevator/rudder servos (pushrods, D-07).
3. **Booms and tail.** Two carbon tubes 0.42 m apart (outer edges) from the wing to −0.62 m.
   Fix the horizontal stabilisers at −0.55 m and the twin fins at the boom tips. Make sure the
   tail-motor notch (0.17 m radius) is clear: the bulkhead face must be **≥ 0.19 m ahead of the
   tail hub** — the simulation's clearance check caught a 2.4 cm error here (D-11).
4. **Nacelles.** Print the two wing nacelles and the tail pod. Each rotates on its carbon axle and
   bearings; the servo drives it through a ball-link pushrod. Check the full range by hand: wing
   nacelles 0–90°, tail pod 0–30°, with no binding and ≥ 1.5 cm prop clearance to every part at
   every angle.
5. **Landing gear.** Two skids under the booms plus a small nose skid (D-09) — three-point stance,
   because the boom skids alone sit behind the CG and the aircraft would tip onto its nose.
6. **Weigh everything** as you go and record it; the budget per line is in `params/params.yaml`.

## 4. Install the electronics

Follow the block diagram and pin map in [hardware/README.md](hardware/README.md). In short:

1. Battery (XT90, 10 AWG) → power module → PDB → three ESCs (14 AWG, capacitor at each ESC).
2. Motor phase wires through a **service loop** at each nacelle hinge, strain-relieved, so the
   full 0–90° tilt never pulls on them.
3. ESC signals → flight controller FMU outputs AUX1–3 (left, right, tail).
4. Servo UBEC 7.4 V → flight-controller servo rail only. Tilt servos → MAIN1–3, ailerons →
   MAIN4–5, elevators → MAIN6–7, rudders → MAIN8 + AUX4, payload servo → AUX5.
5. GPS on a mast above the wing (GPS1), airspeed sensor on I2C with the pitot at the nose, down
   lidar on UART4, ELRS receiver on TELEM3, SiK radio on TELEM1.
6. Pi Zero 2 W from its own 5 V UBEC; UART to TELEM2 (ground shared through that cable); camera
   on the Zero camera cable; forward lidar on the Pi's I2C.
7. **Before connecting the battery:** multimeter continuity check (no short battery + to −),
   then first power-up from a current-limited bench supply (22 V, 2 A).

## 5. Software setup

1. **Flight controller.** Install [QGroundControl](https://qgroundcontrol.com/) on the laptop,
   flash PX4 (≥ 1.14, required by the M10 GPS), choose a *VTOL tiltrotor* airframe (PX4's
   reference tri-tiltrotor is the E-flite Convergence), then set the outputs exactly as in the
   pin map. Calibrate accelerometer, gyro, compass, level horizon, airspeed (`SENS_EN_MS4525DO`)
   and radio. Set the failsafes from `hardware/README.md` §6.
2. **RC transmitter.** Bind the RP1 to the Boxer (ELRS). Assign switches like the simulator:
   SA arm, SB flight mode, SC position/stabilised, SD transition, SE companion/RL on-off.
   Practise in the simulator first with the same radio over USB (`bash scripts/run_gcs.sh`).
3. **Companion computer.** Raspberry Pi OS Lite on the Zero 2 W; enable the UART and the camera
   (`dtoverlay=imx708`); `pip install onnxruntime pymavlink`; copy `policies/s1_hover.onnx`.
   The policy outputs a bounded residual on top of the flight controller's commands (D-15/D-16);
   it is the last thing to enable, after the aircraft flies well on PX4 alone.

## 6. Bench tests (props off unless stated) — and update the model

| Test | How | Update in `params/params.yaml` |
|---|---|---|
| Masses and CG | weigh each part; balance on the CG mark | `mass:` lines, battery position |
| Thrust & torque (props ON, on the stand, clamped) | thrust stand sweep 0–100 % throttle at 22.2 V | `kT`, `kQ`, `max_thrust_per_motor`, motor lag |
| Current draw | same sweep with current logging | power budget check (`hardware/power_budget.py`) |
| Servo speed & backlash | tilt step 0 → 90° under load, film at 240 fps | `servo_rate_limit`, `servo_lag`, backlash |
| Battery | discharge test at 20–30 A | `battery_internal_R`, OCV curve |
| IMU noise | 10 min static log | sensor noise parameters |

Change every tag you measure from `[estimated]`/`[assumed]` to `[measured]`, regenerate the
model (`python model/build_mjcf.py`), re-run `pytest`, re-fly the baseline mission in simulation
and **retrain the RL policies** on the measured model before using them on the aircraft.

## 7. First flights — in this order

1. Tethered hover (rope to a heavy anchor, < 1 m), multicopter mode only, RL off.
2. Free hover in an open field at 2–5 m; tune PX4 rate/attitude gains; check yaw authority —
   **this design's known weak axis** (report §4): keep the first flights in calm wind.
3. Slow hover translations and nose-into-wind holds.
4. Fixed-wing behaviour: hand-launch glide tests with nacelles locked at 90° (or tilt to 90° at
   altitude only after step 5).
5. Transitions at ≥ 30 m altitude, into the wind, with a back-transition switch ready.
6. Only then: auto missions, then the companion computer with the RL residual.

**Legal:** at 3.5 kg most countries require drone registration and/or a pilot certificate,
restrict flight altitude and areas, and regulate radio power (433/868/915 MHz telemetry,
5.8 GHz video). Check your national aviation authority's rules before flying.

**LiPo safety:** charge in a fireproof bag, never unattended, store at storage voltage
(3.8 V/cell), discard puffed packs.
