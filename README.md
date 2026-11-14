# Tilt-Tri-Rotor VTOL Drone — Simulation & RL

Final-year project: MuJoCo simulation, baseline controller and RL policy for a 3-motor
tilt-rotor VTOL drone.

## Quick start (Windows + WSL2)

> All simulation work runs inside WSL2 Ubuntu 26.04.
> The project files stay on the Windows filesystem and are accessed from WSL via `/mnt/c/...`.

### Prerequisites

- Windows 11 with WSL2 + Ubuntu 26.04 (already installed)
- NVIDIA driver ≥ 525 for CUDA in WSL2 (current: 591.86)

### One-time environment setup

Open a **WSL2 terminal** (search "Ubuntu" in Start, or `wsl -d Ubuntu-26.04` in PowerShell):

```bash
# 1. Install Miniconda3 (if not already done)
curl -sL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh
bash /tmp/miniconda.sh -b -p ~/miniconda3
~/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
~/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# 2. Create Python 3.11 environment
~/miniconda3/bin/conda create -n drone311 python=3.11 -c conda-forge -y

# 3. Activate and install packages
export PATH="$HOME/miniconda3/envs/drone311/bin:$PATH"
cd /mnt/c/Users/<YOUR_USERNAME>/Desktop/Drone_7th_sem  # adjust path

pip install -r requirements.txt          # pinned versions (torch comes with SB3)

# 4. ffmpeg for videos (no sudo needed)
conda install -n drone311 ffmpeg -c conda-forge -y
```

Exact versions actually installed are logged in `docs/INSTALL_LOG.md`.

### Every session

```bash
wsl -d Ubuntu-26.04   # open WSL terminal from PowerShell
export PATH="$HOME/miniconda3/envs/drone311/bin:$PATH"
cd /mnt/c/Users/<YOUR_USERNAME>/Desktop/Drone_7th_sem
```

Or add this alias to your WSL `~/.bashrc`:
```bash
alias drone='export PATH="$HOME/miniconda3/envs/drone311/bin:$PATH" && cd /mnt/c/Users/<YOUR_USERNAME>/Desktop/Drone_7th_sem'
```

### Run tests

```bash
pytest                     # all tests
pytest tests/test_smoke.py # smoke test only
```

### Reproduce every result in the report

All seeds are fixed; each command regenerates one table/figure.

```bash
python scripts/check_model.py                       # Phase 1 acceptance (mass/CG/inertia/clearance)
MUJOCO_GL=egl python scripts/render_views.py        # docs/renders/*.png
pytest -m "not slow"                                # physics + env tests (fast)
pytest                                              # + full baseline missions (slow)
python control/trim.py                              # eval/trim_table.csv, eval/tilt_corridor.png
python eval/run_mission.py --tag calm               # eval/baseline_metrics.csv row + eval/mission_calm.png
python eval/run_mission.py --tag wind3 --wind 3     # completes
python eval/run_mission.py --tag wind4 --wind 4     # fails (known limitation, report §4)
python eval/run_mission.py --tag wind5 --wind 5     # fails (known limitation, report §4)
python eval/run_mission.py --tag cg0.02 --cg-shift 0.02
MUJOCO_GL=egl python eval/run_mission.py --tag video --video docs/videos/mission_calm.mp4
# RL (≈5.5 h on a laptop for the full chain, see C:/Temp/chain.sh equivalent below)
python rl/train.py --task hover    --steps 1000000 --name s1_hover    --dr --log-std-init -2.3 --lr 1e-4
python rl/train.py --task waypoint --steps 1000000 --name s2_waypoint --dr --log-std-init -2.3 --lr 1e-4 --init runs/s1_hover/final.zip
python rl/train.py --task obstacle --steps 300000 --name s5_obstacle --dr --difficulty 0.4 --init runs/s2_waypoint/final.zip   # mode B (stopped: no learning)
python rl/select_checkpoint.py --name s1_hover --task hover          # best.zip on validation seeds
python rl/select_checkpoint.py --name s2_waypoint --task waypoint
python rl/train.py --task obstacle --mode G --steps 1000000 --name s5_obstacle_G --dr --difficulty 0.4 --log-std-init -1.0 --lr 3e-4
python rl/select_checkpoint.py --name s5_obstacle_G --task obstacle --mode G --difficulty 0.4
bash scripts/evaluate_all.sh                        # eval/rl_metrics.csv (50 test episodes each)
python rl/export_onnx.py runs/s1_hover/best.zip     # ONNX + parity check (also s2, s5_obstacle_G)
python hardware/power_budget.py                     # electrical budget tables
```

### Fly the drone yourself (ground station)

```bash
bash scripts/run_gcs.sh        # then open http://localhost:8765
```

RC-style manual flight (on-screen sticks, keyboard or a USB RC transmitter) and automatic
delivery missions — see [gcs/README.md](gcs/README.md). Electronics design: [hardware/README.md](hardware/README.md).

### Run MuJoCo viewer (requires display / VcXsrv on Windows)

```bash
python scripts/viewer.py
```

## Project structure

```
params/         Vehicle parameters (params.yaml + loader)
model/          MJCF XML models and meshes
sim/            Simulation physics (motor, aero, sensors)
control/        Baseline PID controller and mixers
rl/             Gymnasium environment and RL training
eval/           Evaluation scripts and metrics
gcs/            Ground station: real-time sim server + browser cockpit
hardware/       Electronics: block diagram, BOM, power budget, pin map
docs/           INSTALL_LOG.md, DECISIONS.md, report draft
tests/          pytest tests for every physics block
scripts/        Utility scripts (viewer, trim, render)
```

## GPU

PyTorch is installed with CUDA 12.4 support (`cu124`). The NVIDIA driver (591.86) exposes
CUDA 12.8 on the host; WSL2 passes the GPU through `/dev/dxg`. Verify with:

```bash
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

## Conventions

- World frame: ENU (x=East, y=North, z=Up)
- Body frame: FLU (x=forward, y=left, z=up)
- Units: SI (m, kg, s, N, rad)
- Tilt 0° = thrust up (hover); 90° = thrust forward (cruise)

## See also

- `docs/DECISIONS.md` — design decision log
- `docs/INSTALL_LOG.md` — package versions and install notes
- `params/params.yaml` — all vehicle parameters with source tags