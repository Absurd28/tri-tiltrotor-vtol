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
source ~/miniconda3/bin/activate drone311
cd /mnt/c/Users/<YOUR_USERNAME>/Desktop/Drone_7th_sem  # adjust path

pip install numpy scipy matplotlib pandas pyyaml pytest
pip install "mujoco>=3.2,<4" "gymnasium>=1.0,<2"
pip install stable-baselines3 tensorboard onnx onnxruntime-gpu
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124

# 4. Install ffmpeg (needs sudo)
sudo apt-get install -y ffmpeg
```

### Every session

```bash
wsl -d Ubuntu-26.04   # open WSL terminal from PowerShell
source ~/miniconda3/bin/activate drone311
cd /mnt/c/Users/<YOUR_USERNAME>/Desktop/Drone_7th_sem
```

Or add this alias to your WSL `~/.bashrc`:
```bash
alias drone='source ~/miniconda3/bin/activate drone311 && cd /mnt/c/Users/<YOUR_USERNAME>/Desktop/Drone_7th_sem'
```

### Run tests

```bash
pytest                     # all tests
pytest tests/test_smoke.py # smoke test only
```

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
hardware/       Hardware notes, BOM, wiring (later phases)
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