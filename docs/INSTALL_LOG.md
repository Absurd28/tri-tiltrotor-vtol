# INSTALL_LOG.md

All installations for the Tilt-Tri-Rotor VTOL Drone simulation project.

## Environment

- **Host OS:** Windows 11 Home 10.0.26200
- **Runtime:** WSL2 Ubuntu 26.04 LTS (pre-installed)
- **Python:** 3.11.18 (conda-forge, via Miniconda3)
- **Conda:** 26.9.4

**Why WSL2?** Windows Smart App Control (SAC, `VerifiedAndReputablePolicyState=1`) blocks
unsigned DLLs from PyPI, including MuJoCo, NumPy, SciPy. SAC cannot be bypassed with
`Unblock-File`. WSL2 (Linux) has no such restriction.

**Why Python 3.11?** MuJoCo 3.x and Stable-Baselines3 have wheels for 3.10–3.12.
Ubuntu 26.04's default Python is 3.14, which is too new. Python 3.11 chosen for maximum
pre-built-wheel availability.

---

## Phase 0 Installs

| Package | Version | Source | Why | Approx Size |
|---------|---------|--------|-----|------------|
| Miniconda3 | latest (conda 26.9.4) | repo.anaconda.com (official) | Python env manager, no sudo needed | 213 MB installer |
| Python | 3.11.18 | conda-forge | Required; Ubuntu 26.04 default is 3.14 | ~25 MB |
| numpy | 2.4.6 | PyPI | Core numerical arrays | ~20 MB |
| scipy | 1.17.1 | PyPI | Signal processing, optimization | ~35 MB |
| matplotlib | 3.11.2 | PyPI | Plotting | ~20 MB |
| pandas | 3.1.1 | PyPI | Data logging / CSV export | ~15 MB |
| pyyaml | (latest) | PyPI | params.yaml loading | <1 MB |
| pytest | (latest) | PyPI | Test framework | ~5 MB |
| mujoco | 3.17.0 | PyPI | Physics simulation | ~60 MB |
| gymnasium | 1.4.0 | PyPI | RL environment API | ~5 MB |
| stable-baselines3 | (installing) | PyPI | RL algorithms (PPO, SAC) | ~10 MB |
| tensorboard | (installing) | PyPI | Training visualization | ~15 MB |
| onnx | (installing) | PyPI | Model export | ~5 MB |
| onnxruntime-gpu | (installing) | PyPI | ONNX inference with CUDA | ~300 MB |
| torch (cu124) | (pending) | download.pytorch.org | Deep learning, CUDA 12.4 | ~2.5 GB |
| torchvision | (pending) | download.pytorch.org | Required by SB3 | ~100 MB |
| torchaudio | (pending) | download.pytorch.org | Required by SB3 | ~100 MB |
| ffmpeg | (pending) | Ubuntu apt or conda-forge | Video rendering | ~100 MB |

**Python from python.org (Windows):** Also installed 3.11.9 via winget for Windows testing,
but unused — all simulation work runs in WSL2.

## Notes

- Packages installed with `pip install` inside the `drone311` conda environment.
- No `sudo` used for package installation (all in user home `/home/code_hack_absurd/`).
- `sudo apt-get update` could not be run non-interactively (password required); Python 3.11
  installed via conda-forge instead of apt.