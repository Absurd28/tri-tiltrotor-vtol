# HANDOFF.md

Current status and open items for the Tilt-Tri-Rotor VTOL drone project.
Updated at the end of each phase.

---

## Phase 0: Setup and repo scaffold

**Status:** In progress (2026-11-12)

### Done
- [x] Machine inspected: Windows 11, i5-13420H, RTX 4050, WSL2 Ubuntu 26.04
- [x] Root cause found: Windows SAC blocks unsigned PyPI DLLs → using WSL2
- [x] Miniconda3 installed in WSL2 (no sudo required)
- [x] Python 3.11 conda env `drone311` created
- [x] Core packages installed: numpy 2.4.6, scipy 1.17.1, matplotlib 3.11.2, pandas 3.1.1
- [x] mujoco 3.17.0 and gymnasium 1.4.0 installed and verified
- [ ] stable-baselines3 — installing
- [ ] tensorboard, onnx, onnxruntime-gpu — installing
- [ ] PyTorch cu124 — pending (after above)
- [ ] ffmpeg — pending
- [x] Folder scaffold: params/, model/, sim/, control/, rl/, eval/, hardware/, docs/, tests/
- [x] params/params.yaml — all vehicle parameters with [measured]/[estimated]/[assumed] tags
- [x] params/__init__.py — load_params(), get_param() loader
- [x] sim/logging_utils.py — get_logger(), setup_file_logger()
- [x] model/smoke_test.xml — minimal MuJoCo test model
- [x] tests/test_smoke.py — smoke tests for imports, params, MuJoCo, PyTorch
- [x] conftest.py, pytest.ini
- [x] scripts/viewer.py
- [x] docs/INSTALL_LOG.md, docs/DECISIONS.md
- [x] README.md with full WSL2 install instructions
- [x] .gitignore
- [ ] git commit (pending all packages)
- [ ] pytest run (pending PyTorch install)

### Files created this phase
- `params/params.yaml` — vehicle parameters
- `params/__init__.py` — parameter loader
- `sim/logging_utils.py` — logging helpers
- `model/smoke_test.xml` — test model
- `tests/test_smoke.py` — smoke tests
- `conftest.py`, `pytest.ini` — test config
- `scripts/viewer.py` — MuJoCo viewer launcher
- `docs/INSTALL_LOG.md` — install log
- `docs/DECISIONS.md` — design decisions
- `README.md` — project README
- `.gitignore`

### Parameter changes
None yet. All parameters from project specification.

### Open issues
1. SAC (Smart App Control) on Windows — documented in D-00. All work in WSL2.
2. ffmpeg not yet installed (needs `sudo apt-get` or conda-forge).
3. PyTorch install pending — large (~2.5 GB), will verify CUDA after install.
4. Viewer smoke test cannot run headless in WSL2 without a display server.
   - Launch mode (interactive) works with VcXsrv/Xming on Windows.
   - Offscreen rendering works for video generation (Phase 3+).

---

## Next phase: Phase 1 — Modelling, 3D and MuJoCo

Waiting for user go signal.

Plan:
1. Build MJCF model from params.yaml: fuselage, booms, wing, nacelles, tilt joints
2. Thrust sites, sensor sites, contacts
3. Mass/CG/inertia checker script
4. Viewer with tilt sliders
5. Top/side/bottom renders

Needs from user: nothing at this stage.