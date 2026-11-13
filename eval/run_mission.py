#!/usr/bin/env python3
"""Fly the full baseline mission and report metrics.

Usage:
    python eval/run_mission.py                 # calm air
    python eval/run_mission.py --wind 5        # steady wind m/s
    python eval/run_mission.py --cg-shift 0.02 # payload shifted 2 cm aft
    python eval/run_mission.py --video out.mp4 # offscreen video render

Writes eval/baseline_metrics.csv (appends one row per run) and a trajectory
plot eval/mission_<tag>.png.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from params import load_params  # noqa: E402
from sim.vehicle import Vehicle  # noqa: E402
from sim.wind import Wind  # noqa: E402
from control.baseline import BaselineController  # noqa: E402


def make_vehicle(wind_ms=0.0, cg_shift=0.0, seed=0):
    """cg_shift: move the payload aft by this many metres (robustness test)."""
    import mujoco
    xml_path = ROOT / "model" / "vtol.xml"
    if abs(cg_shift) > 1e-6:
        spec_xml = xml_path.read_text()
        # payload geom pos: shift x by -cg_shift (aft) in the XML
        import re
        def repl(m):
            x = float(m.group(1)) - cg_shift
            return f'name="payload" mass="0.5" pos="{x:.4f}'
        spec_xml = re.sub(r'name="payload" mass="0\.5" pos="([-\d.]+)',
                          repl, spec_xml)
        model = mujoco.MjModel.from_xml_string(spec_xml)
    else:
        model = None
    wind = Wind(steady=(wind_ms * 0.7, wind_ms * 0.7, 0), gust_sigma=wind_ms * 0.2,
                seed=seed) if wind_ms > 0 else Wind(seed=seed)
    return Vehicle(model=model, wind=wind, seed=seed)


def fly(veh, ctrl, t_max=180.0, video=None):
    dt = veh.dt_ctrl
    s = veh.reset(seed=0, pos=(0, 0, 0.147))
    log = {k: [] for k in ("t", "x", "y", "z", "V", "mode", "tilt",
                           "vz", "soc")}
    frames = []
    renderer = None
    if video:
        import mujoco
        renderer = mujoco.Renderer(veh.model, height=480, width=640)
        cam = mujoco.MjvCamera()

    n = int(t_max / dt)
    for i in range(n):
        u = ctrl.update(s, dt)
        s = veh.step(u)
        log["t"].append(s["time"])
        log["x"].append(s["pos"][0]); log["y"].append(s["pos"][1])
        log["z"].append(s["pos"][2]); log["V"].append(s["airspeed"])
        log["vz"].append(s["vel_w"][2]); log["soc"].append(s["soc"])
        log["mode"].append(ctrl.mode)
        log["tilt"].append(np.rad2deg(s["tilt"][0]))
        if renderer and i % 5 == 0:   # 20 fps
            cam.lookat[:] = s["pos"]; cam.distance = 6.0
            cam.azimuth = 135; cam.elevation = -15
            renderer.update_scene(veh.data, camera=cam)
            frames.append(renderer.render().copy())
        if ctrl.mode == "DONE":
            break
        if not np.all(np.isfinite(s["pos"])) or s["pos"][2] < -1:
            print("CRASH/NaN at t=", s["time"])
            break
    if renderer:
        renderer.close()
    return log, frames


def metrics(log, ctrl):
    t = np.array(log["t"]); z = np.array(log["z"])
    modes = np.array(log["mode"]); V = np.array(log["V"])
    out = {}
    out["completed"] = ctrl.mode == "DONE"
    out["duration_s"] = t[-1] if len(t) else 0.0
    # altitude deviation during TRANSITION/CRUISE/BACK vs 15 m reference
    inflight = np.isin(modes, ["TRANSITION", "CRUISE", "BACK"])
    out["alt_dev_max"] = float(np.max(np.abs(z[inflight] - 15.0))) \
        if inflight.any() else np.nan
    # landing error vs the HOVER2 point
    if out["completed"]:
        xy_end = np.array([log["x"][-1], log["y"][-1]])
        i_h2 = np.where(modes == "HOVER2")[0]
        tgt = np.array([log["x"][i_h2[0]], log["y"][i_h2[0]]]) if len(i_h2) \
            else xy_end
        out["landing_err_m"] = float(np.linalg.norm(xy_end - tgt))
    else:
        out["landing_err_m"] = np.nan
    out["max_V"] = float(V.max()) if len(V) else 0
    out["energy_used_pct"] = 100 * (1 - log["soc"][-1]) if log["soc"] else 0
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wind", type=float, default=0.0)
    ap.add_argument("--cg-shift", type=float, default=0.0)
    ap.add_argument("--video", type=str, default=None)
    ap.add_argument("--tag", type=str, default=None)
    args = ap.parse_args()

    tag = args.tag or (f"wind{args.wind:g}" if args.wind else
                       f"cg{args.cg_shift:g}" if args.cg_shift else "calm")
    veh = make_vehicle(args.wind, args.cg_shift)
    ctrl = BaselineController()
    log, frames = fly(veh, ctrl, video=args.video)
    met = metrics(log, ctrl)

    print(f"[{tag}] completed={met['completed']} "
          f"alt_dev={met['alt_dev_max']:.2f} m "
          f"landing_err={met['landing_err_m']:.2f} m "
          f"duration={met['duration_s']:.1f} s "
          f"energy={met['energy_used_pct']:.1f} %")

    # plot
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    t = log["t"]
    axs[0].plot(t, log["z"]); axs[0].axhline(15, ls="--", c="gray")
    axs[0].set_ylabel("alt m")
    axs[1].plot(t, log["V"]); axs[1].set_ylabel("airspeed m/s")
    axs[2].plot(t, log["tilt"]); axs[2].set_ylabel("wing tilt deg")
    axs[2].set_xlabel("t s")
    # mode change markers
    modes = log["mode"]
    for i in range(1, len(modes)):
        if modes[i] != modes[i - 1]:
            for ax in axs:
                ax.axvline(t[i], color="orange", alpha=0.4)
            axs[0].text(t[i], 17, modes[i], rotation=90, fontsize=7)
    fig.suptitle(f"Baseline mission [{tag}]")
    out_png = ROOT / "eval" / f"mission_{tag}.png"
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    print("plot:", out_png.relative_to(ROOT))

    # csv append
    csv_path = ROOT / "eval" / "baseline_metrics.csv"
    new = not csv_path.exists()
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["tag", "wind", "cg_shift", "completed", "alt_dev_max",
                        "landing_err_m", "duration_s", "energy_used_pct"])
        w.writerow([tag, args.wind, args.cg_shift, met["completed"],
                    f"{met['alt_dev_max']:.3f}", f"{met['landing_err_m']:.3f}",
                    f"{met['duration_s']:.1f}",
                    f"{met['energy_used_pct']:.2f}"])

    if args.video and frames:
        import subprocess, tempfile
        tmp = Path(tempfile.mkdtemp())
        for i, fr in enumerate(frames):
            plt.imsave(tmp / f"f{i:05d}.png", fr)
        ff = "/home/code_hack_absurd/miniconda3/envs/drone311/bin/ffmpeg"
        subprocess.run([ff, "-y", "-framerate", "20", "-i",
                        str(tmp / "f%05d.png"), "-c:v", "libx264",
                        "-pix_fmt", "yuv420p", args.video], check=True,
                       capture_output=True)
        print("video:", args.video)

    return 0 if met["completed"] else 1


if __name__ == "__main__":
    sys.exit(main())
