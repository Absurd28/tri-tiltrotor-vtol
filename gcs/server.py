#!/usr/bin/env python3
"""VTOL ground station: real-time simulation server + browser cockpit.

Run inside WSL (conda env drone311):
    MUJOCO_GL=egl python gcs/server.py            # then open http://localhost:8765
Options: --port 8765  --policy runs/s2_waypoint/final.onnx  --fps 25

One websocket (/ws) carries everything:
  browser -> server  JSON commands (sticks at ~30 Hz, arm, modes, mission...)
  server -> browser  binary JPEG frames (rendered by MuJoCo offscreen) and
                     JSON telemetry at 10 Hz
The simulation runs at real time (50 Hz policy/RL period, 100 Hz control,
500 Hz physics) whether or not a browser is connected; a delivery mission
keeps flying if the page is closed.
"""
from __future__ import annotations

import argparse
import asyncio
import io
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402
from websockets.asyncio.server import serve  # noqa: E402
from websockets.datastructures import Headers  # noqa: E402
from websockets.http11 import Response  # noqa: E402

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from gcs.sim_core import FlightSim, LANDMARKS  # noqa: E402
from control.pid import quat_to_euler  # noqa: E402

WEB = Path(__file__).parent / "web"
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript",
         ".css": "text/css", ".svg": "image/svg+xml", ".png": "image/png"}


class Station:
    def __init__(self, policy=None, fps=20, width=640, height=400):
        import torch
        torch.set_num_threads(1)            # tiny MLP; threads only contend
        self.sim = FlightSim(policy_path=policy)
        self.clients = set()
        self.fps = fps
        self.renderer = mujoco.Renderer(self.sim.model, height=height, width=width)
        # WSL renders in software: shadows/reflections cost ~4x per frame
        scn = self.renderer.scene
        scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        self.cam = mujoco.MjvCamera()
        self.vopt = mujoco.MjvOption()
        self.vopt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.cam_mode = "chase"
        self._az = 180.0
        self.paused = False
        self.rt_ratio = 1.0
        self._last_sticks = time.monotonic()

    # ------------------------------------------------------------ input --- #
    def handle(self, msg):
        sim = self.sim
        t = msg.get("type")
        if t == "sticks":
            for k in ("thr", "yaw", "pitch", "roll"):
                sim.sticks[k] = float(np.clip(msg.get(k, 0.0), -1, 1))
            self._last_sticks = time.monotonic()
        elif t == "arm":
            if msg.get("on"):
                if (sim.flight_mode == "MANUAL" and sim.manual_mode == "ANGLE"
                        and sim.sticks["thr"] > -0.8):
                    sim.message = "Arming refused: throttle not low."
                    return
                if (sim.flight_mode == "MANUAL" and sim.manual_mode == "POSITION"
                        and sim.sticks["thr"] > 0.15):
                    sim.message = "Arming refused: centre or lower the throttle."
                    return
            sim.arm(bool(msg.get("on")))
        elif t == "mode":
            sim.set_mode(msg.get("mode", "MANUAL"))
        elif t == "manual_mode":
            m = msg.get("mode", "POSITION")
            if sim.fw:
                sim.message = "Back-transition first."
            elif m in ("POSITION", "ANGLE"):
                sim.manual_mode = m
                sim.sp = sim.s["pos"].copy()
                sim.yaw_sp = quat_to_euler(sim.s["quat"])[2]
                sim.message = f"Manual {m}."
        elif t == "rl":
            sim.rl_assist = bool(msg.get("on"))
            if sim.rl_assist and not sim.policy.available:
                sim.message = "No trained policy found in runs/ - RL unavailable."
        elif t == "vtol":
            sim.vtol_switch(bool(msg.get("fw")))
        elif t == "start":
            sim.start_delivery((msg.get("x", 0), msg.get("y", 0)))
        elif t == "release":
            if sim.payload_attached:
                sim.release_payload()
        elif t == "reset":
            sim.reset(wind_ms=float(msg.get("wind", 0.0)))
        elif t == "cam":
            self.cam_mode = msg.get("mode", "chase")
        elif t == "pause":
            self.paused = bool(msg.get("on"))

    # ----------------------------------------------------------- render --- #
    def frame(self):
        s = self.sim.s
        cam = self.cam
        _, _, yaw = quat_to_euler(s["quat"])
        ydeg = np.degrees(yaw)
        # smooth the chase azimuth so the picture does not jitter with yaw
        d = (ydeg - self._az + 180) % 360 - 180
        self._az += 0.15 * d
        cam.lookat[:] = s["pos"]
        if self.cam_mode == "chase":
            cam.azimuth, cam.elevation, cam.distance = self._az, -12, 5.5
        elif self.cam_mode == "orbit":
            cam.azimuth, cam.elevation, cam.distance = 135, -20, 8.0
        elif self.cam_mode == "top":
            cam.azimuth, cam.elevation, cam.distance = 90, -89, 30.0
        elif self.cam_mode == "nose":
            # onboard camera module: just ahead of the nose, looking forward
            fwd = s["R_bw"][:, 0]
            cam.lookat[:] = s["pos"] + fwd * 6.0 + np.array([0, 0, -0.8])
            cam.azimuth, cam.elevation, cam.distance = ydeg, -8, 5.6
        self.renderer.update_scene(self.sim.veh.data, camera=cam,
                                   scene_option=self.vopt)
        img = self.renderer.render()
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, "JPEG", quality=72)
        return buf.getvalue()

    # ------------------------------------------------------------- loop --- #
    async def run(self):
        period = 0.02                                  # 50 Hz RL period
        min_every = max(1, round(50 / self.fps))
        frame_every = min_every
        k = 0
        t_next = time.perf_counter()
        t_win, n_win = time.perf_counter(), 0
        while True:
            # sticks time out to centre if the browser stops sending
            if time.monotonic() - self._last_sticks > 1.0:
                st = self.sim.sticks
                st["yaw"] = st["pitch"] = st["roll"] = 0.0
                if self.sim.manual_mode == "POSITION":
                    st["thr"] = 0.0
            try:
                if not self.paused:
                    self.sim.step()
                    n_win += 1
                k += 1
                if self.clients:
                    if k % frame_every == 0:
                        jpg = self.frame()
                        await self._broadcast(jpg)
                    if k % 5 == 0:
                        tm = self.sim.telemetry()
                        tm["type"] = "telemetry"
                        tm["rt"] = round(self.rt_ratio, 2)
                        tm["fps"] = round(getattr(self, "video_fps", self.fps))
                        tm["paused"] = self.paused
                        tm["cam"] = self.cam_mode
                        if k % 50 == 0:
                            tm["track"] = self.sim.track[-1500:]
                        await self._broadcast(json.dumps(tm))
            except Exception as e:                     # keep the station alive
                import traceback
                traceback.print_exc()
                self.sim.message = f"Internal error: {e} (press RESET SIM)"
                self.paused = True
            now = time.perf_counter()
            if now - t_win > 2.0:
                self.rt_ratio = n_win * period / (now - t_win)
                t_win, n_win = now, 0
                # physics keeps real time; the video frame rate gives way
                if self.rt_ratio < 0.97 and frame_every < 6:
                    frame_every += 1
                elif self.rt_ratio > 0.995 and frame_every > min_every:
                    frame_every -= 1
                self.video_fps = 50 / frame_every
            t_next += period
            delay = t_next - time.perf_counter()
            if delay < -0.5:                           # far behind: resync
                t_next = time.perf_counter()
                delay = 0
            await asyncio.sleep(max(delay, 0))

    async def _broadcast(self, data):
        dead = []
        for ws in list(self.clients):       # clients may join during awaits
            try:
                await ws.send(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def ws_handler(self, ws):
        self.clients.add(ws)
        await ws.send(json.dumps(dict(
            type="hello", landmarks=LANDMARKS,
            policy=self.sim.policy.name,
            limits=dict(min_fw=300, max_dist=2500))))
        try:
            async for raw in ws:
                try:
                    self.handle(json.loads(raw))
                except Exception as e:                 # bad message: report, carry on
                    self.sim.message = f"Command error: {e}"
        finally:
            self.clients.discard(ws)


def process_request(connection, request):
    """Serve the cockpit page; let /ws upgrade to a websocket."""
    path = request.path.split("?")[0]
    if path == "/ws":
        return None
    name = "index.html" if path in ("/", "") else path.lstrip("/")
    f = (WEB / name).resolve()
    if WEB.resolve() not in f.parents or not f.is_file():
        return connection.respond(404, "not found\n")
    body = f.read_bytes()
    headers = Headers([("Content-Type", TYPES.get(f.suffix, "application/octet-stream")),
                       ("Content-Length", str(len(body))),
                       ("Cache-Control", "no-store")])
    return Response(200, "OK", headers, body)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--policy", default=None)
    ap.add_argument("--fps", type=int, default=20)
    args = ap.parse_args()
    st = Station(policy=args.policy, fps=args.fps)
    print(f"RL policy: {st.sim.policy.name or 'none found (baseline only)'}")
    async with serve(st.ws_handler, args.host, args.port,
                     process_request=process_request, max_size=2 ** 20):
        print(f"Ground station running: open http://localhost:{args.port}")
        await st.run()


if __name__ == "__main__":
    asyncio.run(main())
