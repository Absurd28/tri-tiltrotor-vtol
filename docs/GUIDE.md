# Step-by-step guide: circuitry, model, simulation, flying (PC and phone)

## 0. Open the project terminal (every time)

1. Press **Start**, type **Ubuntu**, open it (or in PowerShell: `wsl -d Ubuntu-26.04`).
2. Run:
   ```bash
   source ~/miniconda3/bin/activate drone311
   cd "/mnt/c/Users/Aryan's_WorkSpace/Desktop/Drone_7th_sem"
   ```
   All commands below are typed in this terminal unless they say *Windows*.

## 1. See the circuitry

| What | How |
|---|---|
| Block diagram | *Windows:* open `hardware\block_diagram.svg` (double-click → opens in your browser) |
| Pin map, wiring rules, failsafes, bring-up checklist | `hardware\README.md` (open in VS Code / any Markdown viewer) |
| Parts list with masses and interfaces | `hardware\bom.csv` (opens in Excel) |
| Power/current/flight-time numbers | `python hardware/power_budget.py` |

Read the block diagram left → right: **battery → power module → PDB → 3 ESCs → motors**;
UBECs feed the servos and the companion computer; the **flight controller** (centre) takes the
sensors (top: GPS, airspeed, lidar; IMU/gyro inside the FC) and drives the 10 servos and 3 ESCs
(right); the **Pi Zero 2 W** (bottom) runs the RL policy and the camera; radio links go to the
ground (bottom row).

## 2. See the 3D model

- **Pictures:** *Windows:* open `docs\renders\` — 7 views (iso hover/cruise/mid, front, side, top, bottom).
- **Interactive 3D (opens a window on your desktop through WSLg):**
  ```bash
  python scripts/viewer_tilt.py
  ```
  Mouse: left-drag rotate, right-drag pan, scroll zoom, double-click to select a part.
  Keys: **Q/A** left nacelle ±5°, **W/S** right nacelle, **E/D** tail, **Z** hover pose, **X** cruise pose.
  If no window appears, use the pictures and videos instead.
- **Check the design numbers:** `python scripts/check_model.py` (mass, CG, inertia, prop clearance → ALL CHECKS PASS).

## 3. See the simulation working

- **Videos** (*Windows:* `docs\videos\`, double-click):
  - `mission_calm.mp4` — full flight: take-off → transition → cruise → back-transition → landing
  - `rl_hover_s1.mp4` — the trained RL policy holding a hover point
  - `baseline_obstacle.mp4` vs `rl_obstacle_G_seed10001.mp4` — same obstacle field without/with RL
- **Plots:** `eval\mission_calm.png` (altitude, speed, tilt vs time), `eval\tilt_corridor.png`.
- **Run it yourself:**
  ```bash
  python eval/run_mission.py --tag calm      # flies the mission, prints metrics, saves the plot
  pytest -m "not slow"                       # ~60 automatic checks of every physics block
  ```

## 4. Fly it on the PC

1. Start the ground station (keep this terminal open):
   ```bash
   bash scripts/run_gcs.sh
   ```
   Wait for `Ground station running: open http://localhost:8765`.
2. *Windows:* open **http://localhost:8765** in Chrome/Edge.
3. **First flight (easiest):** SC = **POSITION**, SE = **RL ON** → press **SA (ARM)** →
   push the **left stick up** (or **W**) to climb → let go to hold height →
   **right stick** (arrow keys) to fly around → left stick down to land → disarm.
4. **Harder:** SE = RL **OFF**; then SC = **ANGLE** (you control the height yourself;
   throttle stays where you leave it, mid-stick ≈ hover).
5. **Wing-borne flight:** climb above 8 m in POSITION, set SD = **WING** → it transitions;
   right stick steers, pull back to climb; SD = **HOVER** to come back.
6. **Automatic delivery:** SB = **AUTO** → click the map (or type East/North) → **Start delivery**.
   Watch the phase strip; it lands, drops the parcel, and flies home by itself.
7. Crashed? Press **RESET SIM**. Optional wind (2–4 m/s) is chosen next to it.
8. **Real RC transmitter:** plug it in by USB (joystick mode), move a stick, open
   *Gamepad / RC transmitter* and assign the axes.

Stop the server with **Ctrl+C** in the terminal.

## 5. Fly it from your phone

The server runs inside WSL, which other devices cannot reach by default. A one-time Windows
network setting is needed — run these yourself in **PowerShell as Administrator**
(Start → type PowerShell → right-click → Run as administrator):

1. Find your PC's Wi-Fi address: `ipconfig` → under *Wireless LAN adapter Wi-Fi* →
   **IPv4 Address**, e.g. `192.168.1.23` (use yours below).
2. Forward the port from Windows into WSL (the server must be running, step 4.1):
   ```powershell
   $wslIp = (wsl -d Ubuntu-26.04 -- hostname -I).Trim().Split(" ")[0]
   netsh interface portproxy add v4tov4 listenaddress=192.168.1.23 listenport=8765 connectaddress=$wslIp connectport=8765
   ```
3. Allow it through the firewall on your private network only:
   ```powershell
   New-NetFirewallRule -DisplayName "VTOL ground station" -Direction Inbound -Protocol TCP -LocalPort 8765 -Action Allow -Profile Private
   ```
   Your Wi-Fi must be set to **Private** (Settings → Network & internet → Wi-Fi → your
   network → Network profile type → Private).
4. On the phone (same Wi-Fi): open **http://192.168.1.23:8765** in Chrome.
5. **Turn the phone sideways** → the page switches to *pilot view*: full-screen video, sticks
   in the bottom corners, switches between them. Tap **⛶** (top right) for full screen
   (Android Chrome). Fly with your thumbs exactly as in step 4. For AUTO missions, turn the phone
   upright to see the map and the *Start delivery* button.

Notes
- The WSL address changes after a reboot: run the two lines of step 2 again (first remove the old
  rule with `netsh interface portproxy delete v4tov4 listenaddress=192.168.1.23 listenport=8765`).
- The page has no password: anyone on your Wi-Fi who knows the address can open it. Use it at home
  only. To undo everything:
  ```powershell
  netsh interface portproxy delete v4tov4 listenaddress=192.168.1.23 listenport=8765
  Remove-NetFirewallRule -DisplayName "VTOL ground station"
  ```
- The PC does all the simulation; the phone only shows video and sends stick positions, so any
  modern phone works. If the video stutters, move closer to the router.

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| `offline` pill in the page | the server isn't running — redo step 4.1 |
| "Arming refused: throttle not low" | ANGLE mode: pull the left stick fully down first |
| Pill shows `0.7× rt` | the PC is busy (e.g. training); close other programs |
| Phone can't connect | same Wi-Fi? network = Private? server running? redo step 5.2 after reboots |
| 3D viewer window doesn't appear | WSLg issue — run `wsl --update` in Windows, or use renders/videos |
