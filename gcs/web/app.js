// VTOL ground station cockpit: sticks/gamepad/keyboard -> websocket -> sim;
// JPEG frames + telemetry back. No external libraries.
"use strict";

const $ = (id) => document.getElementById(id);
const state = { tm: null, landmarks: [], track: [], connected: false };
const sticks = { thr: 0, yaw: 0, pitch: 0, roll: 0 };
let ws = null;

// ------------------------------------------------------------ connection --
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = "blob";
  ws.onopen = () => { state.connected = true; setPill("p-link", "linked", "on"); };
  ws.onclose = () => {
    state.connected = false; setPill("p-link", "offline", "bad");
    setTimeout(connect, 1500);
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data !== "string") { showFrame(ev.data); return; }
    const m = JSON.parse(ev.data);
    if (m.type === "hello") { state.landmarks = m.landmarks || []; }
    else if (m.type === "telemetry") {
      state.tm = m;
      if (m.track) state.track = m.track;
      else if (m.pos) state.track.push([m.pos[0], m.pos[1]]);
      if (state.track.length > 4000) state.track.splice(0, 1000);
      updateUI(m);
    }
  };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

let lastUrl = null;
function showFrame(blob) {
  const url = URL.createObjectURL(blob);
  const img = $("video");
  img.onload = () => { if (lastUrl) URL.revokeObjectURL(lastUrl); lastUrl = url; };
  img.src = url;
}

// ------------------------------------------------------------------ store --
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v ? JSON.parse(v) : d; } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* private mode */ } },
};

// ---------------------------------------------------------------- sticks --
// Mode 2: left = throttle (vertical) + yaw (horizontal); right = pitch + roll.
function makeStick(el, onMove, opts) {
  const knob = el.querySelector(".knob");
  let active = null;
  const pos = { x: 0, y: opts.yStart || 0 };
  function draw() {
    knob.style.left = `${50 + pos.x * 42}%`;
    knob.style.top = `${50 - pos.y * 42}%`;
  }
  function fromEvent(e) {
    const r = el.getBoundingClientRect();
    pos.x = Math.max(-1, Math.min(1, ((e.clientX - r.left) / r.width - 0.5) / 0.42));
    pos.y = Math.max(-1, Math.min(1, -((e.clientY - r.top) / r.height - 0.5) / 0.42));
    draw(); onMove(pos.x, pos.y);
  }
  el.addEventListener("pointerdown", (e) => { active = e.pointerId; el.setPointerCapture(e.pointerId); fromEvent(e); });
  el.addEventListener("pointermove", (e) => { if (e.pointerId === active) fromEvent(e); });
  const end = (e) => {
    if (e.pointerId !== active) return;
    active = null;
    pos.x = 0;
    if (opts.springY()) pos.y = 0;
    draw(); onMove(pos.x, pos.y);
  };
  el.addEventListener("pointerup", end);
  el.addEventListener("pointercancel", end);
  draw();
  return {
    set(x, y) { if (active === null) { pos.x = x; pos.y = y; draw(); } },
    get touching() { return active !== null; },
    resetY(y) { pos.y = y; draw(); },
  };
}

// in ANGLE mode the throttle stick does not spring back (like a real radio)
const angleMode = () => state.tm && state.tm.manual_mode === "ANGLE" && state.tm.mode === "MANUAL";
const stickL = makeStick($("stick-l"), (x, y) => { sticks.yaw = x; sticks.thr = y; },
                         { springY: () => !angleMode() });
const stickR = makeStick($("stick-r"), (x, y) => { sticks.roll = x; sticks.pitch = y; },
                         { springY: () => true });

// -------------------------------------------------------------- keyboard --
const keys = new Set();
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
  if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", " "].includes(e.key)) e.preventDefault();
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  keys.add(e.key.startsWith("Arrow") ? e.key : k);
  if (k === "t") toggleArm();
  if (k === "m") setMode(state.tm && state.tm.mode === "AUTO" ? "MANUAL" : "AUTO");
  if (k === "v") setVtol(!(state.tm && state.tm.fw));
  if (k === "p") send({ type: "release" });
});
window.addEventListener("keyup", (e) => keys.delete(e.key.startsWith("Arrow") ? e.key : e.key.toLowerCase()));
function keyboardSticks(dt) {
  if (!keys.size) return false;
  const ax = (pos, neg) => (keys.has(pos) ? 1 : 0) - (keys.has(neg) ? 1 : 0);
  sticks.roll = ax("ArrowRight", "ArrowLeft") * 0.7;
  sticks.pitch = ax("ArrowUp", "ArrowDown") * 0.7;
  sticks.yaw = ax("d", "a") * 0.7;
  const w = ax("w", "s");
  if (angleMode()) sticks.thr = Math.max(-1, Math.min(1, sticks.thr + w * dt * 0.8));
  else sticks.thr = w * 0.7;
  stickL.set(sticks.yaw, sticks.thr); stickR.set(sticks.roll, sticks.pitch);
  return true;
}

// --------------------------------------------------------------- gamepad --
const CH = ["thr", "yaw", "pitch", "roll"];
let gpMap = store.get("gpMap", { thr: [2, false], yaw: [3, false], pitch: [1, true], roll: [0, false] });
function buildGpMap() {
  const box = $("gp-map"); box.innerHTML = "";
  for (const ch of CH) {
    const l = document.createElement("label");
    l.innerHTML = `<b style="width:44px">${ch.toUpperCase()}</b> axis <select>${
      [0, 1, 2, 3, 4, 5, 6, 7].map((i) => `<option ${gpMap[ch][0] === i ? "selected" : ""}>${i}</option>`).join("")
    }</select><input type="checkbox" ${gpMap[ch][1] ? "checked" : ""}> inv`;
    const [sel, inv] = [l.querySelector("select"), l.querySelector("input")];
    sel.onchange = () => { gpMap[ch][0] = +sel.value; store.set("gpMap", gpMap); };
    inv.onchange = () => { gpMap[ch][1] = inv.checked; store.set("gpMap", gpMap); };
    box.appendChild(l);
  }
}
buildGpMap();
function gamepadSticks() {
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  $("gp-status").textContent = pads.length ? pads[0].id.slice(0, 40) : "none";
  if (!pads.length) return false;
  const gp = pads[0];
  const axBox = $("gp-axes");
  if (axBox.childElementCount !== gp.axes.length) {
    axBox.innerHTML = gp.axes.map((_, i) => `<div class="ax">ax ${i}<div class="bar"><i></i></div></div>`).join("");
  }
  gp.axes.forEach((v, i) => {
    const bar = axBox.children[i].querySelector("i");
    bar.style.left = `${50 + Math.min(v, 0) * 50}%`; bar.style.width = `${Math.abs(v) * 50}%`;
  });
  if (!$("gp-use").checked) return false;
  if (stickL.touching || stickR.touching) return false;
  for (const ch of CH) {
    const [i, inv] = gpMap[ch];
    let v = gp.axes[i] ?? 0;
    if (inv) v = -v;
    if (Math.abs(v) < 0.04) v = 0;                        // deadband
    sticks[ch] = v;
  }
  stickL.set(sticks.yaw, sticks.thr); stickR.set(sticks.roll, sticks.pitch);
  return true;
}

// ------------------------------------------------------------- send loop --
let tPrev = performance.now();
function inputLoop() {
  const now = performance.now(), dt = (now - tPrev) / 1000; tPrev = now;
  if (!gamepadSticks()) keyboardSticks(dt);
  requestAnimationFrame(inputLoop);
}
requestAnimationFrame(inputLoop);
setInterval(() => send({ type: "sticks", ...sticks }), 33);

// -------------------------------------------------------------- switches --
function toggleArm() { send({ type: "arm", on: !(state.tm && state.tm.armed) }); }
function setMode(m) { send({ type: "mode", mode: m }); }
function setVtol(fw) { send({ type: "vtol", fw }); }
$("sw-arm").onclick = toggleArm;
function seg(id, fn) {
  $(id).querySelectorAll("button").forEach((b) => (b.onclick = () => fn(b.dataset.v)));
}
seg("sw-mode", setMode);
seg("sw-man", (v) => {
  send({ type: "manual_mode", mode: v });
  if (v === "ANGLE") { sticks.thr = -1; stickL.resetY(-1); } else { sticks.thr = 0; stickL.resetY(0); }
});
seg("sw-vtol", (v) => setVtol(v === "FW"));
seg("sw-rl", (v) => send({ type: "rl", on: v === "1" }));
$("btn-drop").onclick = () => send({ type: "release" });
$("btn-reset").onclick = () => { send({ type: "reset", wind: +$("wind").value }); state.track = []; };
let paused = false;
$("btn-pause").onclick = () => { paused = !paused; send({ type: "pause", on: paused }); };
$("btn-start").onclick = () => send({ type: "start", x: +$("dx").value, y: +$("dy").value });
document.querySelectorAll(".presets button").forEach((b) => (b.onclick = () => {
  const [x, y] = b.dataset.p.split(",").map(Number); $("dx").value = x; $("dy").value = y; drawMap();
}));
document.querySelectorAll(".cam-switch button[data-cam]").forEach((b) => (b.onclick = () => send({ type: "cam", mode: b.dataset.cam })));
$("btn-fs").onclick = () => {
  const el = document.documentElement;
  if (document.fullscreenElement) document.exitFullscreen();
  else if (el.requestFullscreen) el.requestFullscreen().catch(() => {});
};

function setSeg(id, v) {
  $(id).querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.v === v));
}
function setPill(id, text, cls) { const p = $(id); p.textContent = text; p.className = `pill ${cls || ""}`; }

// ---------------------------------------------------------------- phases --
const PHASES_FW = ["CLIMB", "ALIGN", "SETTLE", "TRANSITION", "CRUISE", "BACK", "APPROACH", "DESCEND", "ON_PAD"];
const PHASES_MC = ["CLIMB", "ALIGN", "SETTLE", "MC_CRUISE", "APPROACH", "DESCEND", "ON_PAD"];
function drawPhases(tm) {
  const ol = $("phases");
  if (tm.mode !== "AUTO" || (!tm.dest && tm.phase !== "COMPLETE")) { ol.innerHTML = ""; return; }
  const dist = tm.dest ? Math.hypot(tm.dest[0] - tm.home[0], tm.dest[1] - tm.home[1]) : 0;
  const list = dist >= 300 ? PHASES_FW : PHASES_MC;
  const i = list.indexOf(tm.phase);
  const leg = tm.payload ? "Leg 1 → destination" : "Leg 2 → home";
  ol.innerHTML = `<li class="now">${tm.phase === "COMPLETE" ? "COMPLETE" : leg}</li>` +
    list.map((p, k) => `<li class="${k < i ? "done" : k === i ? "now" : ""}">${p}</li>`).join("");
}

// -------------------------------------------------------------------- UI --
function updateUI(tm) {
  setPill("p-arm", tm.crashed ? "CRASHED" : tm.armed ? "ARMED" : "DISARMED",
          tm.crashed ? "bad" : tm.armed ? "bad" : "off");
  setPill("p-mode", tm.mode === "AUTO" ? "AUTO" : `MANUAL · ${tm.fw ? "WING" : tm.manual_mode}`, "acc");
  setPill("p-phase", tm.phase, "dim");
  setPill("p-rl", tm.rl_active ? "RL residual ACTIVE" : tm.rl_assist ? (tm.rl_policy ? "RL armed" : "RL: no policy") : "RL off",
          tm.rl_active ? "on" : "dim");
  setPill("p-batt", `${tm.batt_v.toFixed(1)} V · ${tm.soc.toFixed(0)}%`, tm.soc < 25 ? "warn" : "dim");
  setPill("p-link", `linked · ${tm.paused ? "paused" : `${tm.rt.toFixed(2)}× rt · ${tm.fps} fps`}`, tm.rt < 0.8 ? "warn" : "on");
  const armBtn = $("sw-arm");
  armBtn.textContent = tm.armed ? "ARMED" : "DISARMED";
  armBtn.classList.toggle("armed", tm.armed);
  setSeg("sw-mode", tm.mode); setSeg("sw-man", tm.manual_mode);
  setSeg("sw-vtol", tm.fw ? "FW" : "MC"); setSeg("sw-rl", tm.rl_assist ? "1" : "0");
  $("btn-drop").disabled = !tm.payload;
  $("btn-drop").textContent = tm.payload ? "DROP" : "delivered";
  document.querySelectorAll(".cam-switch button").forEach((b) => b.classList.toggle("on", b.dataset.cam === tm.cam));
  $("toast").textContent = tm.message || "";
  drawPhases(tm);
  const T = [
    ["Alt", `${tm.pos[2].toFixed(1)} m`], ["Vz", `${tm.vz.toFixed(1)} m/s`],
    ["Airspeed", `${tm.airspeed.toFixed(1)} m/s`], ["Ground spd", `${tm.gs.toFixed(1)} m/s`],
    ["E / N", `${tm.pos[0].toFixed(0)} / ${tm.pos[1].toFixed(0)}`], ["Heading", `${((90 - tm.yaw + 360) % 360).toFixed(0)}°`],
    ["Tilt L/R", `${tm.tilt[0]}° / ${tm.tilt[1]}°`], ["Tail tilt", `${tm.tilt[2]}°`],
    ["Thrust L/R/T", `${tm.thrust.map((v) => v.toFixed(0)).join("/")} N`], ["Battery", `${tm.batt_v.toFixed(2)} V`],
    ["Payload", tm.payload ? "on board" : "released"], ["To target", tm.dist_to == null ? "—" : `${tm.dist_to.toFixed(0)} m`],
    ["Wind", `${Math.hypot(...tm.wind).toFixed(1)} m/s`], ["Sim time", `${tm.t.toFixed(0)} s`],
  ];
  $("tele").innerHTML = T.map(([k, v]) => `<div><b>${k}</b><span>${v}</span></div>`).join("") +
    `<div style="grid-column:1/-1"><b>Policy</b><span>${tm.rl_policy || "none (baseline only)"}</span></div>`;
  drawMap();
}

// ---------------------------------------------------------------- canvas --
function fitCanvas(c) {
  const r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (c.width !== Math.round(r.width * dpr) || c.height !== Math.round(r.height * dpr)) {
    c.width = Math.round(r.width * dpr); c.height = Math.round(r.height * dpr);
  }
  const ctx = c.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [ctx, r.width, r.height];
}

let mapView = { cx: 0, cy: 0, scale: 1 };
function drawMap() {
  const c = $("map"); const [ctx, W, H] = fitCanvas(c);
  const tm = state.tm;
  const pts = [[0, 0], [+$("dx").value, +$("dy").value]];
  if (tm) { pts.push([tm.pos[0], tm.pos[1]]); if (tm.dest) pts.push(tm.dest); pts.push(tm.home); }
  let [x0, x1, y0, y1] = [Infinity, -Infinity, Infinity, -Infinity];
  for (const [x, y] of pts) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
  const span = Math.max(x1 - x0, y1 - y0, 80) * 1.35;
  mapView = { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, scale: Math.min(W, H) / span };
  const P = (x, y) => [W / 2 + (x - mapView.cx) * mapView.scale, H / 2 - (y - mapView.cy) * mapView.scale];

  ctx.fillStyle = "#0a0f15"; ctx.fillRect(0, 0, W, H);
  // grid
  const step = [10, 25, 50, 100, 250, 500, 1000].find((s) => s * mapView.scale > 45) || 1000;
  ctx.strokeStyle = "#17202a"; ctx.lineWidth = 1; ctx.font = "10px ui-monospace, monospace"; ctx.fillStyle = "#4b5866";
  const gx0 = Math.floor((mapView.cx - W / 2 / mapView.scale) / step) * step;
  const gy0 = Math.floor((mapView.cy - H / 2 / mapView.scale) / step) * step;
  for (let g = gx0; g < mapView.cx + W / 2 / mapView.scale; g += step) {
    const [px] = P(g, 0); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke(); ctx.fillText(g, px + 2, H - 4);
  }
  for (let g = gy0; g < mapView.cy + H / 2 / mapView.scale; g += step) {
    const [, py] = P(0, g); ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke(); ctx.fillText(g, 2, py - 2);
  }
  // landmarks
  ctx.fillStyle = "#3a434d";
  for (const [x, y] of state.landmarks) { const [px, py] = P(x, y); ctx.fillRect(px - 3, py - 3, 6, 6); }
  // planned leg + 300 m fixed-wing ring
  const [hx, hy] = P(0, 0);
  ctx.strokeStyle = "rgba(63,167,255,.25)"; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.arc(hx, hy, 300 * mapView.scale, 0, 2 * Math.PI); ctx.stroke();
  const [dxp, dyp] = P(+$("dx").value, +$("dy").value);
  ctx.strokeStyle = "rgba(240,136,62,.6)";
  ctx.beginPath(); ctx.moveTo(hx, hy); ctx.lineTo(dxp, dyp); ctx.stroke(); ctx.setLineDash([]);
  // track
  if (state.track.length > 1) {
    ctx.strokeStyle = "#3fb950"; ctx.lineWidth = 1.5; ctx.beginPath();
    state.track.forEach(([x, y], i) => { const [px, py] = P(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke();
  }
  // pads
  const pad = (x, y, col, txt) => {
    const [px, py] = P(x, y); ctx.fillStyle = col; ctx.beginPath(); ctx.arc(px, py, 6, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = "#e6edf3"; ctx.fillText(txt, px + 8, py - 6);
  };
  pad(0, 0, "#3fa7ff", "HOME");
  pad(+$("dx").value, +$("dy").value, "#f0883e", `DEST (${$("dx").value}, ${$("dy").value})`);
  // vehicle
  if (tm) {
    const [px, py] = P(tm.pos[0], tm.pos[1]); const a = -tm.yaw * Math.PI / 180;
    ctx.save(); ctx.translate(px, py); ctx.rotate(a);
    ctx.fillStyle = tm.crashed ? "#f85149" : "#ffffff";
    ctx.beginPath(); ctx.moveTo(10, 0); ctx.lineTo(-7, 6); ctx.lineTo(-4, 0); ctx.lineTo(-7, -6); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
}
$("map").addEventListener("click", (e) => {
  const r = $("map").getBoundingClientRect();
  const x = mapView.cx + (e.clientX - r.left - r.width / 2) / mapView.scale;
  const y = mapView.cy - (e.clientY - r.top - r.height / 2) / mapView.scale;
  $("dx").value = Math.round(x / 5) * 5; $("dy").value = Math.round(y / 5) * 5;
  drawMap();
});
["dx", "dy"].forEach((id) => $(id).addEventListener("input", drawMap));

// ------------------------------------------------------------------- HUD --
function drawHUD() {
  const c = $("hud"); const [ctx, W, H] = fitCanvas(c);
  ctx.clearRect(0, 0, W, H);
  const tm = state.tm;
  if (tm) {
    const cx = W / 2, cy = H / 2, ppd = H / 60;              // pixels per degree
    const g = "rgba(120,255,160,.9)";
    ctx.strokeStyle = g; ctx.fillStyle = g; ctx.lineWidth = 1.5; ctx.font = "12px ui-monospace, monospace";
    // pitch ladder rotated by roll
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(-tm.roll * Math.PI / 180);
    for (let p = -30; p <= 30; p += 10) {
      const y = (tm.pitch - p) * ppd; const w = p === 0 ? 120 : 45;
      if (Math.abs(y) > H * 0.35) continue;
      ctx.beginPath(); ctx.moveTo(-w, y); ctx.lineTo(-18, y); ctx.moveTo(18, y); ctx.lineTo(w, y); ctx.stroke();
      if (p) { ctx.fillText(p, w + 4, y + 4); }
    }
    ctx.restore();
    // aircraft symbol
    ctx.beginPath(); ctx.moveTo(cx - 30, cy); ctx.lineTo(cx - 10, cy); ctx.lineTo(cx, cy + 8);
    ctx.lineTo(cx + 10, cy); ctx.lineTo(cx + 30, cy); ctx.stroke();
    // tapes
    const box = (x, label, val) => {
      ctx.fillStyle = "rgba(0,0,0,.45)"; ctx.fillRect(x - 38, cy - 16, 76, 32);
      ctx.strokeRect(x - 38, cy - 16, 76, 32); ctx.fillStyle = g;
      ctx.textAlign = "center"; ctx.fillText(val, x, cy + 5); ctx.fillText(label, x, cy - 22); ctx.textAlign = "left";
    };
    box(70, "AIRSPD m/s", tm.airspeed.toFixed(1));
    box(W - 70, "ALT m", tm.pos[2].toFixed(1));
    ctx.textAlign = "center";
    ctx.fillText(`VS ${tm.vz >= 0 ? "+" : ""}${tm.vz.toFixed(1)}`, W - 70, cy + 34);
    ctx.fillText(`GS ${tm.gs.toFixed(1)}`, 70, cy + 34);
    // heading (compass, 0 = north)
    const hdg = (90 - tm.yaw + 360) % 360;
    ctx.fillStyle = "rgba(0,0,0,.45)"; ctx.fillRect(cx - 34, 8, 68, 22); ctx.fillStyle = g;
    ctx.fillText(`${hdg.toFixed(0).padStart(3, "0")}°`, cx, 24);
    // phone pilot view: sticks/switches cover the bottom, buttons the top right
    const compact = H < 520;
    const yMode = compact ? 48 : H - 44;
    ctx.fillText(`${tm.mode}${tm.mode === "MANUAL" ? " · " + (tm.fw ? "WING" : tm.manual_mode) : ""} · ${tm.phase}`, cx, yMode);
    ctx.textAlign = "left";
    // tilt gauge
    const yTilt = compact ? 48 : H - 44;
    ctx.fillText(`NACELLE ${tm.tilt[0]}°`, 12, yTilt);
    ctx.fillRect(12, yTilt + 8, 100 * Math.min(tm.tilt[0], 90) / 90, 4);
    ctx.strokeRect(12, yTilt + 8, 100, 4);
    if (tm.rl_active) {
      const [bx, by] = compact ? [12, 66] : [W - 150, 40];
      ctx.fillStyle = "rgba(63,185,80,.85)"; ctx.fillRect(bx, by, 138, 20);
      ctx.fillStyle = "#04111f"; ctx.fillText("RL ASSIST ACTIVE", bx + 10, by + 14);
    }
    if (tm.crashed) {
      ctx.fillStyle = "rgba(248,81,73,.9)"; ctx.font = "bold 28px system-ui"; ctx.textAlign = "center";
      ctx.fillText("CRASH — press RESET SIM", cx, cy - 60); ctx.textAlign = "left";
    }
  }
  requestAnimationFrame(drawHUD);
}
requestAnimationFrame(drawHUD);
window.addEventListener("resize", drawMap);

connect();
drawMap();
