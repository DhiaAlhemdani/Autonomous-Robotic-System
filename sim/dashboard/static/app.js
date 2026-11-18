// T800 planar NMPC dashboard. Plain canvas, no external libraries.
// All URLs are relative so the page works behind the sandbox preview proxy.
(function () {
  "use strict";

  const WINDOW = 600;                       // points kept per chart
  const X0 = -1, X1 = 9, Y0 = -4, Y1 = 4;   // scene extent, metres
  const S = {
    runId: null, meta: null, steps: [], done: null, error: null,
    trail: [], dMin: 1.0, dMinObj: 0.5,
  };

  const $ = (id) => document.getElementById(id);
  const fmt = (x, d = 2) => (x === null || x === undefined || Number.isNaN(x)) ? "—" : Number(x).toFixed(d);

  function reset(meta) {
    S.runId = meta.run_id; S.meta = meta; S.steps = []; S.done = null; S.error = null;
    S.trail = []; S.dMin = meta.d_min_person; S.dMinObj = meta.d_min_object;
    $("state").textContent = "running";
    $("start").disabled = true;
  }

  function onStep(ev) {
    S.steps.push(ev);
    if (S.steps.length > WINDOW * 4) S.steps.splice(0, S.steps.length - WINDOW * 4);
    S.trail.push(ev.pelvis);
    if (S.trail.length > 2000) S.trail.shift();
  }

  function onMessage(msg) {
    if (msg.type === "meta") reset(msg);
    else if (msg.type === "step") onStep(msg);
    else if (msg.type === "done") { S.done = msg.result; $("state").textContent = msg.result.stopped ? "stopped" : "finished"; $("start").disabled = false; }
    else if (msg.type === "error") { S.error = msg.message; $("state").textContent = "error"; $("start").disabled = false; }
    draw();
  }

  function connect() {
    const proto = location.protocol === "https:" ? "wss://" : "ws://";
    const ws = new WebSocket(proto + location.host + "/ws");
    ws.onopen = () => { $("conn").textContent = "live"; $("conn").className = "pill ok"; };
    ws.onclose = () => { $("conn").textContent = "reconnecting…"; $("conn").className = "pill bad"; setTimeout(connect, 1000); };
    ws.onmessage = (e) => onMessage(JSON.parse(e.data));
  }

  async function startRun() {
    const seed = parseInt($("seed").value, 10) || 0;
    const maxTime = parseFloat($("maxtime").value) || 30;
    $("start").disabled = true;
    $("state").textContent = "starting…";
    const r = await fetch(`api/run?seed=${seed}&max_time=${maxTime}`, { method: "POST" });
    if (!r.ok) { $("state").textContent = "rejected"; $("start").disabled = false; }
  }

  // ---- scene ----
  function scene() {
    const c = $("scene"), g = c.getContext("2d");
    const W = c.width, H = c.height;
    const sx = (x) => (x - X0) / (X1 - X0) * W;
    const sy = (y) => H - (y - Y0) / (Y1 - Y0) * H;
    const sr = (r) => r / (X1 - X0) * W;
    g.clearRect(0, 0, W, H);
    // grid
    g.strokeStyle = "#edf0f5"; g.lineWidth = 1;
    for (let x = 0; x <= X1; x++) { g.beginPath(); g.moveTo(sx(x), 0); g.lineTo(sx(x), H); g.stroke(); }
    for (let y = -4; y <= 4; y++) { g.beginPath(); g.moveTo(0, sy(y)); g.lineTo(W, sy(y)); g.stroke(); }
    if (!S.steps.length && !S.meta) { g.fillStyle = "#6b7585"; g.fillText("Start an episode", 12, 20); return; }
    const last = S.steps[S.steps.length - 1];
    // goal
    const goal = (last && last.goal) || (S.done && S.done.goal_xy);
    if (goal) {
      g.fillStyle = "#e0a100"; g.beginPath(); g.arc(sx(goal[0]), sy(goal[1]), 8, 0, 2 * Math.PI); g.fill();
    }
    // trail
    if (S.trail.length > 1) {
      g.strokeStyle = "#1f6feb"; g.globalAlpha = 0.35; g.lineWidth = 2; g.beginPath();
      S.trail.forEach((p, i) => { i ? g.lineTo(sx(p[0]), sy(p[1])) : g.moveTo(sx(p[0]), sy(p[1])); });
      g.stroke(); g.globalAlpha = 1;
    }
    if (!last) return;
    // objects
    g.fillStyle = "#2a9d8f";
    last.objects.forEach((o) => { g.beginPath(); g.arc(sx(o[0]), sy(o[1]), sr(0.3), 0, 2 * Math.PI); g.fill(); });
    // persons + keep-out rings
    last.persons.forEach((p) => {
      g.strokeStyle = "rgba(209,67,59,0.5)"; g.setLineDash([4, 4]); g.lineWidth = 1;
      g.beginPath(); g.arc(sx(p[0]), sy(p[1]), sr(S.dMin), 0, 2 * Math.PI); g.stroke(); g.setLineDash([]);
      g.fillStyle = "#d1433b"; g.beginPath(); g.arc(sx(p[0]), sy(p[1]), sr(0.3), 0, 2 * Math.PI); g.fill();
    });
    // robot
    g.fillStyle = last.violation ? "#c62828" : "#1f6feb";
    g.beginPath(); g.arc(sx(last.pelvis[0]), sy(last.pelvis[1]), 9, 0, 2 * Math.PI); g.fill();
    g.strokeStyle = "#1f6feb"; g.lineWidth = 2; g.beginPath();
    g.moveTo(sx(last.pelvis[0]), sy(last.pelvis[1]));
    g.lineTo(sx(last.pelvis[0] + last.accel[0] * 0.5), sy(last.pelvis[1] + last.accel[1] * 0.5));
    g.stroke();
  }

  // ---- charts ----
  function chart(id, series, opts) {
    const c = $(id), g = c.getContext("2d");
    const W = c.width, H = c.height, pad = 34;
    g.clearRect(0, 0, W, H);
    const pts = S.steps.slice(-WINDOW);
    if (!pts.length) return;
    const xs = pts.map((p) => p.t);
    const ys = series.flatMap((s) => pts.map(s.get)).filter((v) => v !== null && v !== undefined && Number.isFinite(v));
    let lo = Math.min(...ys, opts.min ?? Infinity), hi = Math.max(...ys, opts.max ?? -Infinity);
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) return;
    if (hi - lo < 1e-9) { hi = lo + 1; }
    const x0 = xs[0], x1 = xs[xs.length - 1] || x0 + 1;
    const X = (x) => pad + (x - x0) / Math.max(1e-9, x1 - x0) * (W - pad - 8);
    const Y = (y) => H - pad - (y - lo) / (hi - lo) * (H - pad - 8);
    g.strokeStyle = "#d9dee7"; g.fillStyle = "#6b7585"; g.font = "11px system-ui"; g.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const v = lo + (hi - lo) * i / 4, y = Y(v);
      g.beginPath(); g.moveTo(pad, y); g.lineTo(W - 8, y); g.stroke();
      g.fillText(v.toFixed(opts.dp ?? 1), 2, y + 4);
    }
    g.fillText(`t = ${fmt(x1, 1)} s`, W - 70, H - 6);
    (opts.hlines || []).forEach((h) => {
      g.strokeStyle = h.color; g.setLineDash([5, 4]); g.beginPath(); g.moveTo(pad, Y(h.v)); g.lineTo(W - 8, Y(h.v)); g.stroke(); g.setLineDash([]);
      g.fillStyle = h.color; g.fillText(h.label, W - 8 - g.measureText(h.label).width, Y(h.v) - 3);
    });
    series.forEach((s) => {
      g.strokeStyle = s.color; g.lineWidth = 1.5; g.beginPath();
      let started = false;
      pts.forEach((p) => {
        const v = s.get(p);
        if (v === null || v === undefined || !Number.isFinite(v)) return;
        started ? g.lineTo(X(p.t), Y(v)) : g.moveTo(X(p.t), Y(v));
        started = true;
      });
      g.stroke();
    });
    // legend
    let lx = pad + 4;
    series.forEach((s) => { g.fillStyle = s.color; g.fillRect(lx, 8, 10, 3); g.fillStyle = "#1c2330"; g.fillText(s.label, lx + 14, 13); lx += 16 + g.measureText(s.label).width + 12; });
  }

  function quantile(arr, q) {
    if (!arr.length) return null;
    const a = arr.slice().sort((x, y) => x - y);
    return a[Math.min(a.length - 1, Math.floor(q * (a.length - 1)))];
  }

  function kpis() {
    const st = S.steps;
    const last = st[st.length - 1];
    if (!last) return;
    $("k-time").textContent = `${fmt(last.t, 1)} s`;
    $("k-rtf").textContent = last.wall_s > 0 ? fmt(last.t / last.wall_s, 2) + "×" : "—";
    const maxTilt = Math.max(...st.map((p) => p.tilt_deg));
    $("k-tilt").textContent = fmt(maxTilt, 1) + "°";
    $("k-tilt").className = "v " + (maxTilt < 20 ? "ok" : "bad");
    const minP = Math.min(...st.map((p) => p.dist_person));
    $("k-person").textContent = fmt(minP, 2) + " m";
    $("k-person").className = "v " + (minP >= S.dMin ? "ok" : "bad");
    $("k-person-s").textContent = `d_min ${fmt(S.dMin, 2)} m (ASSUMPTION)`;
    const viol = st.filter((p) => p.violation).length;
    $("k-viol").textContent = viol;
    $("k-viol").className = "v " + (viol === 0 ? "ok" : "bad");
    const sv = st.map((p) => p.solve_ms);
    $("k-solve").textContent = `${fmt(quantile(sv, 0.5), 0)} / ${fmt(quantile(sv, 0.99), 0)}`;
    const inf = st.filter((p) => p.nmpc_status === "infeasible").length;
    $("k-inf").textContent = fmt(100 * inf / st.length, 1) + "%";
    const te = st.map((p) => p.track_err_m * 1000);
    $("k-track").textContent = fmt(quantile(te, 0.5), 1) + " mm";
  }

  function draw() {
    scene();
    chart("c-solve", [{ label: "solve ms", color: "#6a4fb3", get: (p) => p.solve_ms }], { min: 0 });
    chart("c-tilt", [{ label: "tilt°", color: "#1f6feb", get: (p) => p.tilt_deg }], { min: 0, max: 20, hlines: [{ v: 20, color: "#c62828", label: "fall limit 20°" }] });
    chart("c-person", [{ label: "person", color: "#d1433b", get: (p) => p.dist_person }, { label: "object", color: "#2a9d8f", get: (p) => p.dist_object }], { min: 0, hlines: [{ v: S.dMin, color: "#d1433b", label: `d_min person ${fmt(S.dMin, 2)}` }], dp: 2 });
    chart("c-speed", [{ label: "speed m/s", color: "#2e7d32", get: (p) => p.speed }, { label: "track err mm", color: "#e0a100", get: (p) => p.track_err_m * 1000 }], { min: 0 });
    chart("c-accel", [{ label: "a_x", color: "#1f6feb", get: (p) => p.accel[0] }, { label: "a_y", color: "#d1433b", get: (p) => p.accel[1] }], { min: -1.5, max: 1.5 });
    kpis();
    if (S.error) $("state").textContent = "error: " + S.error.slice(0, 60);
  }

  $("start").addEventListener("click", startRun);
  draw();
  connect();
})();
