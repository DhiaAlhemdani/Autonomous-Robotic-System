"""Python dashboard (Dash + Plotly) for the T800 planar-base NMPC simulation.

Runs one episode at a time (sim/t800/planar_base.py) in a background thread. The page polls
the run state every 500 ms and redraws the scene and charts.

The rates shown are measured wall-clock rates on the machine that runs this. They are not
real-time guarantees and not target-hardware numbers.

Run from the repository root (after `git submodule update --init --depth 1 third_party/T800-PROJECT`):

    pip install -r sim/dashboard/requirements.txt
    python sim/dashboard/app.py --host 127.0.0.1 --port 8050

Use --host 0.0.0.0 to reach it from another machine, e.g. a laptop running the sandbox image.
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "t800"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "planning"))

import numpy as np  # noqa: E402
import plotly.graph_objects as go  # noqa: E402
from dash import Dash, Input, Output, State, ctx, dcc, html, no_update  # noqa: E402

import planar_base  # noqa: E402
from stage0.config import Stage0Config  # noqa: E402

X0, X1, Y0, Y1 = -1.0, 9.0, -4.0, 4.0
POLL_MS = 500


class RunManager:
    """Holds the current episode's events. Thread-safe."""

    def __init__(self):
        self.lock = threading.Lock()
        self.run_id = 0
        self.events = []
        self.running = False
        self.stop_run = None    # run_id that has been asked to stop
        self.error = None

    def start(self, seed, max_time):
        cfg = Stage0Config()
        with self.lock:
            if self.running:
                self.stop_run = self.run_id  # ask the previous run to stop
            self.run_id += 1
            run_id = self.run_id
            self.events = [{"type": "meta", "seed": seed, "max_time": max_time,
                            "d_min_person": cfg.d_min_person, "d_min_object": cfg.d_min_object,
                            "dt": cfg.dt}]
            self.running = True
            self.error = None
        args_ns = _Args(seed=seed, kp=600.0, kd=20.0, weld_solref=[0.02, 1.0], settle=2.0,
                        max_time=max_time)
        threading.Thread(target=self._worker, args=(run_id, args_ns), daemon=True).start()
        return run_id

    def stop(self):
        with self.lock:
            if self.running:
                self.stop_run = self.run_id

    def _worker(self, run_id, args_ns):
        t0 = time.perf_counter()

        def on_step(ev):
            with self.lock:
                if run_id == self.run_id:
                    ev = dict(ev, type="step", wall_s=round(time.perf_counter() - t0, 3))
                    self.events.append(ev)

        try:
            res = planar_base.run_episode(args_ns, on_step=on_step,
                                          stop=lambda: self.stop_run == run_id)
            res = {k: v for k, v in res.items() if k != "trace"}
            with self.lock:
                if run_id == self.run_id:
                    self.events.append({"type": "done", "result": res})
        except Exception as exc:  # shown in the page instead of killing the server
            with self.lock:
                self.error = repr(exc)
                if run_id == self.run_id:
                    self.events.append({"type": "error", "message": repr(exc)})
        finally:
            with self.lock:
                if run_id == self.run_id:
                    self.running = False

    def snapshot(self):
        with self.lock:
            return list(self.events), self.running, self.error, self.run_id


class _Args:
    def __init__(self, **kw):
        self.__dict__.update(kw)


runs = RunManager()


def _quantile(vals, q):
    return float(np.quantile(vals, q)) if len(vals) else None


def _fmt(x, d=2):
    return "—" if x is None else f"{x:.{d}f}"


def scene_figure(events):
    fig = go.Figure()
    meta = next((e for e in events if e["type"] == "meta"), None)
    steps = [e for e in events if e["type"] == "step"]
    fig.update_layout(
        margin=dict(l=40, r=20, t=30, b=40), height=380,
        xaxis=dict(range=[X0, X1], title="x (m)", zeroline=False),
        yaxis=dict(range=[Y0, Y1], title="y (m)", scaleanchor="x", scaleratio=1, zeroline=False),
        legend=dict(orientation="h", y=1.08), uirevision="scene",
        title=dict(text="Scene from above", x=0.01),
    )
    if meta is None:
        fig.add_annotation(text="Start an episode", x=4, y=0, showarrow=False)
        return fig
    if not steps:
        return fig
    last = steps[-1]
    goal = last["goal"]
    fig.add_trace(go.Scatter(x=[goal[0]], y=[goal[1]], mode="markers", name="goal",
                             marker=dict(size=14, color="#e0a100", symbol="star")))
    fig.add_trace(go.Scatter(x=[p[0] for p in last["objects"]], y=[p[1] for p in last["objects"]],
                             mode="markers", name="object",
                             marker=dict(size=14, color="#2a9d8f")))
    fig.add_trace(go.Scatter(x=[s["pelvis"][0] for s in steps], y=[s["pelvis"][1] for s in steps], mode="lines",
                             name="robot path", line=dict(color="#1f6feb", width=2),
                             opacity=0.4))
    fig.add_trace(go.Scatter(x=[p[0] for p in last["persons"]], y=[p[1] for p in last["persons"]],
                             mode="markers", name="person",
                             marker=dict(size=14, color="#d1433b")))
    for p in last["persons"]:
        fig.add_shape(type="circle", x0=p[0] - meta["d_min_person"], x1=p[0] + meta["d_min_person"],
                      y0=p[1] - meta["d_min_person"], y1=p[1] + meta["d_min_person"],
                      line=dict(color="rgba(209,67,59,0.5)", dash="dot"))
    px, py = last["pelvis"]
    ax, ay = last["accel"]
    fig.add_trace(go.Scatter(x=[px], y=[py], mode="markers", name="robot pelvis",
                             marker=dict(size=16, color="#c62828" if last["violation"] else "#1f6feb")))
    fig.add_annotation(x=px + 0.5 * ax, y=py + 0.5 * ay, ax=px, ay=py, xref="x", yref="y",
                       axref="x", ayref="y", showarrow=True, arrowhead=2, text="",
                       arrowcolor="#1f6feb")
    return fig


def line_chart(steps, title, series, yrange=None, hline=None, hline_label=""):
    fig = go.Figure()
    t = [s["t"] for s in steps]
    for name, color, key in series:
        fig.add_trace(go.Scatter(x=t, y=[key(s) for s in steps], mode="lines", name=name,
                                 line=dict(color=color, width=2)))
    if hline is not None:
        fig.add_hline(y=hline, line=dict(color="#c62828", dash="dash"), annotation_text=hline_label)
    fig.update_layout(title=dict(text=title, x=0.01), height=260, margin=dict(l=50, r=20, t=40, b=40),
                      xaxis_title="simulated time (s)", legend=dict(orientation="h", y=1.15),
                      uirevision=title)
    if yrange:
        fig.update_yaxes(range=list(yrange))
    return fig


def kpi_values(events, running, error):
    steps = [e for e in events if e["type"] == "step"]
    meta = next((e for e in events if e["type"] == "meta"), None)
    done = next((e["result"] for e in events if e["type"] == "done"), None)
    if not steps:
        return {"time": "—", "rtf": "—", "tilt": "—", "person": "—", "viol": "—", "solve": "—",
                "inf": "—", "track": "—", "state": error or ("starting" if running else "idle")}
    last = steps[-1]
    solve = [s["solve_ms"] for s in steps]
    inf = sum(1 for s in steps if s["nmpc_status"] == "infeasible")
    viol = sum(1 for s in steps if s["violation"])
    track = [s["track_err_m"] * 1000 for s in steps]
    rtf = last["t"] / last["wall_s"] if last.get("wall_s") else None
    if done is not None:
        state = "stopped" if done.get("stopped") else "finished"
    elif error:
        state = "error"
    else:
        state = "running"
    return {
        "time": f"{last['t']:.1f} s",
        "rtf": _fmt(rtf, 2) + "×" if rtf else "—",
        "tilt": f"{max(s['tilt_deg'] for s in steps):.1f}°",
        "person": f"{min(s['dist_person'] for s in steps):.2f} m (d_min {meta['d_min_person']:.2f})",
        "viol": str(viol),
        "solve": f"{_fmt(_quantile(solve, 0.5), 0)} / {_fmt(_quantile(solve, 0.99), 0)} ms",
        "inf": f"{100 * inf / len(steps):.1f}%",
        "track": f"{_fmt(_quantile(track, 0.5), 1)} mm",
        "state": state + (f" · goal reached" if done and done.get("goal_reached") else ""),
    }


def build_app():
    app = Dash(__name__, title="T800 planar NMPC dashboard")
    kpi_ids = ["time", "rtf", "tilt", "person", "viol", "solve", "inf", "track"]
    kpi_labels = {"time": "Sim time", "rtf": "Real-time factor (sandbox)", "tilt": "Max pelvis tilt (limit 20°)",
                  "person": "Closest person", "viol": "Violation steps", "solve": "NMPC solve p50 / p99",
                  "inf": "Infeasible steps (brake)", "track": "Tracking error (median)"}

    def card(i):
        return html.Div(style={"background": "#fff", "border": "1px solid #d9dee7", "borderRadius": "6px",
                               "padding": "10px 12px", "minWidth": "150px"}, children=[
            html.Div(kpi_labels[i], style={"fontSize": "12px", "color": "#6b7585"}),
            html.Div("—", id=f"kpi-{i}", style={"fontSize": "18px", "fontWeight": 600}),
        ])

    app.layout = html.Div(style={"fontFamily": "system-ui, sans-serif", "background": "#f6f7f9",
                                 "padding": "12px 20px", "color": "#1c2330"}, children=[
        html.H2("T800 planar base under NMPC", style={"margin": "0 0 4px"}),
        html.Div("Simulation only. The base is driven by a weld to a mocap body; the legs hold a PD "
                 "standing pose. No balance or walking. Rates are measured on this machine, not on "
                 "target hardware.", style={"fontSize": "12px", "color": "#6b7585", "marginBottom": "10px"}),
        html.Div(style={"display": "flex", "gap": "12px", "alignItems": "center", "flexWrap": "wrap"}, children=[
            html.Label(["Seed ", dcc.Input(id="seed", type="number", value=0, min=0, max=10000, style={"width": "90px"})]),
            html.Label(["Max time (s) ", dcc.Input(id="max-time", type="number", value=30, min=1, max=120, style={"width": "90px"})]),
            html.Button("Start episode", id="start", n_clicks=0),
            html.Button("Stop", id="stop", n_clicks=0),
            html.Span("idle", id="state", style={"fontSize": "13px", "color": "#6b7585"}),
        ]),
        html.Div(style={"display": "grid", "gridTemplateColumns": "repeat(auto-fit, minmax(170px, 1fr))",
                        "gap": "10px", "margin": "12px 0"}, children=[card(i) for i in kpi_ids]),
        dcc.Graph(id="scene", config={"displayModeBar": False}),
        html.Div(style={"display": "grid", "gridTemplateColumns": "repeat(auto-fit, minmax(420px, 1fr))",
                        "gap": "10px"}, children=[
            dcc.Graph(id="c-solve", config={"displayModeBar": False}),
            dcc.Graph(id="c-tilt", config={"displayModeBar": False}),
            dcc.Graph(id="c-person", config={"displayModeBar": False}),
            dcc.Graph(id="c-speed", config={"displayModeBar": False}),
            dcc.Graph(id="c-accel", config={"displayModeBar": False}),
        ]),
        dcc.Interval(id="tick", interval=POLL_MS, n_intervals=0),
        html.Div("Stage 0 scenario S2: persons do not react to the robot. Solve times are sandbox "
                 "measurements, not an NMPC rate.", style={"fontSize": "12px", "color": "#6b7585", "marginTop": "10px"}),
    ])

    @app.callback(Output("state", "children", allow_duplicate=True),
                  Input("start", "n_clicks"), State("seed", "value"), State("max-time", "value"),
                  prevent_initial_call=True)
    def on_start(_, seed, max_time):
        try:
            seed = int(seed or 0)
            max_time = float(max_time or 30)
        except (TypeError, ValueError):
            return "invalid seed or max time"
        if not (0 <= seed <= 10000) or not (1.0 <= max_time <= 120.0):
            return "seed must be 0–10000, max time 1–120 s"
        runs.start(seed, max_time)
        return f"starting seed {seed}"

    @app.callback(Output("state", "children", allow_duplicate=True),
                  Input("stop", "n_clicks"), prevent_initial_call=True)
    def on_stop(_):
        runs.stop()
        return "stopping…"

    outputs = ([Output("scene", "figure")]
               + [Output(c, "figure") for c in ["c-solve", "c-tilt", "c-person", "c-speed", "c-accel"]]
               + [Output(f"kpi-{i}", "children") for i in kpi_ids]
               + [Output("state", "children")])

    @app.callback(outputs, Input("tick", "n_intervals"))
    def refresh(_):
        events, running, error, _rid = runs.snapshot()
        steps = [e for e in events if e["type"] == "step"]
        if not events:
            return [no_update] * len(outputs)
        k = kpi_values(events, running, error)
        charts = [
            line_chart(steps, "NMPC solve time (ms)", [("solve", "#6a4fb3", lambda s: s["solve_ms"])], yrange=(0, None)),
            line_chart(steps, "Pelvis tilt (deg)", [("tilt", "#1f6feb", lambda s: s["tilt_deg"])],
                       yrange=(0, 20), hline=20, hline_label="fall limit 20°"),
            line_chart(steps, "Distance to nearest agent (m)",
                       [("person", "#d1433b", lambda s: s["dist_person"]), ("object", "#2a9d8f", lambda s: s["dist_object"])],
                       yrange=(0, None), hline=events[0]["d_min_person"], hline_label="d_min person (ASSUMPTION)"),
            line_chart(steps, "Speed (m/s) and tracking error (mm)",
                       [("speed", "#2e7d32", lambda s: s["speed"]), ("track err", "#e0a100", lambda s: s["track_err_m"] * 1000)],
                       yrange=(0, None)),
            line_chart(steps, "Commanded acceleration (m/s²)",
                       [("a_x", "#1f6feb", lambda s: s["accel"][0]), ("a_y", "#d1433b", lambda s: s["accel"][1])],
                       yrange=(-1.5, 1.5)),
        ]
        return [scene_figure(events)] + charts + [k[i] for i in kpi_ids] + [k["state"]]

    return app


app = build_app()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8050)
    a = ap.parse_args()
    app.run(host=a.host, port=a.port, debug=False)
