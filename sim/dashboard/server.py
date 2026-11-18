"""Real-time dashboard for the T800 planar-base NMPC simulation (sim/t800/planar_base.py).

One episode runs at a time in a background thread. Every control step is appended to an
in-memory event log and streamed to browsers over a WebSocket. A new browser connection gets
the whole log for the current run, so it can join mid-episode.

The rates shown are measured wall-clock rates in this sandbox. They are not real-time
guarantees and not target-hardware numbers.

Run (from the repo root):
    cd sim/dashboard && /tmp/simvenv/bin/python -m uvicorn server:app --host 0.0.0.0 --port 8000
"""
import asyncio
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "t800"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "planning"))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import FileResponse, JSONResponse  # noqa: E402
from fastapi.staticfiles import StaticFiles  # noqa: E402

import planar_base  # noqa: E402
from stage0.config import Stage0Config  # noqa: E402

app = FastAPI(title="T800 planar NMPC dashboard")


class RunManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.run_id = 0
        self.events = []        # list of dicts; first is "meta", last may be "done"
        self.thread = None
        self.stop_flag = False
        self.running = False
        self.last_error = None

    def start(self, seed: int, max_time: float):
        with self.lock:
            if self.running:
                self.stop_flag = True
            self.run_id += 1
            run_id = self.run_id
            cfg = Stage0Config()
            self.events = [{"type": "meta", "run_id": run_id, "seed": seed,
                            "d_min_person": cfg.d_min_person, "d_min_object": cfg.d_min_object,
                            "dt": cfg.dt, "max_time": max_time}]
            self.stop_flag = False
            self.running = True
            self.last_error = None
            args = argparse.Namespace(seed=seed, kp=600.0, kd=20.0,
                                      weld_solref=[0.02, 1.0], settle=2.0, max_time=max_time)
            self.thread = threading.Thread(target=self._worker, args=(run_id, args), daemon=True)
            self.thread.start()
        return run_id

    def _worker(self, run_id, args):
        t0 = time.perf_counter()

        def on_step(ev):
            with self.lock:
                if run_id == self.run_id:
                    ev = dict(ev, type="step", run_id=run_id,
                              wall_s=round(time.perf_counter() - t0, 3))
                    self.events.append(ev)

        try:
            res = planar_base.run_episode(args, on_step=on_step, stop=lambda: self.stop_flag)
            summary = dict(res)
            summary.pop("trace", None)
            with self.lock:
                if run_id == self.run_id:
                    self.events.append({"type": "done", "run_id": run_id, "result": summary})
        except Exception as exc:  # report to the browser instead of dying silently
            with self.lock:
                self.last_error = repr(exc)
                if run_id == self.run_id:
                    self.events.append({"type": "error", "run_id": run_id, "message": repr(exc)})
        finally:
            with self.lock:
                if run_id == self.run_id:
                    self.running = False

    def snapshot(self, from_index: int):
        with self.lock:
            return self.run_id, self.events[from_index:], len(self.events), self.running


runs = RunManager()


@app.get("/")
def index():
    return FileResponse(os.path.join(HERE, "static", "index.html"))


app.mount("/static", StaticFiles(directory=os.path.join(HERE, "static")), name="static")


@app.post("/api/run")
def start_run(seed: int = 0, max_time: float = 30.0):
    if not (0 <= seed <= 10_000) or not (1.0 <= max_time <= 120.0):
        return JSONResponse({"error": "seed must be 0..10000, max_time 1..120 s"}, status_code=400)
    run_id = runs.start(seed, max_time)
    return {"run_id": run_id, "seed": seed}


@app.get("/api/status")
def status():
    run_id, _, n, running = runs.snapshot(10 ** 9)
    return {"run_id": run_id, "events": n, "running": running, "error": runs.last_error}


@app.websocket("/ws")
async def ws(socket: WebSocket):
    await socket.accept()
    sent_run, idx = None, 0
    try:
        while True:
            run_id, batch, n, _ = runs.snapshot(idx if sent_run is not None else 0)
            if sent_run is not None and run_id != sent_run:
                # a new run started: restart from its beginning
                sent_run = None
                continue
            if sent_run is None:
                sent_run = run_id
                run_id, batch, n, _ = runs.snapshot(0)
            for ev in batch:
                await socket.send_json(ev)
            idx = n
            await asyncio.sleep(0.1)
    except (WebSocketDisconnect, RuntimeError):
        return
