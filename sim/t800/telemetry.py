"""In-process telemetry buffer for the free-standing balance run (viewer.py -> dashboard).

viewer.py writes one sample per control step (about 100 Hz of simulated time, thinned to
TELEMETRY_EVERY steps). The dashboard runs in the same process and reads a snapshot.

Thread-safe. Keeps the most recent MAX_SAMPLES samples. Simulation only.
"""
import threading
from collections import deque

MAX_SAMPLES = 6000        # 60 s at 100 Hz
TELEMETRY_EVERY = 5       # keep one sample out of every 5 physics steps (dt 0.002 s -> 100 Hz)


class Telemetry:
    def __init__(self, maxlen=MAX_SAMPLES):
        self._lock = threading.Lock()
        self._buf = deque(maxlen=maxlen)
        self._n = 0            # total samples ever written (survives deque trimming)
        self.state = "not started"
        self.meta = {}

    def set_state(self, text, **meta):
        with self._lock:
            self.state = text
            self.meta.update(meta)

    def push(self, sample):
        with self._lock:
            self._buf.append(sample)
            self._n += 1

    def snapshot(self):
        with self._lock:
            return list(self._buf), self.state, dict(self.meta), self._n

    def clear(self):
        with self._lock:
            self._buf.clear()
            self._n = 0
            self.meta = {}


TELEMETRY = Telemetry()
