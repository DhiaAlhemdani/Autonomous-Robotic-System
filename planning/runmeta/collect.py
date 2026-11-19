"""Collect the run-metadata record for one simulation run."""
import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys
from importlib import metadata

SCHEMA_VERSION = "1"
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _git(*args):
    return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True).stdout.strip()


def _package_version(name):
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return "not-installed"


def config_hash(config):
    canonical = json.dumps(config, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(canonical.encode()).hexdigest()


def collect_meta(scenario_id, seed, config, argv=None):
    commit = _git("rev-parse", "HEAD") or "unknown"
    dirty = bool(_git("status", "--porcelain", "--untracked-files=no", "--ignore-submodules"))
    return {
        "schema_version": SCHEMA_VERSION,
        "scenario_id": scenario_id,
        "seed": int(seed),
        "git_commit": commit,
        "git_dirty": dirty,
        # Set by the container entrypoint (docs/03). Outside a container it is recorded as such.
        "container_digest": os.environ.get("ARS_IMAGE_DIGEST", "not-in-container"),
        "config": config,
        "config_hash": config_hash(config),
        "hardware": {
            "os": platform.platform(),
            "machine": platform.machine(),
            "cpu_count": os.cpu_count(),
            "python": sys.version.split()[0],
        },
        "packages": {n: _package_version(n) for n in ("numpy", "scipy", "mujoco", "quadprog")},
        "command": " ".join(argv if argv is not None else sys.argv),
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
    }
