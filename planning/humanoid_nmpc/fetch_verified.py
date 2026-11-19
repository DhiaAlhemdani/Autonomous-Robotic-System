#!/usr/bin/env python3
"""Install the humanoid NMPC requirements from wheels verified against PyPI.

Why: on some networks, pip's own downloads fail the sha256 check because the
bytes change or are cut off in transit. This script downloads each wheel from
PyPI, checks it against the sha256 that PyPI publishes, retries on any mismatch
or network error, then installs only the verified local files (--no-index).
It does not weaken any check: a file is installed only if its hash matches PyPI.

Run from the repository root, with the repo venv:
    python3 -m venv .venv                       # once
    .venv/bin/python planning/humanoid_nmpc/fetch_verified.py
Wheels are stored in .venv/wheels (gitignored by .venv*/).
Supports CPython 3.x on Linux x86_64 (wheels only; no source builds).
"""
import argparse
import hashlib
import http.client
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
REQ = HERE / "requirements.txt"

# Transitive dependencies of the pinned packages. Versions are the ones pip
# resolved for these pins on the user's machine (Python 3.12, Linux x86_64).
TRANSITIVE = {
    "jinja2": "3.1.6",
    "markupsafe": "3.0.4",
    "joblib": "1.6.0",
    "cloudpickle": "3.1.3",
    "setuptools": "84.2.1",
    "iniconfig": "2.3.1",
    "packaging": "26.3",
    "pluggy": "1.7.0",
    "pygments": "2.21.0",
}


def read_pins():
    pins = {}
    for line in REQ.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        name, _, version = line.partition("==")
        if not version:
            raise SystemExit(f"requirements line must be name==version: {line!r}")
        pins[name.strip().lower()] = version.strip()
    return pins


def fetch_json(url, tries=6):
    for attempt in range(1, tries + 1):
        try:
            with urllib.request.urlopen(url, timeout=60) as r:
                return json.load(r)
        except (urllib.error.URLError, OSError, http.client.HTTPException) as e:
            print(f"  metadata retry {attempt}/{tries}: {e}", flush=True)
            time.sleep(2 * attempt)
    raise SystemExit(f"could not read {url}")


def python_tags_ok(filename):
    """True if the wheel suits this interpreter (CPython on Linux x86_64, or pure Python)."""
    if filename.endswith("-none-any.whl"):
        return True
    if not filename.endswith(".whl"):
        return False
    parts = filename[:-4].split("-")
    if len(parts) < 5:
        return False
    py, abi, plat = parts[-3], parts[-2], parts[-1]
    if "x86_64" not in plat or ("manylinux" not in plat and plat != "linux_x86_64"):
        return False
    maj, minor = sys.version_info[:2]
    if py == f"cp{maj}{minor}" and abi in (f"cp{maj}{minor}", "abi3"):
        return True
    if abi == "abi3" and py.startswith(f"cp{maj}") and py[len(f"cp{maj}"):].isdigit():
        return int(py[len(f"cp{maj}"):]) <= minor
    return False


def pick_file(release):
    for f in release["urls"]:
        if f["packagetype"] == "bdist_wheel" and python_tags_ok(f["filename"]):
            return f
    return None


def download(entry, dest_dir, tries=8):
    dest = dest_dir / entry["filename"]
    want = entry["digests"]["sha256"]
    if dest.exists() and hashlib.sha256(dest.read_bytes()).hexdigest() == want:
        print(f"  ok (cached): {entry['filename']}", flush=True)
        return dest
    tmp = dest.with_name(dest.name + ".part")
    for attempt in range(1, tries + 1):
        try:
            h = hashlib.sha256()
            with urllib.request.urlopen(entry["url"], timeout=120) as r, open(tmp, "wb") as f:
                expected_len = r.headers.get("Content-Length")
                n = 0
                while True:
                    chunk = r.read(1 << 16)
                    if not chunk:
                        break
                    f.write(chunk)
                    h.update(chunk)
                    n += len(chunk)
            if expected_len is not None and int(expected_len) != n:
                raise OSError(f"short read: {n} of {expected_len} bytes")
            if h.hexdigest() == want:
                tmp.replace(dest)
                print(f"  verified: {entry['filename']}", flush=True)
                return dest
            print(f"  hash mismatch (attempt {attempt}/{tries}): {entry['filename']}", flush=True)
        except (urllib.error.URLError, OSError, http.client.HTTPException) as e:
            print(f"  download retry {attempt}/{tries}: {entry['filename']}: {e}", flush=True)
        if tmp.exists():
            tmp.unlink()
        time.sleep(2 * attempt)
    raise SystemExit(f"could not get a verified copy of {entry['filename']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=str(REPO / ".venv" / "wheels"), help="where to keep wheels")
    ap.add_argument("--no-install", action="store_true", help="download and verify only")
    args = ap.parse_args()

    dest_dir = Path(args.dir)
    dest_dir.mkdir(parents=True, exist_ok=True)

    pins = read_pins()
    packages = dict(pins)
    for name, version in TRANSITIVE.items():
        packages.setdefault(name, version)

    paths = []
    for name, version in sorted(packages.items()):
        release = fetch_json(f"https://pypi.org/pypi/{name}/{version}/json")
        entry = pick_file(release)
        if entry is None:
            raise SystemExit(f"no wheel for {name}=={version} matching Python {sys.version_info[:2]} Linux x86_64")
        print(f"{name}=={version}", flush=True)
        paths.append(download(entry, dest_dir))

    if args.no_install:
        print("all wheels verified; not installing (--no-install)")
        return
    print("installing verified wheels from local files (--no-index)", flush=True)
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--no-index", "--no-cache-dir",
                           *[str(p) for p in paths]])
    print("done. run tests from planning/: ../.venv/bin/python -m pytest tests -q")


if __name__ == "__main__":
    main()
