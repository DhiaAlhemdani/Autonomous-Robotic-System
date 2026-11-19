"""Reject run records that are incomplete (docs/02 §7 rule).

Usage:  python -m runmeta.validate runs/**/*.json     (from planning/)
Exit status 1 if any file has no 'meta' block or a missing or malformed field.
"""
import json
import re
import sys

REQUIRED = {
    "schema_version": str,
    "scenario_id": str,
    "seed": int,
    "git_commit": str,
    "git_dirty": bool,
    "container_digest": str,
    "config": dict,
    "config_hash": str,
    "hardware": dict,
    "packages": dict,
    "command": str,
    "created_utc": str,
}


def problems(meta):
    out = []
    for key, typ in REQUIRED.items():
        if key not in meta:
            out.append(f"missing {key}")
        elif not isinstance(meta[key], typ) or (typ is int and isinstance(meta[key], bool)):
            out.append(f"{key} has type {type(meta[key]).__name__}")
    if not out:
        if not re.fullmatch(r"[0-9a-f]{40}", meta["git_commit"]) and meta["git_commit"] != "unknown":
            out.append("git_commit is not a commit hash")
        if meta["git_commit"] == "unknown":
            out.append("git_commit unknown (not a git checkout)")
        if not re.fullmatch(r"[0-9a-f]{64}", meta["config_hash"]):
            out.append("config_hash is not sha256")
        if meta["git_dirty"]:
            out.append("working tree was dirty; result not reproducible from the commit")
    return out


def check_file(path):
    with open(path) as f:
        data = json.load(f)
    meta = data.get("meta")
    if meta is None:
        return ["no meta block"]
    return problems(meta)


def main(paths):
    bad = 0
    for p in paths:
        errs = check_file(p)
        if errs:
            bad += 1
            print(f"REJECT {p}: " + "; ".join(errs))
        else:
            print(f"ok     {p}")
    print(f"{len(paths) - bad}/{len(paths)} runs reportable")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
