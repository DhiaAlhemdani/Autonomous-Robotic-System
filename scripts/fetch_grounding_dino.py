#!/usr/bin/env python3
"""Fetch the Grounding DINO base model from Hugging Face at a pinned revision and verify it.

Usage (repo root):
    pip install huggingface_hub
    python scripts/fetch_grounding_dino.py            # downloads into models/grounding-dino-base/
    python scripts/fetch_grounding_dino.py --verify   # only checks files already present

Upstream: IDEA-Research/grounding-dino-base (Apache-2.0). Revision pinned to a commit, not `main`.
Small files are checked against their git blob SHA-1 from the pinned tree. The weights are checked
against the SHA-256 and size of the Git LFS object. Any mismatch is an error (exit status 1).
"""
import argparse
import hashlib
import sys
from pathlib import Path

REPO_ID = "IDEA-Research/grounding-dino-base"
REVISION = "12bdfa3120f3e7ec7b434d90674b3396eccf88eb"  # upstream commit (pinned)
DEFAULT_DEST = Path(__file__).resolve().parent.parent / "models" / "grounding-dino-base"

# filename -> expected git blob SHA-1 (from the pinned tree)
GIT_BLOBS = {
    "README.md": "14c920e1b2effe7cc8f522a2be907758f828c9f1",
    "config.json": "5a7f6206a1e488c54316e1f594311dd47a03a41b",
    "preprocessor_config.json": "5cb45d963917ed130ce46a93204b349cbec21131",
    "special_tokens_map.json": "a8b3208c2884c4efb86e49300fdd3dc877220cdf",
    "tokenizer.json": "688882a79f44442ddc1f60d70334a7ff5df0fb47",
    "tokenizer_config.json": "ed97a84add5f9b2091e756765ad3ba087a345e17",
    "vocab.txt": "fb140275c155a9c7c5a3b3e0e77a9e839594a938",
}
# filename -> (expected SHA-256, expected size in bytes), from the Git LFS object
WEIGHTS = {
    "model.safetensors": (
        "5548f844c928c4b6f411fa8cbcc2bfa8dbbba437cb1d513975519f93c2a9ed21",
        933400872,
    ),
}


def git_blob_sha1(path):
    data = Path(path).read_bytes()
    h = hashlib.sha1(f"blob {len(data)}\0".encode())
    h.update(data)
    return h.hexdigest()


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def verify(dest):
    """Return a list of problems (empty means every expected file is present and matches)."""
    dest = Path(dest)
    problems = []
    for name, expected in GIT_BLOBS.items():
        p = dest / name
        if not p.is_file():
            problems.append(f"missing: {name}")
        elif git_blob_sha1(p) != expected:
            problems.append(f"blob SHA-1 mismatch: {name}")
    for name, (expected_sha, expected_size) in WEIGHTS.items():
        p = dest / name
        if not p.is_file():
            problems.append(f"missing: {name}")
        elif p.stat().st_size != expected_size:
            problems.append(f"size mismatch: {name} ({p.stat().st_size} bytes)")
        elif sha256_of(p) != expected_sha:
            problems.append(f"SHA-256 mismatch: {name}")
    return problems


def fetch(dest):
    try:
        from huggingface_hub import hf_hub_download
    except ImportError:
        sys.exit("huggingface_hub is not installed: pip install huggingface_hub")
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    for name in [*GIT_BLOBS, *WEIGHTS]:
        hf_hub_download(repo_id=REPO_ID, filename=name, revision=REVISION, local_dir=str(dest))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", default=str(DEFAULT_DEST))
    ap.add_argument("--verify", action="store_true", help="only verify files already present")
    args = ap.parse_args(argv)
    if not args.verify:
        fetch(args.dest)
    problems = verify(args.dest)
    if problems:
        print("FAILED:")
        for p in problems:
            print("  -", p)
        return 1
    print(f"OK: {REPO_ID} @ {REVISION[:12]} verified in {args.dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
