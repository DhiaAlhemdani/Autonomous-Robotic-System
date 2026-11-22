"""Checks the fetch script's verification logic offline (no network)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import fetch_grounding_dino as fgd  # noqa: E402


def test_git_blob_sha1_matches_git(tmp_path):
    f = tmp_path / "x.txt"
    f.write_bytes(b"hello grounding dino\n")
    out = subprocess.run(["git", "hash-object", str(f)], capture_output=True, text=True, check=True).stdout.strip()
    assert fgd.git_blob_sha1(f) == out


def test_verify_reports_missing_files(tmp_path):
    problems = fgd.verify(tmp_path)
    assert any("missing: model.safetensors" in p for p in problems)
    assert any("missing: config.json" in p for p in problems)


def test_verify_flags_wrong_content(tmp_path):
    for name in fgd.GIT_BLOBS:
        (tmp_path / name).write_bytes(b"not the real file")
    (tmp_path / "model.safetensors").write_bytes(b"short")
    problems = fgd.verify(tmp_path)
    assert any("blob SHA-1 mismatch: config.json" in p for p in problems)
    assert any("size mismatch: model.safetensors" in p for p in problems)


def test_pinned_revision_is_a_full_commit_hash():
    assert len(fgd.REVISION) == 40 and all(c in "0123456789abcdef" for c in fgd.REVISION)
