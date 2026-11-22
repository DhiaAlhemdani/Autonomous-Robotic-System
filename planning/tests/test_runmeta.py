import json
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from runmeta.collect import collect_meta, config_hash  # noqa: E402
from runmeta.validate import check_file, problems  # noqa: E402


def test_collected_record_is_complete_and_clean_or_flagged():
    meta = collect_meta("unit_test", 3, {"dt": 0.1}, argv=["x"])
    errs = problems(meta)
    # Outside a git checkout the record is flagged; inside one with a clean tree it is complete.
    assert all(e.startswith(("working tree was dirty", "git_commit unknown")) for e in errs), errs
    assert meta["seed"] == 3 and meta["scenario_id"] == "unit_test"


@pytest.mark.parametrize("key", ["seed", "git_commit", "config_hash", "container_digest", "packages"])
def test_missing_field_is_rejected(key):
    meta = collect_meta("unit_test", 0, {"a": 1}, argv=["x"])
    meta["git_commit"] = "a" * 40
    meta["git_dirty"] = False
    del meta[key]
    assert any(f"missing {key}" in e for e in problems(meta))


def test_complete_clean_record_passes():
    meta = collect_meta("unit_test", 0, {"a": 1}, argv=["x"])
    meta.update(git_commit="a" * 40, git_dirty=False)
    assert problems(meta) == []


def test_config_hash_tracks_config():
    assert config_hash({"a": 1, "b": 2}) == config_hash({"b": 2, "a": 1})
    assert config_hash({"a": 1}) != config_hash({"a": 2})


def test_validator_rejects_run_without_meta(tmp_path):
    p = tmp_path / "run.json"
    p.write_text(json.dumps({"goal_reached": True}))
    assert check_file(str(p)) == ["no meta block"]
    r = subprocess.run([sys.executable, "-m", "runmeta.validate", str(p)],
                       cwd=os.path.join(os.path.dirname(__file__), ".."), capture_output=True)
    assert r.returncode == 1
