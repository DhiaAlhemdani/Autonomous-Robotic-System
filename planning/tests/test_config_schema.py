import copy
import os
import sys
from dataclasses import asdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config_schema import validate  # noqa: E402
from stage0.config import Stage0Config  # noqa: E402

GOOD = {
    "controller": asdict(Stage0Config()),
    "sim": {"kp": 600.0, "kd": 20.0, "weld_solref": [0.02, 1.0], "settle_s": 2.0,
            "max_time_s": 30.0, "solver": "slsqp", "analytic_gradients": True},
}


def test_defaults_conform():
    assert validate(GOOD) == []


def test_unknown_key_rejected():
    c = copy.deepcopy(GOOD)
    c["controller"]["bogus"] = 1
    assert any("bogus: unknown key" in e for e in validate(c))


def test_missing_key_rejected():
    c = copy.deepcopy(GOOD)
    del c["sim"]["kp"]
    assert any("sim.kp: missing" in e for e in validate(c))


def test_range_checked():
    c = copy.deepcopy(GOOD)
    c["controller"]["dt"] = 0.0          # exclusive lower bound
    c["controller"]["risk"] = 0.9        # above upper bound
    errs = validate(c)
    assert any("controller.dt" in e for e in errs)
    assert any("controller.risk" in e for e in errs)


def test_type_checked_and_bool_is_not_a_number():
    c = copy.deepcopy(GOOD)
    c["controller"]["horizon"] = 20.5
    c["sim"]["analytic_gradients"] = 1
    errs = validate(c)
    assert any("controller.horizon" in e for e in errs)
    assert any("sim.analytic_gradients" in e for e in errs)


def test_schema_matches_code_fields():
    from config_schema import CONTROLLER
    assert set(CONTROLLER) == set(asdict(Stage0Config()))
