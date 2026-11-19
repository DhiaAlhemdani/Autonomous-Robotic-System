"""Configuration schema, version 1 (docs/02 §8: frozen before implementation).

Covers the Stage 0 controller and the simulation settings for the planar base. Every
field has a type, a unit, a range, and a flag saying whether its value is a placeholder.
Unknown keys are errors. Changing this schema means a new version and a change record
in docs/02 §10.
"""
from dataclasses import dataclass

SCHEMA_VERSION = "1"


@dataclass(frozen=True)
class Field:
    typ: type
    unit: str
    lo: float | None = None      # inclusive lower bound (None = unbounded)
    hi: float | None = None      # inclusive upper bound
    lo_open: bool = False        # True: bound is exclusive (> lo)
    assumption: bool = False     # True: placeholder, not a platform value


CONTROLLER = {
    "dt": Field(float, "s", lo=0.0, lo_open=True, assumption=True),
    "horizon": Field(int, "steps", lo=1, assumption=True),
    "v_max": Field(float, "m/s", lo=0.0, lo_open=True, assumption=True),
    "a_max": Field(float, "m/s^2", lo=0.0, lo_open=True, assumption=True),
    "d_min_person": Field(float, "m", lo=0.0, lo_open=True, assumption=True),
    "d_min_object": Field(float, "m", lo=0.0, lo_open=True, assumption=True),
    "risk": Field(float, "1", lo=0.0, hi=0.5, lo_open=True, assumption=True),
    "sigma_robot": Field(float, "m", lo=0.0, assumption=True),
    "agent_sigma0": Field(float, "m", lo=0.0, assumption=True),
    "agent_sigma_rate": Field(float, "m/s", lo=0.0, assumption=True),
    "w_goal": Field(float, "1", lo=0.0, assumption=True),
    "w_effort": Field(float, "1", lo=0.0, lo_open=True, assumption=True),
    "w_jerk": Field(float, "1", lo=0.0, assumption=True),
    "w_lat": Field(float, "1", lo=0.0, assumption=True),
}

SIM = {
    "kp": Field(float, "N m/rad", lo=0.0, lo_open=True, assumption=True),
    "kd": Field(float, "N m s/rad", lo=0.0, assumption=True),
    "weld_solref": Field(list, "s, 1"),
    "settle_s": Field(float, "s", lo=0.0),
    "max_time_s": Field(float, "s", lo=0.0, lo_open=True),
    "solver": Field(str, "1"),
    "analytic_gradients": Field(bool, "1"),
}

SECTIONS = {"controller": CONTROLLER, "sim": SIM}


def validate(config):
    """Return a list of error strings; empty means the config conforms to version 1."""
    errs = []
    if set(config) != set(SECTIONS):
        errs.append(f"top-level keys must be exactly {sorted(SECTIONS)}, got {sorted(config)}")
    for sec, spec in SECTIONS.items():
        block = config.get(sec)
        if not isinstance(block, dict):
            errs.append(f"{sec}: must be an object")
            continue
        for k in sorted(set(block) - set(spec)):
            errs.append(f"{sec}.{k}: unknown key")
        for k, f in spec.items():
            if k not in block:
                errs.append(f"{sec}.{k}: missing")
                continue
            v = block[k]
            if f.typ is float:
                ok = isinstance(v, (int, float)) and not isinstance(v, bool)
            elif f.typ is int:
                ok = isinstance(v, int) and not isinstance(v, bool)
            else:
                ok = isinstance(v, f.typ) and not (f.typ is bool and not isinstance(v, bool))
            if not ok:
                errs.append(f"{sec}.{k}: expected {f.typ.__name__}, got {type(v).__name__}")
                continue
            if f.typ in (float, int):
                if f.lo is not None and (v <= f.lo if f.lo_open else v < f.lo):
                    errs.append(f"{sec}.{k}: {v} below {'>' if f.lo_open else '>='} {f.lo} {f.unit}")
                if f.hi is not None and v > f.hi:
                    errs.append(f"{sec}.{k}: {v} above <= {f.hi} {f.unit}")
    return errs
