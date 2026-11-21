"""Load the T800 MJCF in MuJoCo and print its structure. No controller, no simulation loop.

Usage: python check_t800_model.py [path-to-T800-root] (default: vendor/t800)
"""
import sys
from pathlib import Path

import mujoco


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "vendor/t800")
    xml = root / "assets/resource/robot/t800/xml/serial_t800.xml"
    if not xml.is_file():
        print(f"FAIL: model not found: {xml}", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    print(f"mujoco {mujoco.__version__}")
    print(f"bodies={model.nbody} joints={model.njnt} dof={model.nv} "
          f"actuators={model.nu} sensors={model.nsensor}")
    print(f"total body mass from model file = {model.body_mass.sum():.2f} kg (a model value, not measured)")
    print("joints: " + ", ".join(str(j) for j in joints))
    print("OK: model loads and a forward step runs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())