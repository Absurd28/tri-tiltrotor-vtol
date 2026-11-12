#!/usr/bin/env python3
"""Launch MuJoCo viewer for a given model XML.

Usage:
    python scripts/viewer.py                        # opens smoke_test.xml
    python scripts/viewer.py model/my_model.xml     # opens specified model
"""
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))


def main():
    xml = ROOT / "model" / "smoke_test.xml"
    if len(sys.argv) > 1:
        xml = Path(sys.argv[1])
        if not xml.is_absolute():
            xml = ROOT / xml

    if not xml.exists():
        print(f"Error: model not found: {xml}")
        sys.exit(1)

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("mujoco not installed. Activate the drone311 conda env first.")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    print(f"Launching viewer: {xml.name}")
    print(f"  bodies={model.nbody}, joints={model.njnt}, actuators={model.nu}")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()