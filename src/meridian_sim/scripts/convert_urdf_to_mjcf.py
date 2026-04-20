#!/usr/bin/env python3
"""Convert ur5e.urdf to MJCF format using MuJoCo's native URDF loader."""
import os
import sys
import mujoco
import numpy as np

ASSETS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "meridian_description", "assets")
)

URDF_PATH = os.path.join(ASSETS_DIR, "ur5e.urdf")
MJCF_PATH = os.path.join(ASSETS_DIR, "ur5e.xml")


def main():
    if not os.path.isfile(URDF_PATH):
        sys.exit(f"URDF not found: {URDF_PATH}")

    print(f"Loading URDF: {URDF_PATH}")
    with open(URDF_PATH, "r") as f:
        urdf_xml = f.read()

    # Resolve package:// URIs to absolute filesystem paths
    import re
    def resolve_package(m):
        pkg = m.group(1)
        rest = m.group(2)
        return f"/opt/ros/jazzy/share/{pkg}/{rest}"
    urdf_xml = re.sub(r'package://([^/]+)/([^"]+)', resolve_package, urdf_xml)

    model = mujoco.MjModel.from_xml_string(urdf_xml)

    print(f"\nModel loaded: {model.njnt} joints, {model.nbody} bodies")
    print("\nJoint names and limits:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        limited = model.jnt_limited[i]
        lo = model.jnt_range[i, 0] if limited else float("-inf")
        hi = model.jnt_range[i, 1] if limited else float("inf")
        print(f"  [{i}] {name:30s}  range=[{lo:.4f}, {hi:.4f}]")

    os.makedirs(ASSETS_DIR, exist_ok=True)
    mujoco.mj_saveLastXML(MJCF_PATH, model)
    print(f"\nMJCF saved to: {MJCF_PATH}")


if __name__ == "__main__":
    main()
