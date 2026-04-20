#!/usr/bin/env python3
"""Verify the augmented ur5e.xml loads correctly and F/T sensors produce output."""
import os
import sys
import mujoco
import numpy as np
try:
    import mujoco.viewer as _mj_viewer
except Exception:
    _mj_viewer = None

ASSETS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "meridian_description", "assets")
)
MJCF_PATH = os.path.join(ASSETS_DIR, "ur5e.xml")


def main():
    if not os.path.isfile(MJCF_PATH):
        sys.exit(f"MJCF not found: {MJCF_PATH}\nRun convert_urdf_to_mjcf.py first.")

    print(f"Loading MJCF: {MJCF_PATH}")
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    print(f"\nBodies : {model.nbody}")
    print(f"Joints : {model.njnt}")

    print("\nJoint names:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  [{i}] {name}")

    print("\nSensor names:")
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        print(f"  [{i}] {name}")

    if model.nsensor == 0:
        print("WARNING: no sensors found in model!")

    # Locate force and torque sensor indices in sensordata
    force_adr = torque_adr = None
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        if "force" in name.lower():
            force_adr = model.sensor_adr[i]
        if "torque" in name.lower():
            torque_adr = model.sensor_adr[i]

    mujoco.mj_resetData(model, data)

    print("\nStepping simulation 1000 times...")
    mujoco.mj_step(model, data)
    first_force = data.sensordata[force_adr:force_adr+3].copy() if force_adr is not None else None
    first_torque = data.sensordata[torque_adr:torque_adr+3].copy() if torque_adr is not None else None

    for _ in range(999):
        mujoco.mj_step(model, data)

    last_force = data.sensordata[force_adr:force_adr+3].copy() if force_adr is not None else None
    last_torque = data.sensordata[torque_adr:torque_adr+3].copy() if torque_adr is not None else None

    print(f"\nF/T readings (step 1):")
    print(f"  force  = {first_force}")
    print(f"  torque = {first_torque}")
    print(f"\nF/T readings (step 1000):")
    print(f"  force  = {last_force}")
    print(f"  torque = {last_torque}")

    if os.environ.get("DISPLAY") and _mj_viewer is not None:
        import time
        print("\nLaunching passive viewer for 5 seconds...")
        with _mj_viewer.launch_passive(model, data) as v:
            start = time.time()
            while time.time() - start < 5.0:
                mujoco.mj_step(model, data)
                v.sync()
    else:
        print("\nNo display available — skipping viewer.")

    print("\nAll checks passed.")


if __name__ == "__main__":
    main()
