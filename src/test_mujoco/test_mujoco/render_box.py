#!/usr/bin/env python3
"""MuJoCo Python test: render a box using the passive viewer."""

import sys
import time

import mujoco
import mujoco.viewer

from ament_index_python.packages import get_package_share_directory


def main():
    # Locate the model file
    try:
        pkg_share = get_package_share_directory("test_mujoco")
        model_path = f"{pkg_share}/box.xml"
    except Exception:
        # Fallback: assume running from source
        model_path = "src/box.xml"

    # Load model
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
    except Exception as e:
        print(f"Error loading model from '{model_path}': {e}")
        sys.exit(1)

    data = mujoco.MjData(model)

    print("Launching MuJoCo viewer...")
    print("Controls: left-drag = rotate, right-drag = pan, scroll = zoom")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Reset the free camera to look at the box
        viewer.cam.trackbodyid = 1
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 45.0
        viewer.cam.elevation = -20.0

        while viewer.is_running():
            step_start = time.time()

            # Step simulation
            mujoco.mj_step(model, data)

            # Sync viewer
            viewer.sync()

            # Real-time sync (~60 Hz)
            elapsed = time.time() - step_start
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)


if __name__ == "__main__":
    main()
