# biped_ws

ROS2 Humble workspace for a custom bipedal robot: RL-trained walking policy,
deployed through ros2_control on real hardware, MuJoCo, Gazebo, and Isaac Sim.

## About

The robot walks using a policy trained in **Isaac Lab**, then deployed as a
`libtorch`/ONNX model inside a custom **ros2_control** controller
(`biped_control`, `BlindWalkController`). A **YASMIN** finite state machine
(`biped_fsm`) drives the robot through `INIT → PASSIVE → STANDBY → ACTIVE`,
and the controller only runs policy inference once the FSM reaches `ACTIVE`.
The realtime path between the FSM, the IMU, and the controller runs over
**iceoryx2** shared memory instead of ROS topics, to keep it deterministic.

The stack supports four backends behind the same controller code — real
hardware (Sanpo CAN motor board over serial), **MuJoCo**, **Gazebo**, and
**Isaac Sim** — selected via the `sim_mode` launch argument, with a separate
URDF per backend sharing one xacro. There's also a standalone `mujoco_wheeled`
package for testing a wheeled-leg robot variant outside the full ROS2 stack.

Two independent test paths exist for a freshly exported policy: `test_mujoco`
(no ros2_control/FSM/iceoryx2, fastest iteration after export) and the full
stack (`biped_bringup` + `biped_control` + `biped_fsm`). See
[Where to look next](#where-to-look-next) for the file that documents this in
detail.

---

## Status at a glance

| Area | Status |
|---|---|
| RL policy training (Isaac Lab) | 🟡 Policy tuning in progress |
| Standalone MuJoCo policy testing (`test_mujoco`) | 🟡 In progress |
| Full ros2_control + FSM + iceoryx2 stack in MuJoCo | ✅ Done |
| Gazebo / Isaac Sim backends | 🟡 Wired up, less exercised than MuJoCo |
| Real robot deployment | ⏭️ Not started |
| `mujoco_wheeled` (wheeled-leg variant) | 🟡 Early / standalone |

---

## Repository layout

```
biped_ws/
├── src/
│   ├── biped_description/       # URDF/xacro (per-backend ros2_control blocks) + MuJoCo XML
│   ├── biped_control/           # ros2_control plugin: policy inference (libtorch), joint mapping
│   ├── biped_fsm/                # YASMIN state machine (Python + C++ mirror), joystick-driven
│   ├── biped_hardware_interface/ # Sanpo CAN motor board hardware interface (serial)
│   ├── biped_bringup/            # top-level launch files (mujoco/gazebo/isaacsim/real/mock)
│   ├── biped_teleop/             # joystick teleop
│   ├── biped_isaacsim/           # Isaac Sim side of the robot
│   ├── biped_msgs/               # custom messages
│   ├── iox2_msgs/                # iceoryx2 shared-memory message structs (C++ + Python)
│   ├── dm_imu/                   # IMU driver (migrated to ROS2)
│   │
│   ├── test_mujoco/              # standalone MuJoCo policy test, no ROS2 control stack
│   ├── mujoco_wheeled/           # standalone MuJoCo sim, wheeled-leg robot variant
│   ├── test_pkg/                 # dev scratch package (COLCON_IGNORE'd)
│   │
│   └── thrid_party_libraries/    # vendored (submodule: serial-ros2) — typo "thrid_" is real, don't fix
│
├── system_settings/
│   └── device_settings/          # udev rules (ESP32, Sanpo board, sensors)
│
└── src/zzz/                      # scratch — ideas, hardware research, old challenge notes (gitignored)
```

---

## Build & run

```bash
colcon build --symlink-install                          # whole workspace
colcon build --symlink-install --packages-select <pkg>   # one package
source install/setup.bash                                # every new shell, after every build
```

Entry points are in `biped_bringup/launch/`, one per backend:

| Backend | Launch file |
|---|---|
| MuJoCo | `biped_bringup_mujoco_with_control.launch.py` |
| Isaac Sim | `biped_bringup_isaacsim_with_control.launch.py` |
| Gazebo | `biped_bringup_gazebo_with_control.launch.py` |
| Mock hardware | `biped_bringup_mock_with_custom_controller.launch.py` |
| Real robot | `biped_bringup_real_robot_with_{position,trajectory}_control.launch.py` |
| Joystick teleop | `biped_teleop/launch/biped_teleop_launch.py` |
| FSM + teleop (sim exercise) | `biped_fsm/launch/test_fsm_joy.launch.py` |


---

## Hardware

| Item | Detail |
|---|---|
| Motor board | Sanpo CAN board over serial (`serial-ros2` submodule) |
| IMU | Serial-connected, driven by `dm_imu` |
| Compute | Jetson Orin NX (deployment target) |
| Input | Joystick (drives FSM transitions and command velocity) |

---

## Known issues / gotchas

- `test_mujoco` and `biped_control` each keep their own
  `policy_trained_config.hpp` and can silently diverge (joint order,
  damping gain). A mismatch with the YAML controller config's `joints:`
  order produces wrong commands with **no error thrown**.
- `models/exported/policy.pt` / `policy.onnx` are gitignored — deployed
  out-of-band, not committed.
- External deps (`libtorch`, `MuJoCo`, `onnxruntime`, `iceoryx2-cxx`) are
  linked via hard-coded paths in CMake, not pulled in by rosdep.

See `CLAUDE.md` (local, gitignored) for the full architecture writeup and
debugging tips.

---

## Where to look next

| Question | File |
|---|---|
| Full architecture — control path, FSM, iceoryx2, hardware interface | `CLAUDE.md` (local, gitignored) |
| iceoryx2 install recipe | `src/TODO.md` (local, gitignored) |
| Package-specific detail | `<package>/README.md` |
