# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace layout

Standard ROS 2 (Humble) colcon workspace rooted at the repo. All ROS packages live under `src/`. Build artifacts go to `build/`, `install/`, `log/` (gitignored). The `src/thrid_party_libraries/serial-ros2` directory is a git submodule (note the typo `thrid_` in the path is real and used in CMake/import paths). `src/temp/` and `src/TODO.md` are gitignored scratch.

## Build / source / run

```bash
# from workspace root
colcon build --symlink-install                          # build all
colcon build --symlink-install --packages-select <pkg>  # build one
source install/setup.bash                               # source after every build (and in every new shell)

# test a single package
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

There is no top-level lint task; per-package `BUILD_TESTING` runs `ament_lint_auto` (cpplint and copyright are intentionally skipped in every CMakeLists, so adding either is a no-op).

## Running the stack

The user-facing entry points are launch files in `biped_bringup/launch/`. Each combines a sim/robot bringup with the matching control launch:

- `biped_bringup_mujoco_with_control.launch.py` — MuJoCo sim + controller_manager + iceoryx2 IMU bridge
- `biped_bringup_isaacsim_with_control.launch.py` — IsaacSim
- `biped_bringup_gazebo_with_control.launch.py` — Gazebo (its own controller_manager comes from URDF; the standalone one is conditionally suppressed via `sim_mode`)
- `biped_bringup_mock_with_custom_controller.launch.py` — mock hardware
- `biped_bringup_real_robot_with_{position,trajectory}_control.launch.py` — real robot
- `biped_teleop/launch/biped_teleop_launch.py` — joystick teleop (joy + joy_state + joy_teleop)
- `biped_fsm/launch/test_fsm_joy.launch.py` — runs `mujoco_state_manager_node` + teleop, used to exercise the FSM via the joystick

`use_sim_time` and `sim_mode` (`mujoco` / `isaacsim` / `gazebo` / `mock`) are the load-bearing launch arguments; defaults differ between launch files, so always check before assuming.

## Architecture (the parts that span files)

### Control path: ros2_control + RL policy

`biped_control` is a `controller_interface::ControllerInterface` plugin (`biped_control/BlindWalkController`, exported via `biped_control.xml`). It is loaded by `controller_manager` using `config/blind_walk_policy_controller.yaml` (or `biped_ros2_controllers*.yaml`). Inside the controller `update()`:

1. Pull IMU and FSM state from **iceoryx2 shared memory** (not ROS topics) via subscribers in `iox2_msgs/iox_imu_msgs.hpp` / `iox2_msgs/iox_fsm_states_msg.hpp`.
2. Read joint positions/velocities from ros2_control state interfaces.
3. Build the observation vector with `ObservationBuilder` (history length / scaling / gait phase encoding live in `policy_trained_config.hpp`).
4. Run inference via `TorchPolicy` (libtorch) loading `models/exported/policy.pt`. There is also a parallel ONNX path (`onnxruntime`) used in `test_mujoco/`.
5. Publish target joint commands as `biped_msgs/BipedJointCommand` (realtime publisher) and write to ros2_control command interfaces.

`policy_trained_config.hpp` is the single source of truth that ties the trained network to the robot: joint order, action scale, stiffness/damping, gait params, decimation, observation history. The yaml-declared `joints:` order in the controller config is independent — the controller computes a mapping between the two using `control_to_joint_states_mapping`. **If you change either order, both mappings break silently.**

### State machine: YASMIN (Python) and a C++ mirror

`biped_fsm` runs a YASMIN state machine. Two parallel implementations exist for different deployments:

- **Python**: `biped_fsm/biped_fsm/state_manager_node.py` loads `xml/fsm_states_transistions.xml` (states INIT → PASSIVE → STANDBY → ACTIVE, plus FALLEN/ERROR/STOP). Sensor watchdogs and joystick inputs feed a YASMIN blackboard; transitions are XML-driven. Pluggable Python state classes are imported from `biped_fsm.robot_fsm_states`.
- **C++**: `state_manager_node` and `mujoco_state_manager_node` (built from `src/state_manager.cpp` and `src/mujoco_state_manager.cpp`) — the mujoco variant is the active one for sim, see `mujoco_fsm_states_transistions.xml` and `include/biped_fsm/mujoco_fsm_classes.hpp`. Plugin-style states are exported through `plugins.xml` / `export_fsm_states_plugin.cpp` so YASMIN can load them via pluginlib.

The FSM publishes its current state and command velocity to `iceoryx2` (topics `iox/fsm_data`, `iox/imu_data`, etc.) so the realtime controller can read them without going through the ROS executor. The Python side also has a parallel `multiprocessing.shared_memory` path defined in `biped_fsm/shared_memory_structs.py` (matching C++ structs in `include/biped_fsm/`) — this is legacy/being phased out in favor of iceoryx2.

### Cross-process IPC: iceoryx2

iceoryx2 (zero-copy shm) is used for the realtime path between FSM, IMU publisher, and controller. Message types are plain C++ structs in `iox2_msgs/include/iox2_msgs/*.hpp` with a `IOX2_TYPE_NAME` constant — these are the contract; changing the struct layout requires rebuilding **every** package that uses it. Python bindings exist in `iox2_msgs/iox2_msgs/iox_imu_msgs.py`. The `mujoco_imu_iox2_publisher_node` (in `biped_bringup`) bridges sim IMU into iceoryx2.

### Hardware interface

`biped_hardware_interface` exports two pluginlib classes (single-threaded and multi-threaded variants of `BipedSanpoHardwareInterface`) that talk to the Sanpo CAN motor board over serial via the `serial-ros2` submodule. Selection happens in the URDF (`biped_description/urdf/biped_real_ros2_control.urdf`). `dm_imu` is a separate node that drives the IMU over serial.

### Robot description

Multiple URDFs in `biped_description/urdf/` — one per backend (`biped_gazebo_ros2_control.urdf`, `biped_isaacsim_ros2_control.urdf`, `biped_mujoco_ros2_control.urdf`, `biped_mock_ros2_control.urdf`, `biped_real_ros2_control.urdf`). They share `biped.urdf.xacro` but plug in different `<ros2_control>` blocks. MuJoCo also uses `mujoco_xml/SF_biped.xml` directly.

## External dependencies (not pulled in by rosdep)

These are linked via hard-coded paths in CMake — installing them elsewhere will break the build:

- **libtorch**: `set(CMAKE_PREFIX_PATH "/usr/local/lib/python3.10/dist-packages/torch/share/cmake")` in both `biped_control/CMakeLists.txt` and `test_mujoco/CMakeLists.txt`.
- **MuJoCo**: header at `/usr/local/include/mujoco/mujoco.h`, lib at `/usr/local/lib/libmujoco.*`.
- **onnxruntime**: lib at `/usr/local/lib/libonnxruntime.*`.
- **iceoryx2-cxx** (`iceoryx2-cxx::static-lib-cxx`) — installed system-wide via `make install` from the iceoryx2 repo (see `src/TODO.md` for the install recipe).
- **yasmin** (`ros-humble-yasmin*` apt packages: yasmin, yasmin_ros, yasmin_factory, yasmin_viewer).
- **mujoco_ros2_control_msgs** is required by `biped_fsm`.

## Sim-to-sim policy testing (IsaacLab → ROS2)

There are two independent paths for running the RL walking policy:

**Path A — Standalone MuJoCo** (`test_mujoco`): No ros2_control, no FSM, no iceoryx2. Good for rapid iteration directly after exporting from IsaacLab. Drop `policy.pt` (libtorch) or `policy.onnx` (ONNX) into `test_mujoco/models/exported/` and run:
```bash
ros2 run test_mujoco mujoco_node_torch   # TorchScript path
ros2 run test_mujoco mujoco_node_onnx    # ONNX path
```

**Path B — Full ROS2 stack** (`biped_control` + `biped_bringup` + `biped_fsm`): Drop `policy.pt` into `biped_control/models/exported/`. The controller only runs inference when the FSM is in the `ACTIVE` state. Bring up MuJoCo:
```bash
ros2 launch biped_bringup biped_bringup_mujoco_with_control.launch.py
ros2 launch biped_fsm test_fsm_joy.launch.py   # in a second terminal (joystick + FSM)
```
Then use the joystick to drive through INIT → PASSIVE → STANDBY → ACTIVE.

### Policy config files — critical divergence

**`test_mujoco` and `biped_control` each have their own `policy_trained_config.hpp` that must be kept in sync manually.** They currently diverge:

| Field | `biped_control` | `test_mujoco` |
|---|---|---|
| joint order | L/R interleaved (hip_pitch, hip_roll, knee) | L-first then R |
| `damping_gain` | `-2.5` | `4.0` |

The joint order in `policy_trained_config.hpp` must also match `joints:` in the YAML controller config (`biped_ros2_controllers_mujoco.yaml` or `blind_walk_policy_controller.yaml`). A mismatch produces wrong joint commands **silently** — no error is thrown.

### Observation vector layout

`ObservationBuilder::stacked_observation()` stacks the full history of each term before moving to the next term (oldest-to-newest within each term, not interleaved across terms):

```
[ang_vel×5, proj_gravity×5, cmd_vel×5, joint_pos_rel×5, joint_vel×5, last_actions×5, gait_phase×5, gait_command×5]
```
Where each step is: ang_vel(3), proj_grav(3), cmd_vel(3), joint_pos_rel(6), joint_vel(6), last_actions(6), gait_phase(2), gait_cmd(3) = 32 per step → 160 total. This layout must match the IsaacLab training observation config exactly.

Joint positions are expressed **relative to the initial pose** (`initial_joint_pose` values), not absolute.

### Dropping a new exported policy

1. Export TorchScript from IsaacLab: `policy.pt`
2. Copy to `src/biped_control/models/exported/policy.pt` (and/or `src/test_mujoco/models/exported/policy.pt`)
3. Rebuild only the affected package: `colcon build --symlink-install --packages-select biped_control`
4. If joint order, history length, or observation terms changed in training, update `policy_trained_config.hpp` in **both** packages and the YAML controller config.

## Debugging tips

- **iceoryx2 services**: `iox2 service list` (CLI installed via cargo) shows live publishers and subscribers — useful for confirming the IMU bridge and FSM are publishing to `iox/imu_data` / `iox/fsm_data`.
- **Controller activation log**: on `on_activate`, the controller prints the state/command interface ordering — check this when suspecting joint mapping bugs.
- **Policy step log**: the first policy step prints a full observation breakdown per term per history step (look for `=== OBSERVATION BREAKDOWN ===` in controller output).
- **FSM not advancing**: the controller skips inference unless `fsm_state == ACTIVE`. Check `iox2 service list` if FSM data isn't arriving.

## Conventions worth knowing

- `use_sim_time` is set inconsistently across launch files (sometimes default `true`, sometimes `false`). Treat it as a per-launch override and don't assume.
- `biped_bringup_mujoco_with_control.launch.py` has `sim_mode` default `"isaacsim"` — this appears to be a bug; override with `sim_mode:=mujoco` when launching.
- Every CMakeLists currently sets `ament_cmake_copyright_FOUND TRUE` and `ament_cmake_cpplint_FOUND TRUE` to skip those linters — leave that alone unless adding licensing.
- The path `src/thrid_party_libraries/` (typo) is referenced in `.gitmodules`; do not "fix" the spelling without updating the submodule path everywhere.
- `models/exported/policy.pt` and `policy.onnx` in `biped_control/models/` and `test_mujoco/models/` are the deployed RL policies; they are gitignored binaries delivered out-of-band. The CMakeLists installs the whole `models/` directory into the package share.
