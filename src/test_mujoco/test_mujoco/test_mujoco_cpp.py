import mujoco
import os
import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

xml = os.path.expanduser("/workspaces/biped_ws/src/test_mujoco/mujoco_xml/SF_biped.xml")


policy_path = os.path.expanduser(
    "/workspaces/biped_ws/src/test_mujoco/models/exported/policy.pt"
)

model = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(model)
policy = torch.jit.load(policy_path)
policy.eval()

# --- config ---
sim_dt = 1.0 / 500.0
decimation = 4
obs_history_len = 5
lin_vel_scale = 1.0
ang_vel_scale = 1.0
dof_pos_scale = 1.0
dof_vel_scale = 1.0

initial_joint_pose = [
    ("left_hip_pitch_joint", 0.3),
    ("right_hip_pitch_joint", -0.3),
    ("left_hip_roll_joint", 0.0),
    ("right_hip_roll_joint", 0.0),
    ("left_knee_joint", 0.6),
    ("right_knee_joint", -0.6),
]
initial_height = 0.53
gait_freq = 1.75
gait_phase_cfg = 0.5
gait_duration = 0.5
cmd_vel = np.array([0.0, 0.0, 0.0])
initial_jp = np.array([a for _, a in initial_joint_pose])

# --- init sim ---
mujoco.mj_resetData(model, data)
for name, angle in initial_joint_pose:
    data.qpos[model.joint(name).qposadr] = angle
    data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = angle
data.qpos[2] = initial_height
mujoco.mj_forward(model, data)


# --- obs helpers ---
def raw_obs():
    q_raw = data.sensor("orientation").data[[1, 2, 3, 0]].astype(np.double)
    r = R.from_quat(q_raw)
    pg = r.apply([0, 0, -1], inverse=True)
    lv = r.apply(data.qvel[:3], inverse=True)
    av = data.sensor("angular_velocity").data.copy()
    jp = np.array(
        [data.qpos[int(model.joint(n).qposadr)] for n, _ in initial_joint_pose]
    )
    jv = np.array(
        [data.qvel[int(model.joint(n).dofadr)] for n, _ in initial_joint_pose]
    )
    return pg, lv, av, jp, jv


# def build_terms(last_actions, gait_time):
#     pg, lv, av, jp, jv = raw_obs()
#     phase = (gait_time * gait_freq) % 1.0
#     terms = [
#         av * ang_vel_scale,
#         pg,
#         cmd_vel * np.array([lin_vel_scale, lin_vel_scale, ang_vel_scale]),
#         (jp - initial_jp) * dof_pos_scale,
#         jv * dof_vel_scale,
#         last_actions.copy(),
#         np.array([np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]),
#         np.array([gait_freq, gait_phase_cfg, gait_duration]),
#     ]

#     # DEBUG: print each term's size
#     names = ["ang_vel", "proj_g", "cmd", "jpos", "jvel", "actions", "phase", "gait"]
#     for n, t in zip(names, terms):
#         print(f"  {n}: shape={np.asarray(t).shape}  size={np.asarray(t).size}")

#     return terms


def build_terms(last_actions, gait_time):
    pg, lv, av, jp, jv = raw_obs()
    phase = (gait_time * gait_freq) % 1.0
    return [
        av * ang_vel_scale,
        pg,
        cmd_vel * np.array([lin_vel_scale, lin_vel_scale, ang_vel_scale]),
        (jp - initial_jp) * dof_pos_scale,
        jv * dof_vel_scale,
        last_actions.copy(),
        np.array([np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]),
        np.array([gait_freq, gait_phase_cfg, gait_duration]),
    ]


from collections import deque

num_obs_terms = 8
buffers = [deque(maxlen=obs_history_len) for _ in range(num_obs_terms)]


def push(terms):
    for i, t in enumerate(terms):
        buffers[i].append(t.astype(np.float64))


def stacked():
    parts = []
    for buf in buffers:
        for snap in buf:
            parts.append(snap)
    return np.concatenate(parts).astype(np.float32)


last_actions = np.zeros(6)
gait_time = 0.0

# prime
t0 = build_terms(last_actions, gait_time)
for _ in range(obs_history_len):
    push(t0)


def run_policy(label):
    v = stacked()
    print(f"stacked_size = {v.size}")
    obs_tensor = torch.from_numpy(v).unsqueeze(0).float()
    with torch.no_grad():
        act = policy(obs_tensor).detach().cpu().numpy().flatten()
    print(f"{label} actions = " + " ".join(f"{a:.10f}" for a in act))


run_policy("step=0 ")

for cycle in range(1, 11):
    for _ in range(decimation):
        mujoco.mj_step(model, data)
    gait_time += sim_dt * decimation
    push(build_terms(last_actions, gait_time))
    if cycle in (1, 5, 10):
        run_policy(f"step={cycle * decimation}")
