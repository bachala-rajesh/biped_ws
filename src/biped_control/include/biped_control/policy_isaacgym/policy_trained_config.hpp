#pragma once

#include <string>
#include <vector>
#include <utility>
#include <array>

// Configuration for the IsaacGym `biped_walk_flat` walking policy.
//
// Values mirror legged_gym `Biped_Walk_Cfg` (+ `LeggedRobotCfg` base):
//   - sim.dt = 0.005, control.decimation = 4              -> 50 Hz policy rate
//   - control.action_scale = 0.25
//   - control.stiffness/damping = 8 / 0.5 (all joints)
//   - normalization.obs_scales = {ang_vel 0.25, dof_pos 1.0, dof_vel 0.05}
//   - command scale = {lin_vel 2.0, lin_vel 2.0, ang_vel 0.25}
//   - init_state default joint angles / base height
//
// Observation (189) layout, term-major, history-major within each term:
//   [ rpy x10 | ang_vel x10 | cmd(1) | (q-q0) x10 | qd x10 | last_action(1) ]
// where each "x10" block is [current, t-1, ..., t-9] at the policy rate and
// the 9 past frames are ZERO until they fill in (matches reset behaviour).

namespace locomotion_isaacgym {

struct PolicyTrainedConfig {
  // Simulation / control timing (LeggedRobotCfg.sim.dt, control.decimation)
  double sim_dt     = 1.0 / 200.0;   // 0.005 s
  int    decimation = 4;             // policy runs every 4 sim steps -> 50 Hz

  // Observation history: current frame + 9 past frames
  int obs_history_len = 10;
  int num_history_terms = 4;         // rpy, ang_vel, joint_pos_rel, joint_vel

  // Observation scaling (normalization.obs_scales). rpy is unscaled.
  double ang_vel_scale = 0.25;
  double dof_pos_scale = 1.0;
  double dof_vel_scale = 0.05;
  // Command scaling: vx, vy use lin_vel scale (2.0); yaw uses ang_vel scale (0.25)
  std::array<double, 3> cmd_scale = {0.3, 0.0, 0.0};

  // Observation / action clipping (normalization.clip_*)
  double clip_observations = 100.0;
  double clip_actions      = 100.0;

  // Joint order AND initial pose (init_state.default_joint_angles).
  std::vector<std::pair<std::string, double>> initial_joint_pose = {
    {"left_hip_pitch_joint",   0.3},
    {"left_hip_roll_joint",    0.0},
    {"left_knee_joint",        0.6},
    {"right_hip_pitch_joint", -0.3},
    {"right_hip_roll_joint",   0.0},
    {"right_knee_joint",      -0.6},
  };
  double initial_height = 0.53;   // init_state.pos[2]

  // Action postprocessing: position_target = action * action_scale + q0
  double action_scale = 0.25;

  // Command velocity [vx, vy, yaw_rate]
  std::array<double, 3> cmd_vel = {0.0, 0.0, 0.0};

  // PD gains (control.stiffness / control.damping). Applied downstream by the
  // hardware/sim layer; kept here for documentation and configuration.
  double stiffness_gain = 8.0;
  double damping_gain   = 0.5;

  // Termination thresholds (check_termination), for optional fall detection.
  double term_roll  = 0.8;
  double term_pitch = 1.0;

  // Control order of joints expected by the policy (== initial_joint_pose order).
  std::vector<std::string> control_order_joints = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_knee_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_knee_joint",
  };

  // mapping between the control order and the joint order
  std::vector<int> control_to_joint_states_mapping = [this]() {
    std::vector<int> mapping;
    for (size_t i = 0; i < control_order_joints.size(); ++i) {
      for (size_t j = 0; j < initial_joint_pose.size(); ++j) {
        if (control_order_joints[i] == initial_joint_pose[j].first) {
          mapping.push_back(j);
          break;
        }
      }
    }
    return mapping;
  }();

  // mapping from joint order to control order
  std::vector<int> joint_to_control_order_mapping = [this]() {
    std::vector<int> mapping;
    for (size_t i = 0; i < initial_joint_pose.size(); ++i) {
      for (size_t j = 0; j < control_order_joints.size(); ++j) {
        if (initial_joint_pose[i].first == control_order_joints[j]) {
          mapping.push_back(j);
          break;
        }
      }
    }
    return mapping;
  }();

};

}  // namespace locomotion_isaacgym
