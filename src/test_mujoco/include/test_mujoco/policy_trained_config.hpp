#pragma once

#include <string>
#include <vector>
#include <utility>
#include <array>

namespace locomotion {

struct PolicyTrainedConfig {
  // Simulation
  double sim_dt    = 1.0 / 200.0;   // 0.002 — matches XML
  int    decimation = 4;             // policy runs every 4 sim steps

  // Observation history
  int obs_history_len = 5;
  int num_obs_terms   = 8;

  // Scaling
  double lin_vel_scale = 1.0;
  double ang_vel_scale = 1.0;
  double dof_pos_scale = 1.0;
  double dof_vel_scale = 1.0;

  // Joint order AND initial pose.
  std::vector<std::pair<std::string, double>> initial_joint_pose = {
    {"left_hip_pitch_joint",   0.3},
    {"right_hip_pitch_joint", -0.3},
    {"left_hip_roll_joint",    0.0},
    {"right_hip_roll_joint",   0.0},
    {"left_knee_joint",        0.6},
    {"right_knee_joint",      -0.6},
  };
  double initial_height = 0.53;

  // Gait command
  double gait_freq     = 1.75;
  double gait_phase    = 0.5;
  double gait_duration = 0.5;

  // Action postprocessing (used in stage 8)
  double action_scale = 0.25;

  // Command velocity
  std::array<double, 3> cmd_vel = {0.0, 0.0, 0.0};

  double stiffness_gain = 40.0;
  double damping_gain   = -2.5;  

  // control order of joints in the policy
  std::vector<std::string> control_order_joints = {
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "left_knee_joint",
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

}  // namespace locomotion