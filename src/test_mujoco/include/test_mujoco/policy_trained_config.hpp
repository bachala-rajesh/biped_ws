#pragma once

#include <string>
#include <vector>
#include <utility>
#include <array>

namespace locomotion {

struct PolicyTrainedConfig {
  // Simulation
  double sim_dt    = 1.0 / 200.0;   // 0.005 — matches XML timestep
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
    {"left_hip_roll_joint",    0.0},
    {"left_knee_joint",        0.6},
    {"right_hip_pitch_joint", -0.3},
    {"right_hip_roll_joint",   0.0},
    {"right_knee_joint",      -0.6},
  };
  double initial_height = 0.57;

  // Gait command
  double gait_freq     = 2.0;
  double gait_phase    = 0.5;
  double gait_duration = 0.5;

  // Action postprocessing (used in stage 8)
  double action_scale = 0.25;

  // Command velocity
  std::array<double, 3> cmd_vel = {0.0, 0.0, 0.0};

  double stiffness_gain = 40.0;
  double damping_gain   = 4.0;  


};

}  // namespace locomotion