#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "test_mujoco/config.hpp"
#include "test_mujoco/mujoco_sim.hpp"
#include "test_mujoco/mujoco_viewer.hpp"
#include "test_mujoco/observation_builder.hpp"
#include "test_mujoco/torch_policy.hpp"

int main() {
  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("test_mujoco");
  const std::string xml_path    = pkg_share + "/mujoco_xml/SF_biped.xml";
  const std::string policy_path = pkg_share + "/models/exported/policy.pt";

  // ---- Setup ----
  test_mujoco::Config cfg;
  test_mujoco::MujocoSim sim(xml_path);
  test_mujoco::MujocoViewer viewer(sim.model(), sim.data());
  test_mujoco::ObservationBuilder obs_builder(cfg);
  test_mujoco::Policy policy(policy_path);

  // ---- Set gains to match Python (overrides XML defaults) ----
  sim.set_gains(cfg.stiffness_gain, cfg.damping_gain);

  // Joint names and initial positions
  std::vector<std::string> joint_names;
  std::vector<float> initial_joint_pos;
  for (const auto& [n, a] : cfg.initial_joint_pose) {
    joint_names.push_back(n);
    initial_joint_pos.push_back(static_cast<float>(a));
  }
  const int num_joints = static_cast<int>(joint_names.size());

  // ---- Initial pose ----
  sim.reset();
  for (const auto& [name, angle] : cfg.initial_joint_pose) {
    sim.set_joint_position(name, angle);
    sim.set_ctrl(name, angle);
  }
  sim.data()->qpos[2] = cfg.initial_height;
  sim.forward();

  // ---- Loop state ----
  std::vector<float> last_actions(num_joints, 0.0f);
  std::vector<double> last_actions_d(num_joints, 0.0);
  double gait_time = 0.0;
  int step_counter = 0;

  // ---- Prime history ----
  obs_builder.scale_update_observation_history(
      sim.get_mujoco_data(joint_names), gait_time, cfg.cmd_vel, last_actions_d);
  obs_builder.init_history();

  // ---- Verify gains ----
  std::cout << "Gains after override:\n";
  for (int i = 0; i < sim.model()->nu; ++i) {
    const char* name = mj_id2name(sim.model(), mjOBJ_ACTUATOR, i);
    std::cout << "  " << name
              << "  gainprm[0]=" << sim.model()->actuator_gainprm[i * 10 + 0]
              << "  biasprm[1]=" << sim.model()->actuator_biasprm[i * 10 + 1]
              << "  biasprm[2]=" << sim.model()->actuator_biasprm[i * 10 + 2]
              << "\n";
  }

  // ---- Timing ----
  // How many physics steps fit in one rendered frame at 60 fps
  const int steps_per_frame =
      static_cast<int>(std::round((1.0 / 60.0) / sim.time_step()));

  std::cout << "Starting control loop."
            << "  steps_per_frame=" << steps_per_frame
            << "  timestep=" << sim.time_step() << "\n";

  bool running = true;

  while (viewer.is_open() && running) {
    auto frame_start = std::chrono::steady_clock::now();

    // ---- Step physics for one frame's worth of sim time ----
    for (int f = 0; f < steps_per_frame; ++f) {

      // Policy step (every decimation steps)
      if (step_counter % cfg.decimation == 0) {
        gait_time += cfg.sim_dt * cfg.decimation;

        auto raw_obs = sim.get_mujoco_data(joint_names);
        obs_builder.scale_update_observation_history(
            raw_obs, gait_time, cfg.cmd_vel, last_actions_d);


        auto stacked = obs_builder.stacked_observation();
        last_actions = policy.forward(stacked);
        for (auto& a : last_actions) a = std::clamp(a, -100.0f, 100.0f);

        for (int i = 0; i < num_joints; ++i) {
          last_actions_d[i] = static_cast<double>(last_actions[i]);
        }
      }

      // Apply control
      for (int i = 0; i < num_joints; ++i) {
        sim.data()->ctrl[i] =
            last_actions[i] * cfg.action_scale + initial_joint_pos[i];
      }

      // Step physics
      sim.step();
      step_counter++;
    }

    // ---- Render ----
    viewer.render();

    // ---- Frame pacing: sleep to maintain 60 fps ----
    auto frame_end = std::chrono::steady_clock::now();
    double frame_elapsed =
        std::chrono::duration<double>(frame_end - frame_start).count();
    double target_frame_time = 1.0 / 60.0;
    if (frame_elapsed < target_frame_time) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(target_frame_time - frame_elapsed));
    }
  }

  std::cout << "Done. Total steps: " << step_counter << "\n";
  return 0;
}