#pragma once

#include <array>
#include <deque>
#include <vector>

#include <biped_control/policy_isaacgym/policy_trained_config.hpp>
#include <biped_control/policy_isaacgym/robot_state_struct.hpp>

// Observation builder for the IsaacGym `biped_walk_flat` policy.
//
// Reproduces legged_gym `compute_observations()` exactly:
//   - 4 history terms (rpy, ang_vel, joint_pos_rel, joint_vel), each a
//     10-frame window ordered [current, t-1, ..., t-9];
//   - command velocity and last action are SINGLE-frame terms;
//   - the 9 past frames start at zero (matching reset_idx) instead of being
//     back-filled with the first sample.
//
// Differences vs. the IsaacLab `locomotion::ObservationBuilder`:
//   - rpy euler angles instead of projected gravity, no gait terms;
//   - newest-first stacking (push_front) vs. oldest-first;
//   - per-component command scaling and an observation clip.

namespace locomotion_isaacgym {

using Deque = std::deque<std::vector<double>>;

constexpr int n_history_terms = 4;

struct ObservationHistory {
    // History terms (10-frame window each), newest at front.
    Deque rpy;             // 3
    Deque ang_vel;         // 3
    Deque joint_pos_rel;   // 6
    Deque joint_vel;       // 6

    // Single-frame terms.
    std::vector<double> cmd_vel;       // 3
    std::vector<double> last_actions;  // 6

    std::array<Deque*, n_history_terms> history_buffers() {
        return {&rpy, &ang_vel, &joint_pos_rel, &joint_vel};
    }

    std::array<const Deque*, n_history_terms> history_buffers() const {
        return {&rpy, &ang_vel, &joint_pos_rel, &joint_vel};
    }
};


class ObservationBuilder {
public:
    explicit ObservationBuilder(const PolicyTrainedConfig& cfg);

    // Read raw state, scale, and push the current frame onto every history term.
    // `last_actions` are the RAW policy outputs from the previous step.
    void scale_update_observation_history(
        const RobotSensorData& raw_sensor_data,
        const std::array<double, 3>& cmd_vel,
        const std::vector<double>& last_actions);

    // Project raw sensor data into the base-frame state used by the policy.
    void form_projected_robot_state(const RobotSensorData& robot_sensor_data, RobotStateData& obs) const;

    // Zero-pad the 9 past frames so every history term has obs_history_len
    // frames. Call once after the first scale_update_observation_history().
    void init_history();

    // Flatten into the 189-d observation in the exact training term order:
    //   rpy(hist) | ang_vel(hist) | cmd(1) | joint_pos_rel(hist) | joint_vel(hist) | last_action(1)
    // Clipped to +/- clip_observations.
    std::vector<float> stacked_observation() const;

private:
    // Push to FRONT (newest-first) and cap the window at obs_history_len.
    void push_recent_observation(Deque& d_history, std::vector<double> recent_data) const;

    PolicyTrainedConfig cfg_;
    std::vector<double> initial_joint_pos_;
    ObservationHistory obs_history_;
};

}  // namespace locomotion_isaacgym
