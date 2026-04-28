#include "test_mujoco/observation_builder.hpp"

#include <cmath>
#include <stdexcept>

namespace test_mujoco {

namespace {
    constexpr double kTwoPi = 6.283185307179586;
}

ObservationBuilder::ObservationBuilder(const Config& cfg): cfg_(cfg) {
        for (const auto& [_, angle] : cfg_.initial_joint_pose) {
            initial_joint_pos_.push_back(angle);
        }
}

void ObservationBuilder::push_recent_observation(std::deque<std::vector<double>>& d_history, std::vector<double> recent_data) const {
    d_history.push_back(recent_data);
    if (d_history.size() > cfg_.obs_history_len) {
        d_history.pop_front();
    }
}


void ObservationBuilder::scale_update_observation(
    const MujocoObservation& raw,
    double gait_time,
    const std::array<double, 3>& cmd_vel,
    const std::vector<double>& last_actions)  {

        // angular velocity scaled
        push_recent_observation(obs_history_.ang_vel, {
            raw.ang_vel[0] * cfg_.ang_vel_scale,
            raw.ang_vel[1] * cfg_.ang_vel_scale,
            raw.ang_vel[2] * cfg_.ang_vel_scale });

        // projected gravity (no scaling)
        push_recent_observation(obs_history_.proj_gravity, {
            raw.proj_gravity[0],
            raw.proj_gravity[1],
            raw.proj_gravity[2] });

        // command velocity scaled
        push_recent_observation(obs_history_.cmd_vel, {
            cmd_vel[0] * cfg_.lin_vel_scale,
            cmd_vel[1] * cfg_.lin_vel_scale,
            cmd_vel[2] * cfg_.lin_vel_scale });

        // joint positions scaled
        std::vector<double> rel_jp(raw.joint_pos.size());
        for (size_t i = 0; i < raw.joint_pos.size(); ++i) {
            rel_jp[i] = (raw.joint_pos[i] - initial_joint_pos_[i]) * cfg_.dof_pos_scale;
        }
        push_recent_observation(obs_history_.joint_pos_rel, std::move(rel_jp));

        // joint velocity scaled
        std::vector<double> jv(raw.joint_vel.size());
        for (size_t i = 0; i < raw.joint_vel.size(); ++i) {
            jv[i] = raw.joint_vel[i] * cfg_.dof_vel_scale;
        }
        push_recent_observation(obs_history_.joint_vel, std::move(jv));

        // last actions no scaling
        push_recent_observation(obs_history_.last_actions, last_actions);

        // gait_phase
        const double phase = std::fmod(gait_time * cfg_.gait_freq, 1.0);
        push_recent_observation(obs_history_.gait_phase, 
            {std::sin(kTwoPi * phase),
             std::cos(kTwoPi * phase)});

        // gait command
        push_recent_observation(obs_history_.gait_command, 
            {cfg_.gait_freq,
            cfg_.gait_phase,
            cfg_.gait_duration });
}


void ObservationBuilder::init_history() {
    for(auto* d : obs_history_.all_buffers()) {
        if (d->empty()) throw std::runtime_error("history buffer is empty... call scale_update_observation first");
        const auto snap = d->back();
        while ((int)d->size() < cfg_.obs_history_len) {
            d->push_back(snap);
        }
    }
}



std::vector<float> ObservationBuilder::stacked_observation() const {
    std::vector<float> out;
    for (const auto*d : obs_history_.all_buffers()) {
        if ((int)d->size() < cfg_.obs_history_len) {
            throw std::runtime_error("history of data not initialised");
        }

        for (const auto& term : *d) {
            for (double v : term) {
                out.push_back(static_cast<float>(v));
            }
        }
    }
    return out;

}
}
