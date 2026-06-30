#include "biped_control/policy_isaacgym/observation_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace locomotion_isaacgym {

namespace {

// Reproduce legged_gym's custom get_euler_xyz (atan2-based, range +/- pi).
// Input quaternion components are in (x, y, z, w) order, matching the IMU
// quaternion and the world orientation of the base used in training.
std::array<double, 3> quat_to_euler_xyz(double qx, double qy, double qz, double qw) {
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = qw * qw - qx * qx - qy * qy + qz * qz;
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    double pitch;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = qw * qw + qx * qx - qy * qy - qz * qz;
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

}  // namespace


ObservationBuilder::ObservationBuilder(const PolicyTrainedConfig& cfg) : cfg_(cfg) {
    for (const auto& [_, angle] : cfg_.initial_joint_pose) {
        initial_joint_pos_.push_back(angle);
    }
    // Singles default to zero until first update (matches reset).
    obs_history_.cmd_vel.assign(3, 0.0);
    obs_history_.last_actions.assign(cfg_.control_order_joints.size(), 0.0);
}


void ObservationBuilder::push_recent_observation(
    Deque& d_history, std::vector<double> recent_data) const {
    // Newest frame goes to the front: [current, t-1, ..., t-9].
    d_history.push_front(std::move(recent_data));
    if (static_cast<int>(d_history.size()) > cfg_.obs_history_len) {
        d_history.pop_back();
    }
}


void ObservationBuilder::form_projected_robot_state(
    const RobotSensorData& robot_sensor_data, RobotStateData& obs) const {

    // base orientation -> roll/pitch/yaw (world->base euler, +/- pi)
    obs.rpy = quat_to_euler_xyz(
        robot_sensor_data.imu_quat.x,
        robot_sensor_data.imu_quat.y,
        robot_sensor_data.imu_quat.z,
        robot_sensor_data.imu_quat.w);

    // angular velocity is already in the base frame (gyro on the imu site)
    obs.ang_vel = robot_sensor_data.imu_gyro;

    // joint positions / velocities (control order)
    obs.joint_pos = robot_sensor_data.joint_pos;
    obs.joint_vel = robot_sensor_data.joint_vel;

    // fall detection mirrors check_termination(): |pitch|>1.0 or |roll|>0.8
    obs.fallen = std::abs(obs.rpy[1]) > cfg_.term_pitch ||
                 std::abs(obs.rpy[0]) > cfg_.term_roll;
}


void ObservationBuilder::scale_update_observation_history(
    const RobotSensorData& raw_sensor_data,
    const std::array<double, 3>& cmd_vel,
    const std::vector<double>& last_actions) {

    RobotStateData robot_state;
    form_projected_robot_state(raw_sensor_data, robot_state);

    // rpy (no scaling)
    push_recent_observation(obs_history_.rpy, {
        robot_state.rpy[0],
        robot_state.rpy[1],
        robot_state.rpy[2]});

    // angular velocity scaled
    push_recent_observation(obs_history_.ang_vel, {
        robot_state.ang_vel[0] * cfg_.ang_vel_scale,
        robot_state.ang_vel[1] * cfg_.ang_vel_scale,
        robot_state.ang_vel[2] * cfg_.ang_vel_scale});

    // joint positions relative to default, scaled
    std::vector<double> rel_jp(robot_state.joint_pos.size());
    for (size_t i = 0; i < robot_state.joint_pos.size(); ++i) {
        rel_jp[i] = (robot_state.joint_pos[i] - initial_joint_pos_[i]) * cfg_.dof_pos_scale;
    }
    push_recent_observation(obs_history_.joint_pos_rel, std::move(rel_jp));

    // joint velocity scaled
    std::vector<double> jv(robot_state.joint_vel.size());
    for (size_t i = 0; i < robot_state.joint_vel.size(); ++i) {
        jv[i] = robot_state.joint_vel[i] * cfg_.dof_vel_scale;
    }
    push_recent_observation(obs_history_.joint_vel, std::move(jv));

    // command velocity (single frame): vx, vy by lin_vel scale; yaw by ang_vel scale
    obs_history_.cmd_vel = {
        cmd_vel[0] * cfg_.cmd_scale[0],
        cmd_vel[1] * cfg_.cmd_scale[1],
        cmd_vel[2] * cfg_.cmd_scale[2]};

    // last actions (single frame, no scaling)
    obs_history_.last_actions = last_actions;
}


void ObservationBuilder::init_history() {
    for (auto* d : obs_history_.history_buffers()) {
        if (d->empty()) {
            throw std::runtime_error(
                "history buffer is empty... call scale_update_observation_history first");
        }
        // Zero-pad the past frames (b1..b9 start at zero in training).
        const std::vector<double> zeros(d->front().size(), 0.0);
        while (static_cast<int>(d->size()) < cfg_.obs_history_len) {
            d->push_back(zeros);
        }
    }
}


std::vector<float> ObservationBuilder::stacked_observation() const {
    std::vector<float> out;
    out.reserve(189);

    const double clip = cfg_.clip_observations;
    auto emit = [&out, clip](double v) {
        out.push_back(static_cast<float>(std::clamp(v, -clip, clip)));
    };
    auto emit_history = [&](const Deque& d) {
        if (static_cast<int>(d.size()) < cfg_.obs_history_len) {
            throw std::runtime_error("history of data not initialised");
        }
        for (const auto& frame : d) {
            for (double v : frame) emit(v);
        }
    };

    // Exact training term order:
    //   rpy | ang_vel | cmd | joint_pos_rel | joint_vel | last_action
    emit_history(obs_history_.rpy);
    emit_history(obs_history_.ang_vel);
    for (double v : obs_history_.cmd_vel) emit(v);
    emit_history(obs_history_.joint_pos_rel);
    emit_history(obs_history_.joint_vel);
    for (double v : obs_history_.last_actions) emit(v);

    return out;
}

}  // namespace locomotion_isaacgym
