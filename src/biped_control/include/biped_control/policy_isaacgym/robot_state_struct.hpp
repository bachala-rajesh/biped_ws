#pragma once

#include <array>
#include <vector>

// Helper data structures for the IsaacGym `biped_walk_flat` walking policy.
//
// Kept in a dedicated `locomotion_isaacgym` namespace so this policy can be
// built and used alongside the original IsaacLab helpers in `locomotion`
// without any symbol clashes. `RobotSensorData` is intentionally identical in
// layout to `locomotion::RobotSensorData` so the same sensor-reading code in a
// controller can populate either one.

namespace locomotion_isaacgym {

// Robot state projected into the base frame, as consumed by the policy.
// IsaacGym `biped_walk_flat` uses roll/pitch/yaw euler angles (NOT projected
// gravity) and does not use base linear velocity in its observation.
struct RobotStateData {
    std::array<double, 3> rpy;        // roll, pitch, yaw  (base orientation, world->base)
    std::array<double, 3> ang_vel;    // base-frame angular velocity (gyro)
    std::vector<double> joint_pos;    // joint positions, control order
    std::vector<double> joint_vel;    // joint velocities, control order
    bool fallen = false;
};

struct IMU_QUATERNION {
    double x;
    double y;
    double z;
    double w;
};

struct RobotSensorData {
    IMU_QUATERNION imu_quat;
    std::array<double, 3> imu_gyro;
    std::array<double, 3> imu_accel;
    std::array<double, 3> lin_vel_world;   // unused by this policy, kept for parity
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
};

}  // namespace locomotion_isaacgym
