#pragma once


#include <array>
#include <vector>

namespace test_mujoco {

    struct RobotState {
        std::array<double, 3> proj_gravity;
        std::array<double, 3> lin_vel;
        std::array<double, 3> ang_vel;
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
        bool fallen = false;
    };

    struct RobotSensorData {
        std::array<double, 4> imu_quat;
        std::array<double, 3> imu_gyro;
        std::array<double, 3> imu_accel;
        std::array<double, 3> lin_vel_world;        // linear velocity in the world frame
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
    };
}
