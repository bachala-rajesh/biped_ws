#pragma once


#include <array>
#include <vector>

namespace locomotion {

    struct RobotStateData {
        std::array<double, 3> proj_gravity;
        std::array<double, 3> lin_vel;
        std::array<double, 3> ang_vel;
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
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
        std::array<double, 3> lin_vel_world;        // linear velocity in the world frame
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
    };
}
