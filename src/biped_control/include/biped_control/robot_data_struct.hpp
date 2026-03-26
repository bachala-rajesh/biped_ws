#pragma once

#include <cstdint>

struct RobotData {
    // robot state
    uint8_t current_robot_state;

    // imu
    float imu_roll, imu_pitch, imu_yaw;

    // cmd_vel
    float cmd_linear_x, cmd_linear_y, cmd_angular_z;
};