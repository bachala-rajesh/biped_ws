#ifndef ROBOT_SHARED_MEMORY_HPP
#define ROBOT_SHARED_MEMORY_HPP

#include <cstdint>

#define CACHE_LINE 64

struct RobotSharedMemory {
    // --- Header (8 bytes) ---
    uint64_t timestamp_shim_ns;

    // --- IMU Block  ---
    alignas(CACHE_LINE) uint32_t imu_seq;
    float imu_data[4]; 

    // --- Joy Block  ---
    alignas(CACHE_LINE) uint32_t joy_seq;
    float joy_data[4];
    bool emergency_stop_cmd;

    // --- Policy Block  ---
    alignas(CACHE_LINE) uint32_t action_seq;
    float target_torques[12];
};

#endif