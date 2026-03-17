#pragma once

#include <cstdint>
#include <atomic>

// Standard cache line size for Jetson Orin NX (ARM Cortex-A78AE)
constexpr size_t CACHE_LINE_SIZE = 64;

struct alignas(CACHE_LINE_SIZE) RobotStateCommand {
    std::atomic<uint32_t> seq_counter{0}; // Sequence counter to avoid race conditions
    uint8_t current_state;
};

struct alignas(CACHE_LINE_SIZE) VelocityCommand {
    std::atomic<uint32_t> seq_counter{0};
    float linear_x;
    float linear_y;
    float angular_z;
    // 12 bytes total. The compiler pads the remaining 52 bytes automatically.
};

struct alignas(CACHE_LINE_SIZE) ImuData {
    std::atomic<uint32_t> seq_counter{0};
    float orientation[3];         // roll, pitch, yaw
    // 12 bytes for orientation + 4 bytes for seq_counter = 16 bytes total. The compiler pads the remaining 48 bytes automatically.
};



// The main Shared Memory block that groups them all together
struct BipedSharedMemory {
    RobotStateCommand robot_state;
    VelocityCommand cmd_vel;
    ImuData imu;
};