#pragma once

#include <cstdint>
#include <atomic>

// Standard cache line size for Jetson Orin NX (ARM Cortex-A78AE)
constexpr size_t CACHE_LINE_SIZE = 64;

enum class RobotState : uint8_t {
    INIT = 10, 
    PASSIVE = 20, 
    STANDBY = 30, 
    ACTIVE = 40,
    FALLEN = 50, 
    ERROR = 60, 
    STOP = 70
};

// Thread 1 can write to this...
struct alignas(CACHE_LINE_SIZE) StateCommand {
    std::atomic<uint32_t> seq_counter{0}; // Sequence counter to avoid race conditions
    RobotState current_state;
};

// ...while Thread 2 writes to this, with zero CPU wait time.
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
    float angular_velocity[3];    // x, y, z
    float linear_acceleration[3]; // x, y, z
    // 40 bytes total. Safely fits inside one 64-byte cache line.
};

struct alignas(CACHE_LINE_SIZE) JointData {
    std::atomic<uint32_t> seq_counter{0};
    // 6 DOF biped = 6 joints. 
    float position[6];
    float velocity[6];
    // 48 bytes total. Safely fits inside one 64-byte cache line.
};

// The main Shared Memory block that groups them all together
struct BipedSharedMemory {
    StateCommand state;
    VelocityCommand cmd_vel;
    ImuData imu;
    JointData joints;
};