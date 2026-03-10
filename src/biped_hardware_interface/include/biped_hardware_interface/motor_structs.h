#pragma once

#include <cstdint>

struct MotorState {
    uint8_t id;
    float raw_position;
    float filtered_position = 0.0f; 
    float velocity;
    float torque;

    MotorState() : id(0), raw_position(0.0), filtered_position(0.0), velocity(0.0), torque(0.0) {}
    MotorState(int id, float raw_pos, float filtered_pos, float vel, float tor)
        : id(id), raw_position(raw_pos), filtered_position(filtered_pos), velocity(vel), torque(tor) {}
};


struct MotorCommand {
    uint8_t id;
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;

    MotorCommand() : id(0), position(0.0), velocity(0.0), kp(0.0), kd(0.0), torque(0.0) {}
    MotorCommand(uint8_t id, float p, float v, float kp_val, float kd_val, float t)
        : id(id), position(p), velocity(v), kp(kp_val), kd(kd_val), torque(t) {}
};

