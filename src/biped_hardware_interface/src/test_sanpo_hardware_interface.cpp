#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN
// 1. You MUST define this before including spdlog to enable SPDLOG_DEBUG

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>
#include <cstdint>
#include <iomanip>
#include <fmt/core.h>
#include <array>
#include <vector>
#include <cmath>
#include <serial/serial.h>
#include <thread>
#include <chrono>
#include "motor_structs.h"
#include "sanpo_hardware_interface.h"

using Clock = std::chrono::steady_clock;

#define PORT_CAN_CHANNEL_1 "/dev/ttyACM0"
#define PORT_CAN_CHANNEL_2 "/dev/ttyACM1"
#define BAUD_RATE 1000000

int main() {
    auto color_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    color_sink->set_color(spdlog::level::info, color_sink->white);
    color_sink->set_color(spdlog::level::debug, color_sink->blue);
    color_sink->set_color(spdlog::level::err, color_sink->red);
    color_sink->set_color(spdlog::level::warn, color_sink->yellow); 

    auto logger = std::make_shared<spdlog::logger>("my_logger", color_sink);
    spdlog::set_default_logger(logger);
    
    // Set the new pattern (Level, Line Number, Message)
    // spdlog::set_pattern("[%^%l%$] [Line %#] %v");
    spdlog::set_pattern("%^[%l] [Line %#] %v%$");
    spdlog::set_level(spdlog::level::info);

    // 3. Use uppercase macros to capture the exact line number
    SPDLOG_INFO("Test of helper script of hardware interface.");

    std::string port = PORT_CAN_CHANNEL_1; // e.g., "/dev/ttyACM0"
    uint32_t baud = BAUD_RATE;             // 1000000
    uint8_t can_channel = 1;
    std::vector<uint8_t> test_motors = {1,2,3}; 
    std::unordered_map<uint8_t, MotorState> motor_states;

    // initialize motor states with default values for all test motors
    for (auto motor_id : test_motors) {
        motor_states[motor_id] = MotorState{motor_id, 0.0f, 0.0f, 0.0f, 0.0f};
    }


    // init SanpoHardwareInterface instance
    SanpoHardwareInterface sanpo_comm(port, baud, can_channel, test_motors, motor_states);

    // connect to the sanpo board
    if (!sanpo_comm.connect_sanpo()) {
        SPDLOG_ERROR("Failed to connect to SANPO...");
        return -1;
    }
    else {
        SPDLOG_INFO("Connected to SANPO successfully.");
    }

    // enable motors
    std::vector<uint8_t> failed_motors = sanpo_comm.enable_motors();
    if (!failed_motors.empty()) {
        SPDLOG_ERROR("Failed to enable motors: {}", fmt::join(failed_motors, ", "));
        if (!sanpo_comm.disconnect_sanpo_and_motors()) {
            SPDLOG_ERROR("Failed to disconnect from SANPO after failed motor enable.");
        }
        return -1;
    }
    else {
        SPDLOG_INFO("All motors enabled successfully in the port {}.", port);
    }

    SPDLOG_INFO("Starting SINE WAVE TEST...");

    // --- Control Loop Parameters ---
    const double amplitude = 1.57; // Radians
    const double period = 6.0;     // Seconds for one full wave
    const float kp = 30.0f;        // Position stiffness (Start low!)
    const float kd = 1.0f;         // Velocity damping
    double omega = 2.0 * M_PI / period;

    // timing setup (100 hz)
    const int loop_rate_hz = 200;
    const auto loop_duration = std::chrono::microseconds(1000000 / loop_rate_hz);
    const double test_duration_seconds = 12.0;

    auto start_time = Clock::now();
    auto next_loop_time = start_time + loop_duration;

    while (true) {
        auto current_time = Clock::now();
        std::chrono::duration<double> elapsed_time = current_time - start_time;
        double t = elapsed_time.count();

        // exit after the test duration
        if (t > test_duration_seconds) {
            break;  
        }

        // compute the desired position for the sine wave
        float target_position = static_cast<float>(amplitude * std::sin(omega * t));
        float target_velocity = static_cast<float>(amplitude * omega * std::cos(omega * t));

        // command the motors
        std::vector<MotorCommand> motor_commands;
        for (auto motor_id : test_motors) {
            motor_commands.push_back(MotorCommand(motor_id, target_position, target_velocity, kp, kd, 0.0f));
        }
        sanpo_comm.write_motor_commands(motor_commands);

        // read the response and update motor states
        sanpo_comm.update_motor_states();
        // log the updated motor state for debug
        SPDLOG_INFO("Commanded Target Position: {:.2f} deg, motor- {:.2f} deg, {:.2f} deg, {:.2f} deg", 
            target_position * 180.0f / M_PI, 
            motor_states[test_motors[0]].filtered_position * 180.0f / M_PI, 
            motor_states[test_motors[1]].filtered_position * 180.0f / M_PI, 
            motor_states[test_motors[2]].filtered_position * 180.0f / M_PI
        );

        // sleep until the next loop time
        std::this_thread::sleep_until(next_loop_time);
        next_loop_time += loop_duration;
    }

    SPDLOG_INFO("SINE WAVE TEST completed.");

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100 milliseconds before starting the test
    if (sanpo_comm.disconnect_sanpo_and_motors()) {
        SPDLOG_INFO("Disconnected from SANPO successfully after test.");
    } else {
        SPDLOG_ERROR("Failed to disconnect from SANPO after failed motor enable.");
        return -1;
    }


    return 0;
}

