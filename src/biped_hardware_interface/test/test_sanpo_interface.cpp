#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
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
#include "sanpo_interface.h"

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
    spdlog::set_level(spdlog::level::debug);

    // 3. Use uppercase macros to capture the exact line number
    SPDLOG_INFO("Test of helper script of hardware interface.");

    std::string port = PORT_CAN_CHANNEL_1; // e.g., "/dev/ttyACM0"
    uint32_t baud = BAUD_RATE;             // 1000000
    uint8_t can_channel = 1;
    std::vector<uint8_t> test_motors = {1}; 
    std::unordered_map<uint8_t, MotorState> motor_states;

    // initialize motor states with default values for all test motors
    for (auto motor_id : test_motors) {
        motor_states[motor_id] = MotorState{motor_id, 0.0f, 0.0f, 0.0f, 0.0f};
    }


    // init SanpoInterface instance
    SanpoInterface sanpo_comm(port, baud, can_channel, test_motors, motor_states);

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

    std::this_thread::sleep_for(std::chrono::seconds(5)); // wait for 5 seconds before starting the test
    if (sanpo_comm.disconnect_sanpo_and_motors()) {
        SPDLOG_INFO("Disconnected from SANPO successfully after test.");
    } else {
        SPDLOG_ERROR("Failed to disconnect from SANPO after failed motor enable.");
        return -1;
    }


    return 0;
}

