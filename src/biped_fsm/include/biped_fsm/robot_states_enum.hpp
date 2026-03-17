#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <chrono>

using namespace std::chrono_literals;

namespace robot_states_config {
    constexpr double kRollFallDetectionThresholdDeg = 15.0;
    constexpr double kPitchFallDetectionThresholdDeg = 15.0;
    constexpr std::chrono::duration<double> kImuFailureTimeoutSec = 0.5s;
    constexpr std::chrono::duration<double> kJointStatesFailureTimeoutSec = 0.5s;
    constexpr std::chrono::duration<double> kInitDelaySec = 1.0s;
    constexpr std::chrono::duration<double> kJoyStartButtonHoldSec = 2.0s;
    constexpr std::chrono::duration<double> kWaitTimeIntervalSec = 3.0s;
}


namespace robot_states_enum {

    // Define the enum
    enum class RobotState : uint8_t {
        INIT = 10, 
        PASSIVE = 20, 
        STANDBY = 30, 
        ACTIVE = 40,
        FALLEN = 50, 
        ERROR = 60, 
        STOP = 70
    };

    // C++17 inline map connects the string to the enum
    inline const std::unordered_map<std::string, RobotState> ROBOT_STATE_MAP = {
        {"INIT", RobotState::INIT}, 
        {"PASSIVE", RobotState::PASSIVE}, 
        {"STANDBY", RobotState::STANDBY}, 
        {"ACTIVE", RobotState::ACTIVE},
        {"FALLEN", RobotState::FALLEN}, 
        {"ERROR", RobotState::ERROR}, 
        {"STOP", RobotState::STOP}
    };

    // Helper function to keep your node completely clean
    inline uint8_t get_state_enum_value(const std::string& state_str) {
        auto it = ROBOT_STATE_MAP.find(state_str);
        if (it != ROBOT_STATE_MAP.end()) {
            return static_cast<uint8_t>(it->second);
        }
        return static_cast<uint8_t>(RobotState::ERROR); // Default safety fallback
    }

} // namespace robot_states_enum