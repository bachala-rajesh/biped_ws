/*
TODO:
- Add joint states check in the passive to standby mode
*/

#pragma once

#include "yasmin/state.hpp"
#include "yasmin/blackboard.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "biped_fsm/robot_states_enum.hpp"
#include <math.h>
#include "robot_states_enum.hpp"

namespace biped_fsm {

bool check_for_user_interrupt(yasmin::State* state) {
    // Check if Ctrl+C was pressed (Yasmin's built-in cancel flag)
    if (state->is_canceled()) { 
        
        // Return an EMPTY string. This completely breaks the FSM loop.
        std::cout << "\n interrupted by user. Exiting..." << std::endl;
        return true;
    }
    return false;
}

bool check_imu_health(std::shared_ptr<yasmin::Blackboard> blackboard) {
    // check if the imu is alive or not

    if (!blackboard->get<bool>("imu_status")) {
        YASMIN_LOG_ERROR("[ERROR] IMU is not connected!");
        return false;
    } 

    // check if the imu data is recent enough
    double last_imu_time = blackboard->get<double>("last_imu_time");
    double current_time = blackboard->get<double>("current_ros_time");

    if (current_time - last_imu_time > robot_states_config::kImuFailureTimeoutSec.count()) {
        YASMIN_LOG_ERROR("[ERROR] IMU data is outdated!");
        return false;
    }

    return true;
}


bool check_joint_states_health(std::shared_ptr<yasmin::Blackboard> blackboard) {
    // check if the joint_states data is recent or not

    if (!blackboard->get<bool>("joint_states_status")) {
        YASMIN_LOG_ERROR("[ERROR] joint_states data is not available!");
        return false;
    } 

    // check if the joint_states data is recent enough
    double last_joint_states_time = blackboard->get<double>("last_joint_states_time");
    double current_time = blackboard->get<double>("current_ros_time");

    if (current_time - last_joint_states_time > robot_states_config::kJointStatesFailureTimeoutSec.count()) {
        YASMIN_LOG_ERROR("[ERROR] joint_states data is outdated!");
        return false;
    }

    return true;
}

bool check_joy_state_health(std::shared_ptr<yasmin::Blackboard> blackboard) {
    // check if the joy_state data is present or not

    if (!blackboard->get<bool>("joy_state_status")) {
        YASMIN_LOG_ERROR("[ERROR] joystick state data in not available!");
        return false;
    } 

    return true;
}

float rad_to_deg(float rad) {
    return rad * 180.0 / M_PI;
}

float deg_to_rad(float deg) {
    return deg * M_PI / 180.0;
}

bool fall_detection(std::shared_ptr<yasmin::Blackboard> blackboard) {
    // check the robot has fallen or not based on the imu data
    double roll = std::abs(blackboard->get<double>("imu_roll"));
    double pitch = std::abs(blackboard->get<double>("imu_pitch"));

    if (roll > deg_to_rad(robot_states_config::kRollFallDetectionThresholdDeg) || pitch > deg_to_rad(robot_states_config::kPitchFallDetectionThresholdDeg)) {
        YASMIN_LOG_WARN("[WARNING] Fall detected!");
        return true;
    }
    return false;
}

// --------------------------------------------------------------------------------------
// -------------------------------------- FSM states ------------------------------------
// --------------------------------------------------------------------------------------

class InitState : public yasmin::State {
public:
    InitState() : yasmin::State({"ready", "imu_failure", "joint_states_failure", "joy_failure", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::INIT); 

        YASMIN_LOG_INFO("Initializing robot sensors...");
        std::this_thread::sleep_for(robot_states_config::kInitDelaySec); 

        auto last_time = std::chrono::steady_clock::now();

        while (true) {
            YASMIN_LOG_INFO("Checking health of IMU, jointstates and joy states...");
            bool imu_health = check_imu_health(blackboard);
            bool joint_states_health = check_joint_states_health(blackboard);
            bool joy_state_health = check_joy_state_health(blackboard);

            if (imu_health && joint_states_health && joy_state_health) {
                break;
            }

            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_time >= robot_states_config::kWaitTimeIntervalSec) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        //check for user interrupt
        if (check_for_user_interrupt(this)) return "system_shutdown";

        if (!check_imu_health(blackboard)) {
            return "imu_failure";
        } 
        if (!check_joint_states_health(blackboard)) {
            return "joint_states_failure";
        }
        if (!check_joy_state_health(blackboard)) {
            return "joy_failure";
        }

        YASMIN_LOG_INFO("All sensors are healthy. Robot is ready!");
        std::this_thread::sleep_for(robot_states_config::kInitDelaySec); 
        return "ready";
    }
};

class PassiveState : public yasmin::State {
public:
    PassiveState() : yasmin::State({"user_start", "imu_failure", "joint_states_failure", "fall_detected", "user_stop", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::PASSIVE); 

        bool start_button_hold = false;
        std::string joystate;
        std::chrono::steady_clock::time_point last_joy_start_time;
 
        while (true) {
            // check for imu health
            if (!check_imu_health(blackboard)) return "imu_failure";

            // check for joint_states health
            if (!check_joint_states_health(blackboard)) return "joint_states_failure";

            // check for fall detection
            if (fall_detection(blackboard)) return "fall_detected";

            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for user stop
            if (joystate == "stop" || joystate == "emergency_stop") return "user_stop";

            // check for user start
            if (joystate == "start") {
                if (!start_button_hold) {
                    start_button_hold = true;
                    last_joy_start_time = std::chrono::steady_clock::now();
                }
                else {
                    auto current_time = std::chrono::steady_clock::now();
                    if (current_time - last_joy_start_time > robot_states_config::kJoyStartButtonHoldSec) {
                        YASMIN_LOG_INFO("User has started the robot!");
                        return "user_start";
                    }
                }
            }
            else {
                start_button_hold = false;
            }

            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
        }
    }
};

class StandbyState : public yasmin::State {
public:
    StandbyState() : yasmin::State({"user_active", "user_stop", "imu_failure", "joint_states_failure", "fall_detected", "emergency_stop", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {

        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::STANDBY);
        std::string joystate;

        while (true) {
            // check for imu health
            if (!check_imu_health(blackboard)) return "imu_failure";

            // check for joint_states health
            if (!check_joint_states_health(blackboard)) return "joint_states_failure";

            // check for fall detection
            if (fall_detection(blackboard)) return "fall_detected";

            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for user active
            if (joystate == "active") return "user_active";
            
            // check for emergency stop
            if (joystate == "emergency_stop") return "emergency_stop";
            
            // check for user stop
            if (joystate == "stop") return "user_stop";
           
            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        }
    }
};


class ActiveState : public yasmin::State {
public:
    ActiveState() : yasmin::State({"fall_detected", "imu_failure", "joint_states_failure", "user_standby", "emergency_stop", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::ACTIVE); 
        std::string joystate;

        while (true) {
            // check for imu health
            if (!check_imu_health(blackboard)) return "imu_failure";

            // check for joint_states health
            if (!check_joint_states_health(blackboard)) return "joint_states_failure";

            // check for fall detection
            if (fall_detection(blackboard)) return "fall_detected";

            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for user standby
            if (joystate == "standby") return "user_standby";
            
            // check for emergency stop
            if (joystate == "emergency_stop") return "emergency_stop";
            
            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        }
    }
};

class FallenState : public yasmin::State {
public:
    FallenState() : yasmin::State({"reset", "emergency_stop", "user_stop", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::FALLEN); 
        std::string joystate;

        while (true){
            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for reset
            if (joystate == "start") return "reset";
            
            // check for emergency stop
            if (joystate == "emergency_stop") return "emergency_stop";
            
            // check for user stop
            if (joystate == "stop") return "user_stop";

            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
};

class ErrorState : public yasmin::State {
public:
    ErrorState() : yasmin::State({"recovered", "emergency_stop", "user_stop", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {

        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::ERROR); 
        std::string joystate;

        while (true){
            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for recovered
            if (joystate == "start") return "recovered";

            // check for emergency stop
            if (joystate == "emergency_stop") return "emergency_stop";

            // check for user stop
            if (joystate == "stop") return "user_stop";

            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

class StopState : public yasmin::State {
public:
    StopState() : yasmin::State({"reset", "system_shutdown"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::STOP); 

        std::string joystate;

        while (true){
            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for reset
            if (joystate == "start") return "reset";

            //check for user interrupt
            if (check_for_user_interrupt(this)) return "system_shutdown";

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};




} // namespace biped_fsm
