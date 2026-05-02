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
#include <math.h>
#include "robot_states_enum.hpp"
#include "mujoco_ros2_control_msgs/srv/reset_world.hpp"
#include <yasmin_ros/service_state.hpp>



using std::placeholders::_1;
using std::placeholders::_2;



namespace biped_fsm {


// --------------------------------------------------------------------------------------
// -------------------------------------- FSM states ------------------------------------
// --------------------------------------------------------------------------------------


class MujocoInitState : public yasmin_ros::ServiceState<mujoco_ros2_control_msgs::srv::ResetWorld> 
{
    public:
        MujocoInitState() : yasmin_ros::ServiceState<mujoco_ros2_control_msgs::srv::ResetWorld>(
            "/mujoco_ros2_control_node/reset_world", 
            std::bind(&MujocoInitState::create_request_handler, this, _1),
            {"ready"},
            std::bind(&MujocoInitState::response_handler, this, _1, _2)),keyframe_("home")
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                YASMIN_LOG_INFO("resetting the mujoco world");
            }
            
            private: 
            std::string keyframe_;
            
        mujoco_ros2_control_msgs::srv::ResetWorld::Request::SharedPtr create_request_handler( yasmin::Blackboard::SharedPtr blackboard)
        {
            blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::INIT); 
            auto request = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
            request->keyframe = keyframe_;
            return request;
        }

        std::string response_handler(std::shared_ptr<yasmin::Blackboard> blackboard, mujoco_ros2_control_msgs::srv::ResetWorld::Response::SharedPtr response)
        {
            if (response->success) {
                YASMIN_LOG_INFO("ResetWorld success: %s", response->message.c_str());
            } else {
                YASMIN_LOG_WARN("ResetWorld failed: %s", response->message.c_str());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));


            return "ready";
        }


    

};



class MujocoActiveState : public yasmin::State {
public:
    MujocoActiveState() : yasmin::State({"reset"}) {}
    std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override {
        blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::ACTIVE); 
        std::string joystate;

        while (true) {

            // get the latest joy state
            joystate = blackboard->get<std::string>("joy_state");

            // check for user reset
            if (joystate == "start") return "reset";
            
        }
    }
};




} // namespace biped_fsm
