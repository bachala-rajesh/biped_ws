// Copyright (c) 2026, bachala rajesh
// Copyright (c) 2026, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "biped_control/blind_walk_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "biped_fsm/robot_states_enum.hpp"


using BipedJointCommand = biped_msgs::msg::BipedJointCommand;
using ControllerCommandPublisher = realtime_tools::RealtimePublisher<BipedJointCommand>;

using namespace iox2;
#define IOX_IMU_TOPIC "iox/imu_data"
#define IOX_FSM_TOPIC "iox/fsm_data"

namespace biped_control
{
  BlindWalkController::BlindWalkController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn BlindWalkController::on_init()
  {

    try
    {
      param_listener_ = std::make_shared<blind_walk_controller::ParamListener>(get_node());
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BlindWalkController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();

    if (!params_.state_joints.empty())
    {
      state_joints_ = params_.state_joints;
    }
    else
    {
      state_joints_ = params_.joints;
    }

    if (params_.joints.size() != state_joints_.size())
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
        params_.joints.size(), state_joints_.size());
      return CallbackReturn::FAILURE;
    }

    // initialize the iox2 services and subscribers
    auto result =init_iox2_services();  
    if (result != controller_interface::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize iox2 services");
      return result;
    }  
    
    policy_commands_msg_ = BipedJointCommand();
    

    // resize the messages based on the number of joints
    resize_msgs_based_on_joints();
    
    try
    {
      // State publisher
      ros_policy_publisher_ =
        get_node()->create_publisher<BipedJointCommand>("~/policy_joint_commands", rclcpp::SystemDefaultsQoS());
      rt_policy_publisher_ = std::make_unique<ControllerCommandPublisher>(ros_policy_publisher_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }


  controller_interface::InterfaceConfiguration BlindWalkController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(params_.joints.size());  // position
    // Request 1 command interface per joint: position
    for (const auto & joint : params_.joints)
    {
      command_interfaces_config.names.push_back(joint + "/position");
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration BlindWalkController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(state_joints_.size() * 2);  // position, velocity

    // Request 2 state interfaces per joint: position and velocity
    for (const auto & joint : state_joints_)
    {
      state_interfaces_config.names.push_back(joint + "/position");
      state_interfaces_config.names.push_back(joint + "/velocity");
    }

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn BlindWalkController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // set the initial commands to prevent kickbacks
    // loop through all the joints
    for (size_t i = 0; i <  state_joints_.size(); ++i) {
      size_t cmd_base_idx = i*CMD_IFC_PER_JOINT; 
      size_t state_base_idx = i*STATE_IFC_PER_JOINT; 

      // read current position from state interface
      double current_position = state_interfaces_[state_base_idx + STATE_POS_ITFS].get_value();

      // set position command to current position to prevent jumps
      command_interfaces_[cmd_base_idx].set_value(current_position);
      
      //populate the policy command message with initial joint positions
      policy_commands_msg_.position[i] = current_position;


      RCLCPP_INFO(get_node()->get_logger(), "Activated joint %s with initial position %.2f", state_joints_[i].c_str(), current_position);
    }

    // publish the initial 
    if (rt_policy_publisher_ && rt_policy_publisher_->trylock()) {
      rt_policy_publisher_->msg_ = policy_commands_msg_;
      rt_policy_publisher_->msg_.header.stamp = get_node()->now();
      rt_policy_publisher_->unlockAndPublish();
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BlindWalkController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,


    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      // Read current position from state interface
      double current_position = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_POS_ITFS].get_value();
      
      // Hold at current position (safe for both sim and real)
      command_interfaces_[i * CMD_IFC_PER_JOINT].set_value(current_position);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type BlindWalkController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
  {
    // iox2 imu data
    auto imu_sample = iox_imu_subscriber_->receive();
    if (imu_sample.has_value()) {
      auto& imu_sample_value = imu_sample.value();
      if (imu_sample_value.has_value()) {
        const auto& imu = imu_sample_value->payload();
        // debug
        // RCLCPP_INFO(get_node()->get_logger(), "Received IMU data - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", imu.roll, imu.pitch, imu.yaw);
      } 
    }

    // fsm data
    auto fsm_sample = iox_fsm_subscriber_->receive();
    if (fsm_sample.has_value()) {
      auto& fsm_sample_value = fsm_sample.value();
      if (fsm_sample_value.has_value()) {
        const auto& fsm_data = fsm_sample_value->payload();
        // debug
        // RCLCPP_INFO(get_node()->get_logger(), "Received FSM data - Current State: %s", robot_states_enum::get_state_string(fsm_data.current_state).c_str());
        // RCLCPP_INFO(get_node()->get_logger(), "Received FSM data - Linear X: %.2f, Linear Y: %.2f, Angular Z: %.2f", fsm_data.linear_x, fsm_data.linear_y, fsm_data.angular_z);
      }
    }
    

    // get joint states data
    std::vector<double> joint_positions(state_joints_.size());
    std::vector<double> joint_velocities(state_joints_.size());
    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      joint_positions[i] = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_POS_ITFS].get_value();
      joint_velocities[i] = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_VEL_ITFS].get_value();
      
      // debug
      // RCLCPP_INFO(get_node()->get_logger(), "Joint %s - Position: %.2f, Velocity: %.2f", state_joints_[i].c_str(), joint_positions[i], joint_velocities[i]);
    }

    // debugging
    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      command_interfaces_[i * CMD_IFC_PER_JOINT].set_value(5.0); // set position command to 5.0 for testing
  
    }
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop
    if (rt_policy_publisher_ && rt_policy_publisher_->trylock())
    {
      rt_policy_publisher_->msg_.header.stamp = time;
      for (size_t i = 0; i < state_joints_.size(); ++i)
      {
        size_t cmd_base_idx = i*CMD_IFC_PER_JOINT;
        rt_policy_publisher_->msg_.position[i] = command_interfaces_[cmd_base_idx + CMD_POS_ITFS].get_value(); // position command
      }
      rt_policy_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }





  void BlindWalkController::resize_msgs_based_on_joints() {
    size_t num_joints = params_.joints.size();
    policy_commands_msg_.joint_name.resize(num_joints);
    policy_commands_msg_.position.resize(num_joints);

    // fill the values in the messages with default values
    for (size_t i = 0; i < num_joints; ++i)
    {
      policy_commands_msg_.joint_name[i] = params_.joints[i];
      policy_commands_msg_.position[i] = 0.0;
    }
}




  controller_interface::CallbackReturn BlindWalkController::init_iox2_services() {
      // create iox2 node and connect to the services
      iox_node_ = NodeBuilder().create<ServiceType::Ipc>().value();

      // imu iox2 service
      auto imu_service_name = ServiceName::create(IOX_IMU_TOPIC);
      auto imu_service_result = iox_node_->service_builder(imu_service_name.value()).publish_subscribe<IoxImuData>().open_or_create();
      if (!imu_service_result.has_value()) {
        auto error = imu_service_result.error();
        std::cerr << "Failed! Error: " 
                  << iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error) 
                  << " (code: " << static_cast<int>(error) << ")" << std::endl;
        return controller_interface::CallbackReturn::FAILURE;
      }

      auto &iox_imu_service_ = imu_service_result.value();
      iox_imu_subscriber_ = iox_imu_service_.subscriber_builder().create().value();
      std::cout << "✅ iox2 Subscriber for imu connected successfully!... Listening for IMU data" << std::endl;

      // fsm iox2 service
      auto fsm_service_name = ServiceName::create(IOX_FSM_TOPIC);
      auto fsm_service_result = iox_node_->service_builder(fsm_service_name.value()).publish_subscribe<IoxFsmData>().open_or_create();
      if (!fsm_service_result.has_value()) {
        auto error = fsm_service_result.error();
        std::cerr << "Failed! Error: " 
                  << iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error) 
                  << " (code: " << static_cast<int>(error) << ")" << std::endl;
        return controller_interface::CallbackReturn::FAILURE;
      }

      auto &iox_fsm_service_ = fsm_service_result.value();
      iox_fsm_subscriber_ = iox_fsm_service_.subscriber_builder().create().value();
      std::cout << "✅ iox2 Subscriber for FSM connected successfully!... Listening for FSM data" << std::endl;
      return controller_interface::CallbackReturn::SUCCESS;
  } 

}  // namespace biped_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  biped_control::BlindWalkController, 
  controller_interface::ControllerInterface
)



// TODO:
// - the robot_current_state is not in string.. need to add the robot states from fsm
// - think about whether we can use ptr messages to write from nonRT
//- add the logic of sequence counter checking before updating the data in this method: read_data_from_shm_and_write_to_RT()
// - order the current robot states 
