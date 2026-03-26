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

using BipedShmMsg = biped_msgs::msg::BipedShmData;
using BipedJointCommand = biped_msgs::msg::BipedJointCommand;
using ControllerCommandPublisher = realtime_tools::RealtimePublisher<BipedJointCommand>;


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
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
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

  // Store the gains
  default_kp_ = params_.kp;  // read from yaml
  default_kd_ = params_.kd;  // read from yaml

  
  RCLCPP_INFO(get_node()->get_logger(), 
      "Default gains: Kp=%.2f, Kd=%.2f", default_kp_, default_kd_);

  latest_shm_data_msg_ = std::make_unique<BipedShmMsg>();
  policy_commands_msg_ = BipedJointCommand();
  
  // connect to shared memory
  if (!connect_to_shm(params_.shm_name))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to connect to shared memory: %s", params_.shm_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // resize the messages based on the number of joints
  resize_msgs_based_on_joints();
  init_shm_buffer_with_initial_data();    // for safety, initialize the buffer with default data before starting to read from shared memory
  
  try
  {
    // State publisher
    ros_policy_publisher_ =
      get_node()->create_publisher<BipedJointCommand>("~/policy_joint_commands", rclcpp::SystemDefaultsQoS());
    rt_policy_publisher_ = std::make_unique<ControllerCommandPublisher>(ros_policy_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration BlindWalkController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size() * 5);  // position, velocity, effort, Kp, Kd
  // Request 5 command interfaces per joint: position, velocity, effort, Kp, Kd
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/position");
    command_interfaces_config.names.push_back(joint + "/velocity");
    command_interfaces_config.names.push_back(joint + "/effort");
    command_interfaces_config.names.push_back(joint + "/kp");
    command_interfaces_config.names.push_back(joint + "/kd");
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
  // get the intial read fom the shm
  if (shm_ptr_) {
    read_data_from_shm_and_write_to_RT(shm_ptr_);
    RCLCPP_INFO(get_node()->get_logger(), "Current robot state from shared memory: %s", robot_state_.c_str());
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "failed to read the initial robot state from the shared memory");
  }

  // set the initial commands to prevent kickbacks
  // loop through all the joints
  for (size_t i = 0; i <  state_joints_.size(); ++i) {
    size_t cmd_base_idx = i*5; // 5 command interfaces per joint: position, velocity, effort, Kp, Kd
    size_t state_base_idx = i*2; // 2 state interfaces per joint: position, velocity

    // read current position from state interface
    double current_position = state_interfaces_[state_base_idx + STATE_POS_ITFS].get_value();

    // set position command to current position to prevent jumps
    command_interfaces_[cmd_base_idx + CMD_POS_ITFS].set_value(current_position);
    command_interfaces_[cmd_base_idx + CMD_VEL_ITFS].set_value(0.0); // set velocity command to zero
    command_interfaces_[cmd_base_idx + CMD_EFF_ITFS].set_value(0.0); // set effort command to zero
    command_interfaces_[cmd_base_idx + CMD_KP_ITFS].set_value(default_kp_); // set Kp to default value
    command_interfaces_[cmd_base_idx + CMD_KD_ITFS].set_value(default_kd_); // set Kd to default value

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

  // start the thread to read from shared memory and write to real-time buffer
  if (shm_ptr_) {
    shm_thread_ = std::thread(&BlindWalkController::read_data_from_shm_and_write_to_RT, this, shm_ptr_);
    shm_thread_.detach();
    thread_running_ = true;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BlindWalkController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,

  // stop the thread
  thread_running_ = false;
  if (shm_thread_.joinable()) {
    shm_thread_.join();
    RCLCPP_INFO(get_node()->get_logger(), "Shared memory reader thread stopped successfully.");
    shm_ptr_ = nullptr; // reset the shared memory pointer
  }

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BlindWalkController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto shm_data = shm_buffer_.readFromRT();

  std::vector<double> joint_positions(state_joints_.size());
  std::vector<double> joint_velocities(state_joints_.size());

  // read joints_data from state interfaces 
  for (size_t i = 0; i < state_joints_.size(); ++i) {
    size_t state_base_idx = i*2; // 2 state interfaces per joint: position and velocity
    joint_positions[i] = state_interfaces_[state_base_idx + STATE_POS_ITFS].get_value();
    joint_velocities[i] = state_interfaces_[state_base_idx + STATE_VEL_ITFS].get_value();
  }


  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  if (rt_policy_publisher_ && rt_policy_publisher_->trylock())
  {
    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      rt_policy_publisher_->msg_.header.stamp = time;
      rt_policy_publisher_->msg_.position[i] = command_interfaces_[i*5 + CMD_POS_ITFS].get_value(); // position command
      rt_policy_publisher_->msg_.velocity[i] = command_interfaces_[i*5 + CMD_VEL_ITFS].get_value(); // velocity command
      rt_policy_publisher_->msg_.effort[i] = command_interfaces_[i*5 + CMD_EFF_ITFS].get_value(); // effort command
      rt_policy_publisher_->msg_.kp[i] = command_interfaces_[i*5 + CMD_KP_ITFS].get_value(); // Kp command
      rt_policy_publisher_->msg_.kd[i] = command_interfaces_[i*5 + CMD_KD_ITFS].get_value(); // Kd command  
      rt_policy_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}


bool  BlindWalkController::connect_to_shm(const std::string & shm_name ) {
    try {
        bip::shared_memory_object shm_obj(bip::open_only, shm_name.c_str(), bip::read_only);
        bip::mapped_region region(shm_obj, bip::read_only);
        shm_ptr_ = static_cast<const BipedSharedMemory*>(region.get_address());
        return true;
    } catch (const bip::interprocess_exception &ex) {
        std::cerr << "Error connecting to shared memory: " << ex.what() << std::endl;
        return false;
    }
  }

// this function reads data from shared memory and writes it to the real-time buffer for use in the control loop
void  BlindWalkController::read_data_from_shm_and_write_to_RT(const BipedSharedMemory* shm_ptr) {
    rclcpp::Rate rate(1000); // read at 1kHz, adjust as needed

    // read from shm
    while (thread_running_) {
      latest_shm_data_msg_->header.stamp = get_node()->now();
      latest_shm_data_msg_->robot_state = robot_states_enum::get_state_string(shm_ptr->robot_state.current_state);
      latest_shm_data_msg_->joy_cmd.linear.x = shm_ptr->cmd_vel.linear_x;
      latest_shm_data_msg_->joy_cmd.linear.y = shm_ptr->cmd_vel.linear_y;
      latest_shm_data_msg_->joy_cmd.angular.z = shm_ptr->cmd_vel.angular_z;
      latest_shm_data_msg_->imu_orientation[0] = shm_ptr->imu.orientation[0];
      latest_shm_data_msg_->imu_orientation[1] = shm_ptr->imu.orientation[1];
      latest_shm_data_msg_->imu_orientation[2] = shm_ptr->imu.orientation[2];

      // update the robot state
      robot_state_ = latest_shm_data_msg_->robot_state;

      // write to RT
      shm_buffer_.writeFromNonRT(*latest_shm_data_msg_);

      rate.sleep();
    }

}


void BlindWalkController::init_shm_buffer_with_initial_data() {
  BipedShmMsg init_shm_data = BipedShmMsg();
  init_shm_data.header.stamp = get_node()->now();
  init_shm_data.robot_state = robot_states_enum::get_state_string(static_cast<uint8_t>(robot_states_enum::RobotState::INIT)); // set to INIT state

  // init the buffer with the initial data
  shm_buffer_.initRT(init_shm_data);
}



void BlindWalkController::resize_msgs_based_on_joints() {
  size_t num_joints = params_.joints.size();
  policy_commands_msg_.joint_name.resize(num_joints);
  policy_commands_msg_.position.resize(num_joints);
  policy_commands_msg_.velocity.resize(num_joints);
  policy_commands_msg_.effort.resize(num_joints);
  policy_commands_msg_.kp.resize(num_joints);
  policy_commands_msg_.kd.resize(num_joints);

  // fill the values in the messages with default values
  for (size_t i = 0; i < num_joints; ++i)
  {
    policy_commands_msg_.joint_name[i] = params_.joints[i];
    policy_commands_msg_.position[i] = 0.0;
    policy_commands_msg_.velocity[i] = 0.0;
    policy_commands_msg_.effort[i] = 0.0;
    policy_commands_msg_.kp[i] = default_kp_;
    policy_commands_msg_.kd[i] = default_kd_;
  }
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
