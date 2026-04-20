// Copyright (c) 2026, Bachala Rajesh
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

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN


#include <limits>
#include <vector>

#include "biped_hardware_interface/biped_sanpo_hardware_interface_single_threaded.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <cmath>

namespace biped_hardware_interface
{
hardware_interface::CallbackReturn BipedSanpoHardwareInterfaceSingleThreaded::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware

  N_LEFT_MOTORS = left_leg_motor_ids_.size();
  N_RIGHT_MOTORS = right_leg_motor_ids_.size();

  try {
    left_leg_port_name_= info_.hardware_parameters["left_leg_port"];
    right_leg_port_name_= info_.hardware_parameters["right_leg_port"];
    baudrate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  } catch(const std::exception & e) {
    RCLCPP_FATAL(
        rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"),
        "Failed to read hardware parameters: %s",
        e.what());
    return CallbackReturn::ERROR;
  }
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // prefill the MotorCommand vectors and the MotorState
  for (auto motor_id : left_leg_motor_ids_) {
    left_leg_motor_commands_.push_back(MotorCommand(motor_id, 0.0f, 0.0f, KP, KD, 0.0f));
    left_leg_motor_states_[motor_id] = MotorState{motor_id, 0.0f, 0.0f, 0.0f, 0.0f}; // initialize with default state
  }
  for (auto motor_id : right_leg_motor_ids_) {
    right_leg_motor_commands_.push_back(MotorCommand(motor_id, 0.0f, 0.0f, KP, KD, 0.0f));
    right_leg_motor_states_[motor_id] = MotorState{motor_id, 0.0f, 0.0f, 0.0f, 0.0f}; // initialize with default state
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BipedSanpoHardwareInterfaceSingleThreaded::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  // create serial communication objects
  left_leg_comms_ = std::make_unique<SanpoInterface>(
                                                        left_leg_port_name_, 
                                                        baudrate_, 
                                                        left_leg_can_channel_, 
                                                        left_leg_motor_ids_,
                                                        left_leg_motor_states_
                                                      );

  right_leg_comms_ = std::make_unique<SanpoInterface>(
                                                        right_leg_port_name_,
                                                        baudrate_,
                                                        right_leg_can_channel_,
                                                        right_leg_motor_ids_,
                                                        right_leg_motor_states_
                                                      );
  
  //open the serial port and perform the handshake
  RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "Opening sanpo serial ports and performing handshake");

  if (!left_leg_comms_->connect_sanpo() || !right_leg_comms_->connect_sanpo()) {
    RCLCPP_ERROR(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "Failed to connect to SANPO ports...");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "Connected to SANPO ports successfully for both legs.");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BipedSanpoHardwareInterfaceSingleThreaded::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // export position states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

    // export velocity states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BipedSanpoHardwareInterfaceSingleThreaded::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // TODO(anyone): insert correct interfaces
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn BipedSanpoHardwareInterfaceSingleThreaded::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "Activating motors...");

  // Attempt to enable both leg motors
  const uint8_t max_attempts = 3;
  std::vector<uint8_t> left_failed, right_failed;
  for (uint8_t attempt = 1; attempt <= max_attempts; ++attempt) {
    left_failed = left_leg_comms_->enable_motors();
    right_failed = right_leg_comms_->enable_motors(); 

    // If no motors failed, we can break early
    if (left_failed.empty() && right_failed.empty()) {
          break; 
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // brief pause between attempts

  } 

  // Check if any motors failed
  if (!left_failed.empty() || !right_failed.empty()) {
    std::stringstream ss;
    
    if (!left_failed.empty()) {
      ss << "Left [ ";
      for (auto id : left_failed) { ss << static_cast<int>(id) << " "; }
      ss << "] ";
    }
    
    if (!right_failed.empty()) {
      ss << "Right [ ";
      for (auto id : right_failed) { ss << static_cast<int>(id) << " "; }
      ss << "]";
    }

    // Safety Rollback: Disable all motors if any motor failed to enable
    RCLCPP_WARN(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "Rolling back: Disabling all motors as some motors failed to enable...");
    left_leg_comms_->disable_motors();
    right_leg_comms_->disable_motors();

    // Print the combined error
    RCLCPP_ERROR(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), 
      "Activation failed. Motors offline: %s", ss.str().c_str());

    return CallbackReturn::ERROR;
  }

  // update states of the motor
  for (int i = 0; i < 3; ++i) {
    read_motor_states();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // init the hw_commands_position_ for the motors to avoid a sudden jump when the robot is activated
  for (size_t i = 0; i < info_.joints.size(); ++i) {
      hw_commands_position_[i] = hw_states_position_[i];

      RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), 
                                    "%s: Initial command position set to current state %.3f radians, %.2f degrees",
                                    info_.joints[i].name.c_str(), hw_commands_position_[i], hw_commands_position_[i] * 180.0 / M_PI);
  }


  RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "All motors enabled successfully. Biped is ACTIVE.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BipedSanpoHardwareInterfaceSingleThreaded::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "Deactivating the hardware interface and disabling motors...");

  left_leg_comms_->disable_motors();
  right_leg_comms_->disable_motors();

  RCLCPP_INFO(rclcpp::get_logger("BipedSanpoHardwareInterfaceSingleThreaded"), "All motors disabled successfully. Biped is INACTIVE.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BipedSanpoHardwareInterfaceSingleThreaded::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states
  // Fetch the latest CAN frames from the hardware buffers
  read_motor_states();

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type BipedSanpoHardwareInterfaceSingleThreaded::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'

  // Transfer ROS 2 commands to Left Leg MotorCommand structs
  for (size_t i = 0; i < N_LEFT_MOTORS; ++i) {
    left_leg_motor_commands_[i].position = hw_commands_position_[i];
  }

  // Transfer ROS 2 commands to Right Leg MotorCommand structs
  for (size_t i = 0; i < N_RIGHT_MOTORS; ++i) {
    size_t ros_index = i + N_LEFT_MOTORS;       // Offset by number of left leg motors for the right leg
    right_leg_motor_commands_[i].position = hw_commands_position_[ros_index];
  }

  // Send the CAN frames to the hardware
  left_leg_comms_->write_motor_commands(left_leg_motor_commands_);
  right_leg_comms_->write_motor_commands(right_leg_motor_commands_);

  return hardware_interface::return_type::OK;
}


void BipedSanpoHardwareInterfaceSingleThreaded::read_motor_states(){
  left_leg_comms_->update_motor_states();
  right_leg_comms_->update_motor_states();

  // Transfer Left Leg data to ROS 2 
  for (size_t i = 0; i < N_LEFT_MOTORS; ++i) {
    uint8_t id = left_leg_motor_ids_[i];
    hw_states_position_[i] = left_leg_motor_states_[id].filtered_position;
    hw_states_velocity_[i] = left_leg_motor_states_[id].velocity;
  }

  // Transfer Right Leg data to ROS 2 
  for (size_t i = 0; i < N_RIGHT_MOTORS; ++i) {
    uint8_t id = right_leg_motor_ids_[i];
    size_t ros_index = i + N_LEFT_MOTORS; // Offset by number of left leg motors for the right leg
    hw_states_position_[ros_index] = right_leg_motor_states_[id].filtered_position;
    hw_states_velocity_[ros_index] = right_leg_motor_states_[id].velocity;
  }
}

}  // namespace biped_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  biped_hardware_interface::BipedSanpoHardwareInterfaceSingleThreaded, hardware_interface::SystemInterface)
