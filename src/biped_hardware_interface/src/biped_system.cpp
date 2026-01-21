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

#include <limits>
#include <vector>

#include "biped_hardware_interface/biped_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace biped_hardware_interface
{
hardware_interface::CallbackReturn BipedSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try {
      left_leg_port_name_ = info_.hardware_parameters["left_leg_port"];
      right_leg_port_name_ = info_.hardware_parameters["right_leg_port"];
      baud_rate_ = std::stoul(info_.hardware_parameters["baud_rate"]);
  } catch(const std::out_of_range & e) {
      RCLCPP_FATAL(
          rclcpp::get_logger("BipedSystemHardware"),
          "Port names for legs not specified in hardware parameters");
      return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("BipedSystemHardware"),
      "Initializing Biped. Left leg port: %s, Right leg port: %s, Baud rate: %lu",
      left_leg_port_name_.c_str(),
      right_leg_port_name_.c_str(),
      baud_rate_);

  // Resize all the vectors to the number of joints
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BipedSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BipedSystemHardware"), "configuring hardware...");

  // try connecting left leg
  if (left_leg_comms_.connect(left_leg_port_name_, baud_rate_, 1)) {
    RCLCPP_INFO(
        rclcpp::get_logger("BipedSystemHardware"),
        "Successfully connected to left leg microcontroller on port %s",
        left_leg_port_name_.c_str());
  }
  else {
    RCLCPP_FATAL(
        rclcpp::get_logger("BipedSystemHardware"),
        "Failed to connect to left leg microcontroller on port %s",
        left_leg_port_name_.c_str());
    return CallbackReturn::ERROR;
  }

  // try connecting right leg
  if (right_leg_comms_.connect(right_leg_port_name_, baud_rate_, 1)) {
    RCLCPP_INFO(
        rclcpp::get_logger("BipedSystemHardware"),
        "Successfully connected to right leg microcontroller on port %s",
        right_leg_port_name_.c_str());
  }
  else {
    RCLCPP_FATAL(
        rclcpp::get_logger("BipedSystemHardware"),
        "Failed to connect to right leg microcontroller on port %s",
        right_leg_port_name_.c_str());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BipedSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // export position 
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    
    // export velocity
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BipedSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, 
      &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn BipedSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BipedSystemHardware"), "Activating hardware...");

  // initialize commands to current states
  for (size_t i =0; i < hw_states_position_.size(); ++i)
  {
    if (std::isnan(hw_states_position_[i])) {
      hw_commands_[i] = 0.0;
      hw_states_position_[i] = 0.0;
      hw_states_velocity_[i] = 0.0;
    }
    else {
      hw_commands_[i] = hw_states_position_[i];
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BipedSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BipedSystemHardware"), "Deactivating hardware...");
  left_leg_comms_.disconnect();
  right_leg_comms_.disconnect();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BipedSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> left_pos(3, 0.0), left_vel(3, 0.0);
  std::vector<double> right_pos(3, 0.0), right_vel(3, 0.0);

  // read left leg states
  if (left_leg_comms_.read_latest_joint_states(left_pos, left_vel, 'L')) {
    hw_states_position_[0] = left_pos[0];
    hw_states_position_[1] = left_pos[1];
    hw_states_position_[2] = left_pos[2];
    hw_states_velocity_[0] = left_vel[0];
    hw_states_velocity_[1] = left_vel[1];
    hw_states_velocity_[2] = left_vel[2];
  }
  
  // read right leg states
  if (right_leg_comms_.read_latest_joint_states(right_pos, right_vel, 'R')) {
    hw_states_position_[3] = right_pos[0];
    hw_states_position_[4] = right_pos[1];
    hw_states_position_[5] = right_pos[2];
    hw_states_velocity_[3] = right_vel[0];
    hw_states_velocity_[4] = right_vel[1];
    hw_states_velocity_[5] = right_vel[2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BipedSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // split the commands for left and right legs
  std::vector<double> left_leg_commands = {hw_commands_[0], hw_commands_[1], hw_commands_[2]};
  std::vector<double> right_leg_commands = {hw_commands_[3], hw_commands_[4], hw_commands_[5]};

  // send commands
  left_leg_comms_.send_motor_commands(left_leg_commands);
  right_leg_comms_.send_motor_commands(right_leg_commands);

  return hardware_interface::return_type::OK;
}

}  // namespace biped_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  biped_hardware_interface::BipedSystemHardware, hardware_interface::SystemInterface)
