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

#pragma once
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN


#ifndef BIPED_HARDWARE_INTERFACE__BIPED_SANPO_HARDWARE_INTERFACE_SINGLE_THREADED_HPP_
#define BIPED_HARDWARE_INTERFACE__BIPED_SANPO_HARDWARE_INTERFACE_SINGLE_THREADED_HPP_

#include <memory>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <array>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include "motor_structs.h"
#include "sanpo_interface.h"

namespace biped_hardware_interface
{
class BipedSanpoHardwareInterfaceSingleThreaded : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void read_motor_states();

  std::string left_leg_port_name_, right_leg_port_name_;
  uint32_t baudrate_;

  // CAN channels
  uint8_t left_leg_can_channel_ = 1;
  uint8_t right_leg_can_channel_ = 3;

  // Motor CAN ids
  std::vector<uint8_t> left_leg_motor_ids_ {1,2,3};
  std::vector<uint8_t> right_leg_motor_ids_ {4,5,6};

  // Motor states
  std::unordered_map<uint8_t, MotorState> left_leg_motor_states_;
  std::unordered_map<uint8_t, MotorState> right_leg_motor_states_;

  //Motor commands
  std::vector<MotorCommand> left_leg_motor_commands_;
  std::vector<MotorCommand> right_leg_motor_commands_;

  // serial communication objects
  std::unique_ptr<SanpoInterface> left_leg_comms_;
  std::unique_ptr<SanpoInterface> right_leg_comms_;

  // interfaces position
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_commands_kp_;
  std::vector<double> hw_commands_kd_;

  float KP= KP_DEAFULT; // position stiffness inital value, can be tuned later based on the hardware response
  float KD = KD_DEAFULT; // velocity damping initial value, can be tuned later based on the hardware response

  uint8_t N_LEFT_MOTORS = 0;    // number of motors in the left leg, initialized to 0 and will be set based on the size of left_leg_motor_ids_
  uint8_t N_RIGHT_MOTORS = 0;   // number of motors in the right leg, initialized to 0 and will be set based on the size of right_leg_motor_ids_
};



}  // namespace biped_hardware_interface

#endif  // BIPED_HARDWARE_INTERFACE__BIPED_SANPO_HARDWARE_INTERFACE_SINGLE_THREADED_HPP_
