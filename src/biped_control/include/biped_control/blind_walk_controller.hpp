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

#ifndef BIPED_CONTROL__BLIND_WALK_CONTROLLER_HPP_
#define BIPED_CONTROL__BLIND_WALK_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <optional>
#include "iox2/iceoryx2.hpp"
#include "iox2_msgs/iox_imu_msgs.hpp"
#include "iox2_msgs/iox_fsm_states_msg.hpp"


#include "controller_interface/controller_interface.hpp"
#include <biped_control/blind_walk_controller_parameters.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "biped_msgs/msg/biped_joint_command.hpp"
#include "biped_fsm/robot_states_enum.hpp"

// TODO(anyone): Replace with controller specific messages

using namespace iox2;

namespace biped_control
{
  // name constants for state interfaces
  static constexpr size_t STATE_POS_ITFS = 0;   //position state index
  static constexpr size_t STATE_VEL_ITFS = 1;   //velocity state index

  // name constants for command interfaces
  static constexpr size_t CMD_POS_ITFS = 0;     //position command

  static constexpr size_t CMD_IFC_PER_JOINT = 1;
  static constexpr size_t STATE_IFC_PER_JOINT = 2;



  class BlindWalkController : public controller_interface::ControllerInterface
  {
  public:
    BlindWalkController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    
  private:
    void resize_msgs_based_on_joints();
    controller_interface::CallbackReturn init_iox2_services();

    
    // variables
    std::shared_ptr<blind_walk_controller::ParamListener> param_listener_;
    blind_walk_controller::Params params_;
    
    // ros2 publisher and real time publisher
    using BipedJointCommand = biped_msgs::msg::BipedJointCommand;
    using ControllerCommandPublisher = realtime_tools::RealtimePublisher<BipedJointCommand>;
    rclcpp::Publisher<BipedJointCommand>::SharedPtr ros_policy_publisher_;
    std::unique_ptr<ControllerCommandPublisher> rt_policy_publisher_;
    BipedJointCommand policy_commands_msg_;
    
    
    // internal state variables
    std::vector<double> initial_joint_positions_;
    std::vector<std::string> state_joints_;
    robot_states_enum::RobotState robot_state_ = robot_states_enum::RobotState::INIT;
    

    // iox2 related variables
    std::optional<iox2::Node<iox2::ServiceType::Ipc>> iox_node_;
    std::optional<iox2::Subscriber<iox2::ServiceType::Ipc, IoxImuData, void>> iox_imu_subscriber_;
    std::optional<iox2::Subscriber<iox2::ServiceType::Ipc, IoxFsmData, void>> iox_fsm_subscriber_;

    

    
  
  };

}  // namespace biped_control

#endif  // BIPED_CONTROL__BLIND_WALK_CONTROLLER_HPP_
