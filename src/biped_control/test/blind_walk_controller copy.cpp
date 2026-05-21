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

const std::string pkg_share_biped_control =  ament_index_cpp::get_package_share_directory("biped_control");
const std::string policy_path = pkg_share_biped_control + "/models/exported/policy.pt";

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
    
    // configuration of policy related variables
    obs_builder_ = std::make_unique<locomotion::ObservationBuilder>(cfg_);
    policy_ = std::make_unique<locomotion::Policy>(policy_path);


    // resize the messages based on the number of joints
    resize_ros_biped_msgs_based_on_joints();
    
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


    // joint state order
    RCLCPP_INFO(get_node()->get_logger(), "=== State Interface Order ===");
    for (size_t i = 0; i < state_interfaces_.size(); i++) {
        RCLCPP_INFO(get_node()->get_logger(), "[%zu] %s", i, 
                    state_interfaces_[i].get_name().c_str());
    }

    // Add in on_activate() after the state interface print
    RCLCPP_INFO(get_node()->get_logger(), "=== Command Interface Order ===");
    for (size_t i = 0; i < command_interfaces_.size(); i++) {
        RCLCPP_INFO(get_node()->get_logger(), "[%zu] %s", i,
                    command_interfaces_[i].get_name().c_str());
    }
    // form the initial policy observation
    form_initial_policy_observation();

    // check for all the sensors to be available before activating
    // if (!imu_available_) {
    //   RCLCPP_ERROR(get_node()->get_logger(), "Failed to get imu data");
    //   return controller_interface::CallbackReturn::FAILURE;
    // }
    // if (!fsm_available_) {
    //   RCLCPP_ERROR(get_node()->get_logger(), "Failed to get fsm data");
    //   return controller_interface::CallbackReturn::FAILURE;
    // }
    // if (!joints_data_available_) {
    //   RCLCPP_ERROR(get_node()->get_logger(), "Failed to get joint states data");
    //   return controller_interface::CallbackReturn::FAILURE;
    // }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BlindWalkController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {


    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      // Read current position from state interface
      double current_position = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_POS_ITFS].get_value();
      
      // Hold at current position (safe for both sim and real)
      command_interfaces_[i * CMD_IFC_PER_JOINT].set_value(current_position);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  void BlindWalkController::resize_ros_biped_msgs_based_on_joints() {
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


  void BlindWalkController::form_initial_policy_observation() {
    size_t num_joints = params_.joints.size();
    last_actions_.resize(num_joints, 0.0);
    scaled_last_actions_.resize(num_joints, 0.0);

    // form robotsensor data
    get_iox_imu_data(robot_sensor_data_, imu_available_);
    get_iox_fsm_data(cmd_vel_, fsm_state_, fsm_available_);
    get_joint_states_data(robot_sensor_data_, joints_data_available_);

    // update the initial observations
    obs_builder_->scale_update_observation_history(robot_sensor_data_, gait_time_, cmd_vel_, last_actions_);
    obs_builder_->init_history();
  }
  

  void BlindWalkController::get_iox_imu_data(locomotion::RobotSensorData& robot_sensor_data, bool& imu_available) {
    auto imu_sample = iox_imu_subscriber_->receive();
    if (imu_sample.has_value()) {
      auto& imu_sample_value = imu_sample.value();
      if (imu_sample_value.has_value()) {
        const auto& imu_data = imu_sample_value->payload();
        robot_sensor_data.imu_quat.x = imu_data.orientation_x;
        robot_sensor_data.imu_quat.y = imu_data.orientation_y;
        robot_sensor_data.imu_quat.z = imu_data.orientation_z;
        robot_sensor_data.imu_quat.w = imu_data.orientation_w;
        //Add indentation for imu_gyro and imu_accel arrays
        robot_sensor_data.imu_gyro = std::array<double, 3>{
                                          imu_data.ang_vel_x, 
                                          imu_data.ang_vel_y, 
                                          imu_data.ang_vel_z
                                        };
        
        robot_sensor_data.imu_accel = std::array<double, 3>{
                                          imu_data.lin_acc_x, 
                                          imu_data.lin_acc_y, 
                                          imu_data.lin_acc_z
                                        };

        imu_available = true;
        // RCLCPP_INFO(get_node()->get_logger(), "Received IMU data - orientation_x: %.2f, orientation_y: %.2f, orientation_z: %.2f, orientation_w: %.2f", imu.orientation_x, imu.orientation_y, imu.orientation_z, imu.orientation_w);
      } 
    }
  }

  void BlindWalkController::get_iox_fsm_data(
              std::array<double, 3>& cmd_vel, 
              robot_states_enum::RobotState& fsm_state,
              bool& fsm_available) {
    auto fsm_sample = iox_fsm_subscriber_->receive();
    if (fsm_sample.has_value()) {
      auto& fsm_sample_value = fsm_sample.value();
      if (fsm_sample_value.has_value()) {
        const auto& fsm_data = fsm_sample_value->payload();
        fsm_state = fsm_data.current_state;
        cmd_vel = std::array<double, 3>{
                                          fsm_data.linear_x, 
                                          fsm_data.linear_y, 
                                          fsm_data.angular_z
                                        };
        fsm_available = true;
        // RCLCPP_INFO(get_node()->get_logger(), "Received FSM data - Current State: %s", robot_states_enum::get_state_string(fsm_data.current_state).c_str());
        // RCLCPP_INFO(get_node()->get_logger(), "Received FSM data - Linear X: %.2f, Linear Y: %.2f, Angular Z: %.2f", fsm_data.linear_x, fsm_data.linear_y, fsm_data.angular_z);
      }
    }
  }

  // void BlindWalkController::get_joint_states_data(locomotion::RobotSensorData& robot_sensor_data, bool& joints_data_available) {
  //   for (size_t i = 0; i < state_joints_.size(); ++i)
  //   {
  //     auto joint_position = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_POS_ITFS].get_value();
  //     auto joint_vel = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_VEL_ITFS].get_value();
  //     robot_sensor_data.joint_pos.push_back(joint_position);
  //     robot_sensor_data.joint_vel.push_back(joint_vel);
  //     joints_data_available = true;
  //     // RCLCPP_INFO(get_node()->get_logger(), "Joint %s - Position: %.2f, Velocity: %.2f", state_joints_[i].c_str(), joint_positions[i], joint_velocities[i]);
  //   }
  // }


void BlindWalkController::get_joint_states_data(
    locomotion::RobotSensorData& robot_sensor_data, bool& joints_data_available)
{
  const size_t n = state_joints_.size();
  robot_sensor_data.joint_pos.resize(n);
  robot_sensor_data.joint_vel.resize(n);

  for (size_t i = 0; i < n; ++i) {
    robot_sensor_data.joint_pos[i] =
        state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_POS_ITFS].get_value();
    robot_sensor_data.joint_vel[i] =
        state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_VEL_ITFS].get_value();
  }
  joints_data_available = true;
}


controller_interface::return_type BlindWalkController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // ── 1. READ SENSORS at 200 Hz ─────────────────────────────────────────
  robot_sensor_data_.joint_pos.clear();
  robot_sensor_data_.joint_vel.clear();

  get_iox_imu_data(robot_sensor_data_, imu_available_);
  get_iox_fsm_data(cmd_vel_, fsm_state_, fsm_available_);
  get_joint_states_data(robot_sensor_data_, joints_data_available_);

  // ── 2. POLICY STEP at 50 Hz (every decimation ticks) ──────────────────
  if (counter_ % cfg_.decimation == 0)
  {
    // Only run policy when all sensors are live and FSM is ACTIVE
    if (imu_available_ && fsm_available_ && joints_data_available_
        && fsm_state_ == robot_states_enum::RobotState::ACTIVE)
    {
      // 2a. Build observation (uses current sensor data + last_actions_)
      obs_builder_->scale_update_observation_history(
          robot_sensor_data_, gait_time_, cmd_vel_, last_actions_);

      // 2b. Run policy
      auto stacked_obs  = obs_builder_->stacked_observation();
      auto raw_actions  = policy_->forward(stacked_obs);  // policy order, raw [-1,1]

      // 2c. Clamp and store raw actions — fed back as last_actions_ next step
      //     Store BEFORE scaling — policy was trained on unscaled last_action obs
      for (size_t i = 0; i < raw_actions.size(); i++) {
        last_actions_[i] = std::clamp((double)raw_actions[i], -100.0, 100.0);
      }

      // 2d. Debug print on first policy step
      // Add this in update() right after stacked_obs is built, on first step
      if (counter_ == 0) {
          RCLCPP_INFO(get_node()->get_logger(), "=== OBSERVATION BREAKDOWN ===");
          // With obs_history_len=5, each term repeats 5 times
          // Term sizes: ang_vel=3, proj_grav=3, cmd_vel=3, jpos=6, jvel=6, actions=6, phase=2, gcmd=3
          // Total per timestep = 32, total = 160
          
          int idx = 0;
          auto print_term = [&](const char* name, int size) {
              std::string vals = "";
              for (int t = 0; t < cfg_.obs_history_len; t++) {
                  vals += "t" + std::to_string(t) + ":[";
                  for (int j = 0; j < size; j++) {
                      vals += std::to_string(stacked_obs[idx++]);
                      if (j < size-1) vals += ",";
                  }
                  vals += "] ";
              }
              RCLCPP_INFO(get_node()->get_logger(), "  %s: %s", name, vals.c_str());
          };

          print_term("ang_vel    (3)", 3);
          print_term("proj_grav  (3)", 3);
          print_term("cmd_vel    (3)", 3);
          print_term("joint_pos  (6)", 6);
          print_term("joint_vel  (6)", 6);
          print_term("last_act   (6)", 6);
          print_term("gait_phase (2)", 2);
          print_term("gait_cmd   (3)", 3);
      }

      // 2e. Advance gait time AFTER obs is built (at policy rate)
      gait_time_ += cfg_.sim_dt * cfg_.decimation;  // = 0.005 * 4 = 0.02s per step
    }
  }
  counter_++;

  // ── 3. WRITE COMMANDS at 200 Hz (zero-order hold between policy steps) ─
  //
  // joint ordering is confirmed identical:
  //   hw order    = [lhp, rhp, lhr, rhr, lk, rk]  (from debug output)
  //   policy order = [lhp, rhp, lhr, rhr, lk, rk]  (policy_trained_config.hpp)
  //   cmd order   = [lhp, rhp, lhr, rhr, lk, rk]  (from debug output)
  // So index i maps directly: policy[i] → cmd[i], no remapping needed.
  //
  // initial_joint_pose is in policy order → directly use index i
  //
  for (size_t i = 0; i < cfg_.control_order_joints.size(); ++i)
  {
    double target_pos = cfg_.action_scale * last_actions_[i]
                        + cfg_.initial_joint_pose[i].second;

    command_interfaces_[i * CMD_IFC_PER_JOINT].set_value(target_pos);

    // also update the ROS message for debugging
    policy_commands_msg_.position[i] = target_pos;
  }

  // ── 4. PUBLISH debug message (non-critical, best effort) ──────────────
  if (rt_policy_publisher_ && rt_policy_publisher_->trylock()) {
    rt_policy_publisher_->msg_ = policy_commands_msg_;
    rt_policy_publisher_->msg_.header.stamp = get_node()->now();
    rt_policy_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
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
