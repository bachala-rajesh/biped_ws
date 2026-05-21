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

namespace {
  constexpr double kActionClamp = 100.0;
}

BlindWalkController::BlindWalkController()
: controller_interface::ControllerInterface(),
  robot_sensor_data_{}
{
}

controller_interface::CallbackReturn BlindWalkController::on_init()
{
  try {
    param_listener_ = std::make_shared<blind_walk_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BlindWalkController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (params_.joints.size() != cfg_.control_order_joints.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and policy control_order_joints (%zu) has to be the same!",
      params_.joints.size(), cfg_.control_order_joints.size());
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Validate joint name ordering matches the policy expectation
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    if (params_.joints[i] != cfg_.control_order_joints[i]) {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Joint name mismatch at index %zu: param='%s' policy='%s'",
        i, params_.joints[i].c_str(), cfg_.control_order_joints[i].c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  // Validate state joints match command joints in order 
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    if (state_joints_[i] != params_.joints[i]) {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "State joint order mismatch at index %zu: state_joint='%s' joint='%s'. "
        "State joints must match command joints in order for this controller.",
        i, state_joints_[i].c_str(), params_.joints[i].c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  // Resolve policy model path at configure time (not global init)
  std::string policy_path;
  try {
    policy_path = ament_index_cpp::get_package_share_directory("biped_control")
                  + "/models/exported/policy.pt";
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to locate policy model: %s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Initialize the iox2 services and subscribers
  auto result = init_iox2_services();
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize iox2 services");
    return result;
  }

  policy_commands_msg_ = BipedJointCommand();

  // Configuration of policy related variables
  obs_builder_ = std::make_unique<locomotion::ObservationBuilder>(cfg_);
  policy_ = std::make_unique<locomotion::Policy>(policy_path);

  // Resize the messages based on the number of joints
  resize_ros_biped_msgs_based_on_joints();

  try {
    ros_policy_publisher_ =
      get_node()->create_publisher<BipedJointCommand>("~/policy_joint_commands", rclcpp::SystemDefaultsQoS());
    rt_policy_publisher_ = std::make_unique<ControllerCommandPublisher>(ros_policy_publisher_);
  }
  catch (const std::exception & e) {
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

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints) {
    command_interfaces_config.names.push_back(joint + "/position");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration BlindWalkController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size() * 2);
  for (const auto & joint : state_joints_) {
    state_interfaces_config.names.push_back(joint + "/position");
    state_interfaces_config.names.push_back(joint + "/velocity");
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn BlindWalkController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const size_t num_joints = state_joints_.size();
  last_actions_.resize(num_joints, 0.0);

  // Reset internal state for a fresh activation
  gait_time_ = 0.0;
  counter_ = 0;

  // Hold at current position and derive last_actions_ ]
  for (size_t i = 0; i < num_joints; ++i) {
    const size_t cmd_base_idx = i * CMD_IFC_PER_JOINT;
    const size_t state_base_idx = i * STATE_IFC_PER_JOINT;

    const double current_position = state_interfaces_[state_base_idx + STATE_POS_ITFS].get_value();
    if (!std::isfinite(current_position)) {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Non-finite position on joint %s at activation",
        state_joints_[i].c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }

    // Hold hardware at current position
    command_interfaces_[cmd_base_idx].set_value(current_position);
    policy_commands_msg_.position[i] = current_position;

    // Derive the equivalent action that produces this position from the policy offset
    const double initial_pose = cfg_.initial_joint_pose[i].second;
    last_actions_[i] = std::clamp(
      (current_position - initial_pose) / cfg_.action_scale,
      -kActionClamp, kActionClamp);

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Activated joint %s at %.4f (derived action=%.4f)",
      state_joints_[i].c_str(), current_position, last_actions_[i]);
  }

  // Publish the initial hold commands
  if (rt_policy_publisher_ && rt_policy_publisher_->trylock()) {
    rt_policy_publisher_->msg_ = policy_commands_msg_;
    rt_policy_publisher_->msg_.header.stamp = get_node()->now();
    rt_policy_publisher_->unlockAndPublish();
  }

  // Debug: print interface ordering
  RCLCPP_INFO(get_node()->get_logger(), "=== State Interface Order ===");
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    RCLCPP_INFO(get_node()->get_logger(), "[%zu] %s", i, state_interfaces_[i].get_name().c_str());
  }
  RCLCPP_INFO(get_node()->get_logger(), "=== Command Interface Order ===");
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    RCLCPP_INFO(get_node()->get_logger(), "[%zu] %s", i, command_interfaces_[i].get_name().c_str());
  }

  // Zero-initialize sensor data before first read to avoid garbage / UB
  robot_sensor_data_ = locomotion::RobotSensorData{};
  form_initial_policy_observation();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BlindWalkController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < state_joints_.size(); ++i) {
    const double current_position = state_interfaces_[i * STATE_IFC_PER_JOINT + STATE_POS_ITFS].get_value();
    command_interfaces_[i * CMD_IFC_PER_JOINT].set_value(current_position);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void BlindWalkController::resize_ros_biped_msgs_based_on_joints()
{
  const size_t num_joints = params_.joints.size();
  policy_commands_msg_.joint_name.resize(num_joints);
  policy_commands_msg_.position.resize(num_joints);

  for (size_t i = 0; i < num_joints; ++i) {
    policy_commands_msg_.joint_name[i] = params_.joints[i];
    policy_commands_msg_.position[i] = 0.0;
  }
}

controller_interface::CallbackReturn BlindWalkController::init_iox2_services()
{
  iox_node_ = NodeBuilder().create<ServiceType::Ipc>().value();

  // IMU service
  auto imu_service_name = ServiceName::create(IOX_IMU_TOPIC);
  auto imu_service_result = iox_node_->service_builder(imu_service_name.value())
                              .publish_subscribe<IoxImuData>()
                              .open_or_create();
  if (!imu_service_result.has_value()) {
    auto error = imu_service_result.error();
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to create IMU service: %s (code %d)",
      iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error),
      static_cast<int>(error));
    return controller_interface::CallbackReturn::FAILURE;
  }

  auto & iox_imu_service_ = imu_service_result.value();
  iox_imu_subscriber_ = iox_imu_service_.subscriber_builder().create().value();
  RCLCPP_INFO(get_node()->get_logger(), "iox2 IMU subscriber connected");

  // FSM service
  auto fsm_service_name = ServiceName::create(IOX_FSM_TOPIC);
  auto fsm_service_result = iox_node_->service_builder(fsm_service_name.value())
                              .publish_subscribe<IoxFsmData>()
                              .open_or_create();
  if (!fsm_service_result.has_value()) {
    auto error = fsm_service_result.error();
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to create FSM service: %s (code %d)",
      iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error),
      static_cast<int>(error));
    return controller_interface::CallbackReturn::FAILURE;
  }

  auto & iox_fsm_service_ = fsm_service_result.value();
  iox_fsm_subscriber_ = iox_fsm_service_.subscriber_builder().create().value();
  RCLCPP_INFO(get_node()->get_logger(), "iox2 FSM subscriber connected");
  return controller_interface::CallbackReturn::SUCCESS;
}

void BlindWalkController::form_initial_policy_observation()
{
  // last_actions_ is already sized and initialized to hold-position values in on_activate()

  get_iox_imu_data(robot_sensor_data_, imu_available_);
  get_iox_fsm_data(cmd_vel_, fsm_state_, fsm_available_);
  get_joint_states_data(robot_sensor_data_, joints_data_available_);

  obs_builder_->scale_update_observation_history(robot_sensor_data_, gait_time_, cmd_vel_, last_actions_);
  obs_builder_->init_history();
}

void BlindWalkController::get_iox_imu_data(
  locomotion::RobotSensorData& robot_sensor_data, bool& imu_available)
{
  auto imu_sample = iox_imu_subscriber_->receive();
  if (imu_sample.has_value()) {
    auto& imu_sample_value = imu_sample.value();
    if (imu_sample_value.has_value()) {
      const auto& imu_data = imu_sample_value->payload();
      robot_sensor_data.imu_quat.x = imu_data.orientation_x;
      robot_sensor_data.imu_quat.y = imu_data.orientation_y;
      robot_sensor_data.imu_quat.z = imu_data.orientation_z;
      robot_sensor_data.imu_quat.w = imu_data.orientation_w;

      robot_sensor_data.imu_gyro = {
        imu_data.ang_vel_x,
        imu_data.ang_vel_y,
        imu_data.ang_vel_z};

      robot_sensor_data.imu_accel = {
        imu_data.lin_acc_x,
        imu_data.lin_acc_y,
        imu_data.lin_acc_z};

      imu_available = true;
    }
  }
}

void BlindWalkController::get_iox_fsm_data(
  std::array<double, 3>& cmd_vel,
  robot_states_enum::RobotState& fsm_state,
  bool& fsm_available)
{
  auto fsm_sample = iox_fsm_subscriber_->receive();
  if (fsm_sample.has_value()) {
    auto& fsm_sample_value = fsm_sample.value();
    if (fsm_sample_value.has_value()) {
      const auto& fsm_data = fsm_sample_value->payload();
      fsm_state = fsm_data.current_state;
      cmd_vel = {
        fsm_data.linear_x,
        fsm_data.linear_y,
        fsm_data.angular_z};
      fsm_available = true;
    }
  }
}

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t num_joints = cfg_.control_order_joints.size();

  // debug
  cmd_vel_ = {0.3, 0.0, 0.0};

  // 1. READ SENSORS at controller rate
  get_iox_imu_data(robot_sensor_data_, imu_available_);
  get_iox_fsm_data(cmd_vel_, fsm_state_, fsm_available_);
  get_joint_states_data(robot_sensor_data_, joints_data_available_);

  // 2. POLICY STEP at policy rate
  if (counter_ % cfg_.decimation == 0) {
    // Only run policy when all sensors are live
    if (imu_available_ && joints_data_available_)
    {
      // Advance gait time BEFORE building obs to match training (mujoco_node_torch does the same)
      gait_time_ += cfg_.sim_dt * cfg_.decimation;

      // build observation history — last_actions_ must hold RAW policy outputs (no scale, no offset)
      obs_builder_->scale_update_observation_history(
          robot_sensor_data_, gait_time_, cmd_vel_, last_actions_);

      // run policy
      auto stacked_obs = obs_builder_->stacked_observation();
      auto raw_actions = policy_->forward(stacked_obs);

      // store raw clamped actions for the next observation step
      for (size_t i = 0; i < num_joints; ++i) {
        double a = static_cast<double>(raw_actions[i]);
        if (!std::isfinite(a)) { a = 0.0; }
        last_actions_[i] = std::clamp(a, -kActionClamp, kActionClamp);
      }
    }
  }

  ++counter_;

  // Apply scale + initial offset when writing to hardware
  for (size_t i = 0; i < state_joints_.size(); ++i) {
    const double pos_target =
        last_actions_[i] * cfg_.action_scale + cfg_.initial_joint_pose[i].second;
    command_interfaces_[i * CMD_IFC_PER_JOINT].set_value(pos_target);
    policy_commands_msg_.position[i] = pos_target;
  }

  if (rt_policy_publisher_ && rt_policy_publisher_->trylock()) {
    rt_policy_publisher_->msg_ = policy_commands_msg_;    //not required i think
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
