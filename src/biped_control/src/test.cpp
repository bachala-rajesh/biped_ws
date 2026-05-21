
  // controller_interface::return_type BlindWalkController::update(
  //           const rclcpp::Time & time, const rclcpp::Duration & period)
  // {

  //   // get_data from imu, fsm and joint states
  //   get_iox_imu_data(robot_sensor_data_, imu_available_);
  //   get_iox_fsm_data(cmd_vel_, fsm_state_, fsm_available_);
  //   get_joint_states_data(robot_sensor_data_, joints_data_available_);
 
  //   // compute gait_time
  //   gait_time_ += period.seconds();

  //   // update the observations
  //   obs_builder_->scale_update_observation_history(robot_sensor_data_, gait_time_, cmd_vel_, last_actions_);
    
  //   if (counter_ % cfg_.decimation == 0) {
  //     if (imu_available_ && fsm_available_ && joints_data_available_ && fsm_state_ == robot_states_enum::RobotState::ACTIVE) {
  //       // get the stacked observations
  //       auto stacked_obs = obs_builder_->stacked_observation();

  //       // active state
  //       auto actions_f = policy_->forward(stacked_obs);
  //       last_actions_.assign(actions_f.begin(), actions_f.end());
        
  //       // TODO: clamp the actions to the range of the joint limits
  //       for (auto& a : last_actions_) a = std::clamp(a, -100.0, 100.0);

  //     }
  //     // TODO: add the logic for non active states
  //   }
  //   counter_++;
    
  //   // apply control
  //   for (size_t i = 0; i < state_joints_.size(); ++i)
  //   {
  //     auto control_to_joint_idx = cfg_.control_to_joint_states_mapping[i];
  //     auto init_joint_pose = cfg_.initial_joint_pose[control_to_joint_idx].second;
  //     scaled_last_actions_[i] = last_actions_[i] * cfg_.action_scale + init_joint_pose;

  //     size_t cmd_base_idx = i*CMD_IFC_PER_JOINT; 
  //     command_interfaces_[cmd_base_idx].set_value(scaled_last_actions_[i]);
  //   }
    


  //   // publish the actions data with rt_policy_publisher
  //   if (rt_policy_publisher_ && rt_policy_publisher_->trylock())
  //   {
  //     rt_policy_publisher_->msg_.header.stamp = time;
  //     for (size_t i = 0; i < state_joints_.size(); ++i)
  //     {
  //       size_t cmd_base_idx = i*CMD_IFC_PER_JOINT;
  //       rt_policy_publisher_->msg_.position[i] = command_interfaces_[cmd_base_idx].get_value(); // position command
  //     }
  //     rt_policy_publisher_->unlockAndPublish();
  //   }

  //   return controller_interface::return_type::OK;
  // }

