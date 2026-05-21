#include "test_mujoco/mujoco_sim.hpp"
#include <stdexcept>
#include <iostream>
#include <string>
#include <Eigen/Geometry>





namespace locomotion {



MujocoSim::MujocoSim(const std::string& xml_path) {
    char error_buffer[1024];
    model_ = mj_loadXML(xml_path.c_str(), nullptr, error_buffer, sizeof(error_buffer));
    if (!model_) {
        throw std::runtime_error("Failed to load XML file: " + std::string(error_buffer));
    }
    data_ = mj_makeData(model_);

    std::cout << "Loaded: " << xml_path << "\n"
                << " nq: " << model_->nq
                << " nv: " << model_->nv
                << " nu: " << model_->nu
                << " timestep: " << model_->opt.timestep
                << std::endl;

}


MujocoSim::~MujocoSim() {
    if (data_) {
        mj_deleteData(data_);
    }
    if (model_) {
        mj_deleteModel(model_);
    }
}

void MujocoSim::reset() {
    mj_resetData(model_, data_);
}

void MujocoSim::step() {
    mj_step(model_, data_);
}

void MujocoSim::zero_ctrl() {
    for (int i = 0; i < model_->nu; i++) {
        data_->ctrl[i] = 0.0;
    }
}

void MujocoSim::set_joint_position(const std::string& name, double value) {
    int joint_id = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
        throw std::runtime_error("Joint not found: " + name);
    }
    int addr = model_->jnt_qposadr[joint_id];    
    data_->qpos[addr] = value;
}

void MujocoSim::set_ctrl(const std::string& actuator_name, double value) {
    int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id < 0) {
        throw std::runtime_error("Actuator not found: " + actuator_name);
    }
    data_->ctrl[actuator_id] = value;
}

RobotSensorData MujocoSim::get_mujoco_data(const std::vector<std::string>& joint_names) const {
    RobotSensorData robot_sensor_data;
    
    //orientation. Note: the order is in w, x, y, z form
    int orient_sid = mj_name2id(model_, mjOBJ_SENSOR, "imu_quat");
    if (orient_sid < 0) {
        throw std::runtime_error("Orientation sensor not found");
    }
    int orient_adr = model_->sensor_adr[orient_sid];
    robot_sensor_data.imu_quat.w = data_->sensordata[orient_adr + 0];
    robot_sensor_data.imu_quat.x = data_->sensordata[orient_adr + 1];
    robot_sensor_data.imu_quat.y = data_->sensordata[orient_adr + 2];
    robot_sensor_data.imu_quat.z = data_->sensordata[orient_adr + 3];

    
  

    // linear velocity in the base frame
    const Eigen::Vector3d v_world(data_->qvel[0], data_->qvel[1], data_->qvel[2]);
    robot_sensor_data.lin_vel_world = {v_world.x(), v_world.y(), v_world.z()};

    // angular velocity (already in the base frame, from gyro on imu site)
    int gyro_sid = mj_name2id(model_, mjOBJ_SENSOR, "imu_gyro");
    if (gyro_sid < 0) throw std::runtime_error("Angular velocity sensor not found");
    int gyro_addr = model_->sensor_adr[gyro_sid];
    robot_sensor_data.imu_gyro = {data_->sensordata[gyro_addr + 0],
                                data_->sensordata[gyro_addr + 1],
                                data_->sensordata[gyro_addr + 2]};

    
    // joint positions and velocities
    robot_sensor_data.joint_pos.reserve(joint_names.size());
    robot_sensor_data.joint_vel.reserve(joint_names.size());
    for (const auto& name: joint_names) {
        int joint_id = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
        if (joint_id < 0) throw std::runtime_error("Joint not found: " + name);
        int qpos_addr = model_->jnt_qposadr[joint_id];
        int dof_addr = model_->jnt_dofadr[joint_id];
        robot_sensor_data.joint_pos.push_back(data_->qpos[qpos_addr]);
        robot_sensor_data.joint_vel.push_back(data_->qvel[dof_addr]); 
    }

    return robot_sensor_data;
}

void MujocoSim::set_gains(double stiffness, double damping) {
  for (int i = 0; i < model_->nu; ++i) {
    model_->actuator_gainprm[i * 10 + 0] = stiffness;
    model_->actuator_biasprm[i * 10 + 1] = -stiffness;  // position feedback
    model_->actuator_biasprm[i * 10 + 2] = -damping;    // velocity damping (must be negative)
  }
}



} // namespace locomotion