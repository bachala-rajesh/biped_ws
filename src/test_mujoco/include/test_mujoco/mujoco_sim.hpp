#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include <vector>
#include <array>

#include "test_mujoco/robot_state_struct.hpp"

namespace test_mujoco {




class MujocoSim {
public:


    explicit MujocoSim(const std::string& xml_path);
    ~MujocoSim();

    MujocoSim(const MujocoSim&) = delete;
    MujocoSim& operator=(const MujocoSim&) = delete;    
    

    void reset();
    void step();
    void zero_ctrl();
    void forward() {mj_forward(model_, data_);} 

    // raw acess for the model and the data
    mjModel* model() { return model_; }
    mjData* data() { return data_; }


    double time() { return data_->time; }
    double time_step() { return model_->opt.timestep; }

    // set joint position by name
    void set_joint_position(const std::string& name, double value);

    // set data->ctrl for one actuator by name
    void set_ctrl(const std::string& actuator_name, double value);

    // get sensor data from mujoco
    RobotSensorData get_mujoco_data(const std::vector<std::string>& joint_names) const;

    void set_gains(double stiffness, double damping);



private:
    mjModel* model_;
    mjData* data_;
};

} // namespace test_mujoco
