#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include <vector>
#include <array>


namespace test_mujoco {

struct MujocoObservation {
    std::array<double, 3> proj_gravity;
    std::array<double, 3> lin_vel;
    std::array<double, 3> ang_vel;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    bool fallen = false;
};


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

    MujocoObservation get_observation(const std::vector<std::string>& joint_names) const;

    void set_gains(double stiffness, double damping);



private:
    mjModel* model_;
    mjData* data_;
};

} // namespace test_mujoco
