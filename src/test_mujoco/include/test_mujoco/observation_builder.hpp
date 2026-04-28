#pragma once

#include <array>
#include <deque>
#include <vector>

#include <test_mujoco/config.hpp>
#include <test_mujoco/mujoco_sim.hpp>


namespace test_mujoco {

using Deque = std::deque<std::vector<double>>;

constexpr int n_obs_terms = 8;

struct ObservationHistory {
    std::deque<std::vector<double>> ang_vel;            //3
    std::deque<std::vector<double>> proj_gravity;       //3
    std::deque<std::vector<double>> cmd_vel;            //3
    std::deque<std::vector<double>> joint_pos_rel;      //6
    std::deque<std::vector<double>> joint_vel;          //6
    std::deque<std::vector<double>> last_actions;       //6
    std::deque<std::vector<double>> gait_phase;         //2
    std::deque<std::vector<double>> gait_command;       //3


    std::array<Deque*, n_obs_terms> all_buffers(){
        return {
            &ang_vel, 
            &proj_gravity, 
            &cmd_vel, 
            &joint_pos_rel, 
            &joint_vel, 
            &last_actions, 
            &gait_phase, 
            &gait_command
        };
    }

    std::array<const Deque*, n_obs_terms> all_buffers() const {
        return {
            &ang_vel, 
            &proj_gravity, 
            &cmd_vel, 
            &joint_pos_rel, 
            &joint_vel, 
            &last_actions, 
            &gait_phase, 
            &gait_command
        };
    }
};


class ObservationBuilder {
public:
    explicit ObservationBuilder(const Config& cfg);

    // read raw state, build and push 
    void scale_update_observation(const MujocoObservation& raw,
                      double gait_time,
                      const std::array<double, 3>& cmd_vel,
                      const std::vector<double>& last_actions
                    );

    // initialize the history with the first observation
    void init_history();

    // it returns a stacked observation vector
    std::vector<float> stacked_observation() const;



private:
    void push_recent_observation(std::deque<std::vector<double>>& d_history, std::vector<double> recent_data) const; 

    // variables
    Config cfg_;
    std::vector<double> initial_joint_pos_;
    ObservationHistory obs_history_;


};

}

