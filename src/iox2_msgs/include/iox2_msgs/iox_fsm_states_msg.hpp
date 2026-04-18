#include  <iostream>
#include <cstdint>
#include "biped_fsm/robot_states_enum.hpp"

struct IoxFsmData {
    static constexpr const char* IOX2_TYPE_NAME = "IoxFsmData";
    double linear_x;
    double linear_y;
    double angular_z;
    robot_states_enum::RobotState current_state;
};