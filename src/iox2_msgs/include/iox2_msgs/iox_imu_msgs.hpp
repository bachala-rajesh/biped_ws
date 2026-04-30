#include  <iostream>

struct IoxImuData {
    static constexpr const char* IOX2_TYPE_NAME = "IoxImuData";
    // Orientation in quaternion form
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;

    // Angular velocity in rad/s
    double ang_vel_x;
    double ang_vel_y;
    double ang_vel_z;

    // Linear acceleration in m/s^2
    double lin_acc_x;
    double lin_acc_y;
    double lin_acc_z;

    friend std::ostream& operator<<(std::ostream& os, const IoxImuData& data) {
        os << "Orientation: ["
           << data.orientation_x
           << ", Orientation_y=" << data.orientation_y
           << ", Orientation_z=" << data.orientation_z
           << ", Orientation_w=" << data.orientation_w
           << "]"
           << ", Angular velocity: ["
           << data.ang_vel_x
           << ", ang_vel_y=" << data.ang_vel_y
           << ", ang_vel_z=" << data.ang_vel_z
           << "]"
           << ", Linear acceleration: ["
           << data.lin_acc_x
           << ", lin_acc_y=" << data.lin_acc_y
           << ", lin_acc_z=" << data.lin_acc_z
           << "]"
           << std::endl;
        return os;
    }
};  

