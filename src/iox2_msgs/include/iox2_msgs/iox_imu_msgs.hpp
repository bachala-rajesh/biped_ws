#include  <iostream>

struct IoxImuData {
    static constexpr const char* IOX2_TYPE_NAME = "IoxImuData";
    double roll;
    double pitch;
    double yaw;

    friend std::ostream& operator<<(std::ostream& os, const IoxImuData& data) {
        os << "Roll=" << data.roll << ", Pitch=" << data.pitch << ", Yaw=" << data.yaw;
        return os;
    }
};