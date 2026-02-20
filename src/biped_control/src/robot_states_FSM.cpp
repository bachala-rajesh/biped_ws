#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class ImuToRpyNode : public rclcpp::Node {
public:
  ImuToRpyNode() : Node("robot_states_FSM_node")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      10,
      std::bind(&ImuToRpyNode::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "IMU to RPY node started. Subscribed to 'imu' topic.");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto& q = msg->orientation;
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "RPY [rad]: roll=%.3f pitch=%.3f yaw=%.3f",
      roll,
      pitch,
      yaw);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuToRpyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
