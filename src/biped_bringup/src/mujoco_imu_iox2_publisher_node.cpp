#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "iox2/iceoryx2.hpp"
#include "iox2_msgs/iox_imu_msgs.hpp"

using namespace iox2;

#define IOX_IMU_TOPIC "iox/imu_data"


class MujocoImuIoX2Node : public rclcpp::Node
{
public:
  MujocoImuIoX2Node() : Node("mujoco_imu_iox2_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&MujocoImuIoX2Node::imu_callback, this, std::placeholders::_1));

    init_iox_services();  
    RCLCPP_INFO(this->get_logger(), "IMU IOX2 node started. Listening on /imu_sensor_broadcaster/imu");
  }

  void init_iox_services() {
        // create iox2 node and connect to the services
        auto node_name = NodeName::create("imu_publisher_node").value();
        iox_node_ = NodeBuilder().name(node_name).create<ServiceType::Ipc>().value();
        auto service_name = ServiceName::create(IOX_IMU_TOPIC);
        auto imu_service_result = iox_node_->service_builder(service_name.value()).publish_subscribe<IoxImuData>().open_or_create();
        if (!imu_service_result.has_value()) {
            auto error = imu_service_result.error();
            std::cerr << "Failed! Error: " 
                    << iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error) 
                    << " (code: " << static_cast<int>(error) << ")" << std::endl;
        }

        auto& iox_imu_service = imu_service_result.value();

        iox_imu_publisher_ = iox_imu_service.publisher_builder().create().value();
        std::cout << "✅ iox2 Publisher for IMU connected successfully..." << std::endl;
    }



private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // copy the message to the IoxImuData struct
    IoxImuData iox_imu_data;
    iox_imu_data.orientation_x = msg->orientation.x;
    iox_imu_data.orientation_y = msg->orientation.y;
    iox_imu_data.orientation_z = msg->orientation.z;
    iox_imu_data.orientation_w = msg->orientation.w;
    iox_imu_data.ang_vel_x = msg->angular_velocity.x;
    iox_imu_data.ang_vel_y = msg->angular_velocity.y;
    iox_imu_data.ang_vel_z = msg->angular_velocity.z;
    iox_imu_data.lin_acc_x = msg->linear_acceleration.x;
    iox_imu_data.lin_acc_y = msg->linear_acceleration.y;
    iox_imu_data.lin_acc_z = msg->linear_acceleration.z;

    // publish the iox2 data
    auto sample = iox_imu_publisher_->loan_uninit().value();
    auto initialized_sample = sample.write_payload(iox_imu_data);
    send(std::move(initialized_sample)).value();
  }

  // variables
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  std::optional<iox2::Node<iox2::ServiceType::Ipc>> iox_node_;
  std::optional<iox2::Publisher<iox2::ServiceType::Ipc, IoxImuData, void>> iox_imu_publisher_;
    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MujocoImuIoX2Node>());
  rclcpp::shutdown();
  return 0;
}
