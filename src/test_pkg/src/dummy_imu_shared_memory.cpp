// -----------------------------
// generates dummy IMU data and writes to shared memory
// -----------------------------






#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "test_pkg/robot_shared_memory.hpp"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <random>

// namespace for boost interprocess
namespace bip = boost::interprocess;

#define SHM_NAME "robot_control_shm"

class DummyImuNode : public rclcpp::Node
{
public:
    DummyImuNode() : Node("dummy_imu_node")
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DummyImuNode::timer_callback, this));

        // setup random number generator for dummy IMU data
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<float>(-1.0, 1.0);

        try {
            // remove shared memory if it already exists
            bip::shared_memory_object::remove(SHM_NAME);

            // create shared memory object
            shm_obj_ = std::make_unique<bip::shared_memory_object>(
                bip::open_or_create, SHM_NAME, bip::read_write);

            // set size of shared memory
            shm_obj_->truncate(sizeof(RobotSharedMemory));

            // map the memory to this process
            mapped_region_ = std::make_unique<bip::mapped_region>(*shm_obj_, bip::read_write);
            
            // get pointer to shared memory
            shm_ptr_ = static_cast<RobotSharedMemory*>(mapped_region_->get_address());

            // initialize header
            shm_ptr_->imu_seq = 0;

            RCLCPP_INFO(this->get_logger(), "Shared Memory initialized successfully");
        }
        catch (const bip::interprocess_exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing shared memory: %s", ex.what());
            rclcpp::shutdown();
        }
    }

    ~DummyImuNode()
    {
        bip::shared_memory_object::remove(SHM_NAME);
        RCLCPP_INFO(this->get_logger(), "Shared Memory removed");
    }

private: 
    void timer_callback()
    { 
        // generate dummy IMU data
        float x = dist_(gen_);
        float y = dist_(gen_);
        float z = dist_(gen_);
        float w = dist_(gen_);

        // write to shared memory (Seqlock pattern)
        if (shm_ptr_) {
            // Lock: increment the seq to odd
            shm_ptr_->imu_seq++; 

            shm_ptr_->imu_data[0] = x; 
            shm_ptr_->imu_data[1] = y;
            shm_ptr_->imu_data[2] = z;

            // increment sequence after writing
            shm_ptr_->imu_seq++; 

            // 3. Publish ROS Message
            auto message = sensor_msgs::msg::Imu();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "imu_link";
            message.orientation.x = x;
            message.orientation.y = y;
            message.orientation.z = z;
            message.orientation.w = w;
            
            imu_pub_->publish(message);
        }
    };


    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Boost objects
    std::unique_ptr<bip::shared_memory_object> shm_obj_;
    std::unique_ptr<bip::mapped_region> mapped_region_;
    RobotSharedMemory* shm_ptr_ = nullptr;

    // random number generator
    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyImuNode>());
    rclcpp::shutdown();
    return 0;
}