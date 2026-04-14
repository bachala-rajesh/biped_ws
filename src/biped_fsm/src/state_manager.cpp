/*
TODO: 
- modify the subscriber rate of the imu
- add std::mutex to avoid race conditions when accessing the blackboard 
- fill the destroy node method
- note: the imu node is already publsihing RPY.. directly use it
- remove the fake data generation in the timer callback
- move the fsm states transistions file to the config directory
*/

#include <random>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <unordered_map>
#include "yasmin/blackboard.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "biped_fsm/biped_shared_memory_structs.hpp"
#include "biped_fsm/robot_states_enum.hpp"


// namespaces
using namespace std::chrono_literals;
namespace bip = boost::interprocess;

#define SHM_NAME "biped_shm"


// global variables
bool SHUTTING_DOWN = false;



class StateManager : public rclcpp::Node {
public:
    StateManager(std::shared_ptr<yasmin::Blackboard> blackboard) : Node("state_manager_node"), blackboard_(blackboard) {

        // initialize shared memory
        init_shared_memory();

        // callback groups
        imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        joint_states_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        common_non_critical_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  

        // subscription options- assign callback groups to the subscribers
        rclcpp::SubscriptionOptions imu_sub_options;
        imu_sub_options.callback_group = imu_callback_group_;
        rclcpp::SubscriptionOptions joint_states_sub_options;
        joint_states_sub_options.callback_group = joint_states_callback_group_;
        rclcpp::SubscriptionOptions joy_cmd_sub_options;
        joy_cmd_sub_options.callback_group = common_non_critical_callback_group_;
        rclcpp::SubscriptionOptions joy_state_sub_options;
        joy_state_sub_options.callback_group = common_non_critical_callback_group_;

        // subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 
                                                                    10, 
                                                                    std::bind(&StateManager::imu_callback, this, std::placeholders::_1),
                                                                    imu_sub_options
                                                                    );
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 
                                                                    10, 
                                                                    std::bind(&StateManager::joint_states_callback, this, std::placeholders::_1),
                                                                    joint_states_sub_options
                                                                    );
        joy_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/joy/cmd", 
                                                                    10, 
                                                                    std::bind(&StateManager::joy_cmd_callback, this, std::placeholders::_1),
                                                                    joy_cmd_sub_options
                                                                    );
        joy_state_sub_ = this->create_subscription<std_msgs::msg::String>("/joy/state", 
                                                                    10, 
                                                                    std::bind(&StateManager::joy_state_callback, this, std::placeholders::_1),
                                                                    joy_state_sub_options
                                                                    );


        // timer
        timer_ = this->create_wall_timer( 20ms, std::bind(&StateManager::timer_callback, this), common_non_critical_callback_group_ ); //50Hz timer
    }

    void safe_shutdown() {
        // stop the robot 
        RCLCPP_INFO(this->get_logger(), "Shutdown signal received. Stopping the robot safely...");
        
        write_stop_state_to_shm();
        RCLCPP_INFO(this->get_logger(), "Safely disconnecting from shared memory...");
        mapped_region_.reset(); 
        shm_obj_.reset();
        shm_ptr_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "destroying the node...");
    }

    void write_stop_state_to_shm() {
        SHUTTING_DOWN = true;

        if (shm_ptr_ != nullptr) {
            // for safety, write the STOP state to the shared memory multiple times to ensure the policy receives it and acts accordingly
            for (int i = 0; i < 3; ++i) {
                shm_ptr_->robot_state.seq_counter += 1;
                // Force the state to STOP 
                shm_ptr_->robot_state.current_state = static_cast<uint8_t>(robot_states_enum::RobotState::STOP);
                shm_ptr_->robot_state.seq_counter += 1;
                RCLCPP_INFO(this->get_logger(), "Stopping the robot...");
                std::this_thread::sleep_for(100ms); // small delay to increase chances of the policy receiving the STOP state
            }
        }
    }


private:

    void init_shared_memory() {
        try {
            // remove shared memory if it already exists
            bip::shared_memory_object::remove(SHM_NAME);

            // create shared memory object
            shm_obj_ = std::make_unique<bip::shared_memory_object>(
                bip::open_or_create, SHM_NAME, bip::read_write);

            // set size of shared memory
            shm_obj_->truncate(sizeof(BipedSharedMemory));

            // map the memory to this process
            mapped_region_ = std::make_unique<bip::mapped_region>(*shm_obj_, bip::read_write);
            
            // get pointer to shared memory
            shm_ptr_ = static_cast<BipedSharedMemory*>(mapped_region_->get_address());

            RCLCPP_INFO(this->get_logger(), "Shared Memory initialized successfully");
        }
        catch (const bip::interprocess_exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing shared memory: %s", ex.what());
            rclcpp::shutdown();
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        
        // Convert quaternion to roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // update the blackboard
        rclcpp::Time current_time (msg->header.stamp);
        blackboard_->set<double>("last_imu_time", current_time.seconds());
        blackboard_->set<double>("imu_roll", roll);
        blackboard_->set<double>("imu_pitch", pitch);
        blackboard_->set<double>("imu_yaw", yaw);   

        blackboard_->set<bool>("imu_status", true);

        // write data to the shared memory
        shm_ptr_->imu.seq_counter += 1;
        shm_ptr_->imu.orientation[0] = roll;
        shm_ptr_->imu.orientation[1] = pitch;
        shm_ptr_->imu.orientation[2] = yaw;
        shm_ptr_->imu.seq_counter += 1;

    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        
        // create a dictionary and fill it with the joint names and positions
        std::unordered_map<std::string, float> mapped_dict; 
        for (size_t i = 0; i < msg->name.size(); ++i) {
            mapped_dict[msg->name[i]] = msg->position[i];
        }

        // update the blackboard
        rclcpp::Time current_time (msg->header.stamp);
        blackboard_->set<double>("last_joint_states_time", current_time.seconds());
        blackboard_->set<bool>("joint_states_status", true);
        blackboard_->set<std::unordered_map<std::string, float>>("joint_positions_dict", mapped_dict);

    }

    void joy_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        blackboard_->set<bool>("joy_cmd_status", true);

        // write data to the shared memory
        shm_ptr_->cmd_vel.seq_counter += 1;
        shm_ptr_->cmd_vel.linear_x = msg->twist.linear.x;
        shm_ptr_->cmd_vel.linear_y = msg->twist.linear.y;
        shm_ptr_->cmd_vel.angular_z = msg->twist.angular.z;
        shm_ptr_->cmd_vel.seq_counter += 1;

    }

    void joy_state_callback(const std_msgs::msg::String::SharedPtr msg) {
        blackboard_->set<std::string>("joy_state", msg->data);
        blackboard_->set<bool>("joy_state_status", true);
    }

    void timer_callback() {
        // if the shutdown has started, return 
        if (SHUTTING_DOWN) {
            timer_->cancel(); // stop the timer
            return;
        }

        // update the current ROS time in the blackboard
        rclcpp::Time current_time = this->get_clock()->now();
        blackboard_->set<double>("current_ros_time", current_time.seconds());
        
        // fake data for testing
        fake_sensor_data();

        // get the current robot state from the blackboard
        uint8_t robot_state_enum = blackboard_->get<uint8_t>("robot_state");

        // write the current robot state to the shared memory
        shm_ptr_->robot_state.seq_counter += 1;
        shm_ptr_->robot_state.current_state = robot_state_enum;
        shm_ptr_->robot_state.seq_counter += 1;
    }

    void fake_sensor_data() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<double> angle_dist(-0.17, 0.17);
        static std::uniform_real_distribution<double> linear_dist(-1.0, 1.0);
        static std::uniform_real_distribution<double> angular_dist(-1.0, 1.0);

        // Simulate IMU data
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = this->get_clock()->now();
        tf2::Quaternion q;
        q.setRPY(angle_dist(gen), angle_dist(gen), angle_dist(gen)); // Roll, Pitch, Yaw
        imu_msg->orientation.x = q.x();
        imu_msg->orientation.y = q.y();
        imu_msg->orientation.z = q.z();
        imu_msg->orientation.w = q.w();
        imu_callback(imu_msg);

        // joint states data
        auto joint_states_msg = std::make_shared<sensor_msgs::msg::JointState>();
        joint_states_msg->header.stamp = this->get_clock()->now();
        joint_states_callback(joint_states_msg);

        // joy cmd data
        auto joy_cmd_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        joy_cmd_msg->twist.linear.x = linear_dist(gen);
        joy_cmd_msg->twist.angular.z = angular_dist(gen);
        joy_cmd_callback(joy_cmd_msg);

        // joy state data
        auto joy_state_msg = std::make_shared<std_msgs::msg::String>();
        joy_state_msg->data = "start";
        joy_state_callback(joy_state_msg);


    }




    //------------------------- variables ------------------------
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<yasmin::Blackboard> blackboard_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joy_state_sub_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr joint_states_callback_group_;
    rclcpp::CallbackGroup::SharedPtr common_non_critical_callback_group_;

    // Boost objects
    std::unique_ptr<bip::shared_memory_object> shm_obj_;
    std::unique_ptr<bip::mapped_region> mapped_region_;
    BipedSharedMemory* shm_ptr_ = nullptr;
    //------------------------------------------------------------


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // blackboard
    auto blackboard = std::make_shared<yasmin::Blackboard>();
    
    // ros2 node
    auto state_manager_ros_node = std::make_shared<StateManager>(blackboard);

    // -------------------------- set inital values in the blackboard --------------------------
    // timestamps
    double start_time = state_manager_ros_node->get_clock()->now().seconds();
    blackboard->set<double>("last_imu_time", start_time);
    blackboard->set<double>("last_joint_states_time", start_time);
    blackboard->set<double>("current_ros_time", start_time);

    // imu angles
    blackboard->set<double>("imu_roll", 0.0);
    blackboard->set<double>("imu_pitch", 0.0);
    blackboard->set<double>("imu_yaw", 0.0);

    // joint angles
    std::unordered_map<std::string, float> joint_positions_dict;
    blackboard->set<std::unordered_map<std::string, float>>("joint_positions_dict", joint_positions_dict);

    //status flags
    blackboard->set<bool>("imu_status", false);
    blackboard->set<bool>("joint_states_status", false);
    blackboard->set<bool>("joy_cmd_status", false);
    blackboard->set<bool>("joy_state_status", false);

    // states
    blackboard->set<std::string>("joy_state", "none");
    blackboard->set<uint8_t>("robot_state", static_cast<uint8_t>(robot_states_enum::RobotState::INIT)); // set the initial state to ACTIVE

    // -------------------------- create the state machine --------------------------
    yasmin_factory::YasminFactory factory;
    std::string xml_path = "/home/mira/workspaces/biped_ws/src/biped_fsm/src/fsm_states_transistions.xml";
    auto sm = factory.create_sm_from_file(xml_path);
    sm->set_sigint_handler(true);

    // -------------------------- initialize ROS node and executor --------------------------// ros2 demo node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(state_manager_ros_node);

    // -------------------------- set up viewer and background thread --------------------------
    yasmin_viewer::YasminViewerPub viewer(state_manager_ros_node, sm, "robot_states");
    
    // Spin the node in a background thread so it can publish/subscribe
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    std::cout << "Node and Blackboard Initialized." << std::endl;
    
    // --------------------------- run the state machine --------------------------
    try {
        std::string outcome = (*sm)(blackboard);
    } 
    catch (const std::exception& e) {
        std::cout << "\n[WARNING] No valid state... " << e.what() << std::endl;
    }
    
    
    
    // cleanup
    state_manager_ros_node->safe_shutdown();
    rclcpp::shutdown();
    executor.cancel();
    executor_thread.join();
    return 0;
}