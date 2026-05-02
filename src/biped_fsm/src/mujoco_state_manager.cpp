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

#include "iox2/iceoryx2.hpp"
#include "iox2_msgs/iox_fsm_states_msg.hpp"
#include "biped_fsm/robot_states_enum.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>




using namespace iox2;


#define IOX_FSM_TOPIC "iox/fsm_data"



class StateManager : public rclcpp::Node {
public:
    StateManager(std::shared_ptr<yasmin::Blackboard> blackboard) : Node("state_manager_node"), blackboard_(blackboard) {

        // initialize iox node and publisher/subscriber
        init_iox_services();

        // callback groups
        common_non_critical_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
        // subscription options- assign callback groups to the subscribers
        rclcpp::SubscriptionOptions joy_cmd_sub_options;
        joy_cmd_sub_options.callback_group = common_non_critical_callback_group_;
        rclcpp::SubscriptionOptions joy_state_sub_options;
        joy_state_sub_options.callback_group = common_non_critical_callback_group_;

        // subscribers
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



private:

    void joy_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        blackboard_->set<bool>("joy_cmd_status", true);

        // write the velocities to the cmd_vel
        cmd_vel[0] = msg->twist.linear.x;
        cmd_vel[1] = msg->twist.linear.y;
        cmd_vel[2] = msg->twist.angular.z;

    }

    void joy_state_callback(const std_msgs::msg::String::SharedPtr msg) {
        blackboard_->set<std::string>("joy_state", msg->data);
        blackboard_->set<bool>("joy_state_status", true);
    }

    void timer_callback() {

        // get the current robot state from the blackboard
        robot_states_enum::RobotState current_robot_state = blackboard_->get<robot_states_enum::RobotState>("robot_state");

        // log the current robot state
        // RCLCPP_INFO(this->get_logger(), "Current Robot State: %s", robot_states_enum::get_state_string(current_robot_state).c_str());

        // iox2 data
        IoxFsmData iox_fsm_data;
        if (current_robot_state == robot_states_enum::RobotState::ACTIVE) {
            iox_fsm_data.linear_x = cmd_vel[0];
            iox_fsm_data.linear_y = cmd_vel[1];
            iox_fsm_data.angular_z = cmd_vel[2];
        } else {
            iox_fsm_data.linear_x = 0.0;
            iox_fsm_data.linear_y = 0.0;
            iox_fsm_data.angular_z = 0.0;
        }

        iox_fsm_data.current_state = current_robot_state;


        // publish the iox2 data
        auto sample = iox_fsm_publisher_->loan_uninit().value();
        auto initialized_sample = sample.write_payload(iox_fsm_data);
        send(std::move(initialized_sample)).value();
    }

    void init_iox_services() {
        // create iox2 node and connect to the services
        auto node_name = NodeName::create("fsm_publisher_node").value();
        iox_node_ = NodeBuilder().name(node_name).create<ServiceType::Ipc>().value();
        auto service_name = ServiceName::create(IOX_FSM_TOPIC);
        auto fsm_service_result = iox_node_->service_builder(service_name.value()).publish_subscribe<IoxFsmData>().open_or_create();
        if (!fsm_service_result.has_value()) {
            auto error = fsm_service_result.error();
            std::cerr << "Failed! Error: " 
                    << iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error) 
                    << " (code: " << static_cast<int>(error) << ")" << std::endl;
        }

        auto& iox_fsm_service = fsm_service_result.value();

        iox_fsm_publisher_ = iox_fsm_service.publisher_builder().create().value();
        std::cout << "✅ iox2 Publisher for FSM connected successfully..." << std::endl;
    }





    //------------------------- variables ------------------------
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<yasmin::Blackboard> blackboard_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joy_state_sub_;
    rclcpp::CallbackGroup::SharedPtr common_non_critical_callback_group_;
    std::array<float, 3> cmd_vel;

    // iox2 related variables   
    std::optional<iox2::Node<iox2::ServiceType::Ipc>> iox_node_;
    std::optional<iox2::Publisher<iox2::ServiceType::Ipc, IoxFsmData, void>> iox_fsm_publisher_;
    

    
    
    //------------------------------------------------------------


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // blackboard
    auto blackboard = std::make_shared<yasmin::Blackboard>();
    
    // ros2 node
    auto state_manager_ros_node = std::make_shared<StateManager>(blackboard);

    // -------------------------- set inital values in the blackboard --------------------------
    //status flags
    blackboard->set<bool>("joy_cmd_status", false);
    blackboard->set<bool>("joy_state_status", false);

    // states
    blackboard->set<std::string>("joy_state", "none");
    blackboard->set<robot_states_enum::RobotState>("robot_state", robot_states_enum::RobotState::INIT); // set the initial state to ACTIVE

    // -------------------------- create the state machine --------------------------
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("biped_fsm");
    const std::string xml_path    = pkg_share + "/xml/mujoco_fsm_states_transistions.xml";
    yasmin_factory::YasminFactory factory;
    auto sm = factory.create_sm_from_file(xml_path);
    sm->set_sigint_handler(false);

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
    rclcpp::shutdown();
    executor.cancel();
    executor_thread.join();
    return 0;
}