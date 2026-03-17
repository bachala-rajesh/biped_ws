#include <iostream>
#include <string>
#include <thread>
#include <memory>

// ROS 2 headers
#include "rclcpp/rclcpp.hpp"

// Yasmin headers
#include "yasmin/blackboard.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"
#include "yasmin_factory/yasmin_factory.hpp"

int main(int argc, char **argv) {
    std::cout << "--- Starting Yasmin Pluginlib Factory Demo ---" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("simple_fsm_node");
    auto blackboard = std::make_shared<yasmin::Blackboard>();

    // 1. Create the Yasmin Factory
    yasmin_factory::YasminFactory factory;

    // 2. Load the XML file
    std::string xml_path = "/home/mira/workspaces/biped_ws/src/biped_fsm/src/demo_states.xml";
    
    // 3. Create the state machine. 
    // The factory automatically finds the plugins using ROS 2 pluginlib!
    auto sm = factory.create_sm_from_file(xml_path);
    
    // 4. Set up viewer and background thread
    yasmin_viewer::YasminViewerPub viewer(node, sm, "robot_states");

    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    std::cout << "Executing State Machine..." << std::endl;
    
    std::string final_outcome = (*sm)(blackboard);
    
    std::cout << "State Machine Finished with outcome: " << final_outcome << std::endl;

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}