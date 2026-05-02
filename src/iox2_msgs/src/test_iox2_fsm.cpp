#include "iox2/iceoryx2.hpp"
#include "iox2_msgs/iox_fsm_states_msg.hpp"
#include "biped_fsm/robot_states_enum.hpp"
#include <iostream>
#include <iomanip>
#include <unistd.h>

using namespace iox2;

int main() {
    std::cout << "📡 C++ iox2 FSM Subscriber Started" << std::endl;
    std::cout << "Connecting to: iox/fsm_data" << std::endl;
    std::cout << "Waiting for data from FSM publisher...\n" << std::endl;

    // Optional: Set log level
    set_log_level_from_env_or(LogLevel::Warn);

    // 1. Create node
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();

    // 2. Connect to service
    auto service_name = ServiceName::create("iox/fsm_data");

    auto service_result = node.service_builder(service_name.value())
        .publish_subscribe<IoxFsmData>()
        .open_or_create();

    if (!service_result.has_value()) {
        auto error = service_result.error();
        std::cerr << "Failed! Error: "
                  << iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error)
                  << " (code: " << static_cast<int>(error) << ")" << std::endl;
        return 1;
    }

    auto &service = service_result.value();

    // 3. Create subscriber
    auto subscriber = service.subscriber_builder().create().value();

    std::cout << "✅ C++ FSM Subscriber connected successfully!" << std::endl;
    std::cout << "Listening for FSM data (Ctrl+C to exit)\n" << std::endl;

    // 4. Main loop
    while (true) {
        // Receive data (non-blocking)
        auto sample = subscriber.receive().value();

        if (sample.has_value()) {
            const auto& fsm_data = sample->payload();

            // Display data
            std::cout << std::fixed << std::setprecision(4)
                      << "FSM Data:" << std::endl
                      << "   linear_x:    " << fsm_data.linear_x << std::endl
                      << "   linear_y:    " << fsm_data.linear_y << std::endl
                      << "   angular_z:   " << fsm_data.angular_z << std::endl
                      << "   state:       " << robot_states_enum::get_state_string(fsm_data.current_state)
                      << " (" << static_cast<int>(fsm_data.current_state) << ")" << std::endl
                      << std::endl;
        } else {
            // No data available, sleep 10ms (100Hz check rate)
            usleep(10000);
        }
    }

    return 0;
}
