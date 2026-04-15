#include "iox2/iceoryx2.hpp"
#include "iox2_msgs/iox_imu_msgs.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>

using namespace iox2;


int main() {
    std::cout << "📡 C++ iox2 IMU Subscriber Started" << std::endl;
    std::cout << "Connecting to: iox/imu_data" << std::endl;
    std::cout << "Waiting for data from Python publisher...\n" << std::endl;
    
    // Optional: Set log level
    set_log_level_from_env_or(LogLevel::Warn);
    
    // 1. Create node 
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();


    // 2. Connect to service 
    auto service_name = ServiceName::create("iox/imu_data");

    auto service_result = node.service_builder(service_name.value())
        .publish_subscribe<IoxImuData>()
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
    
    std::cout << "✅ C++ Subscriber connected successfully!" << std::endl;
    std::cout << "Listening for IMU data (Ctrl+C to exit)\n" << std::endl;
    

    // 4. Main loop
    while (true) {
        // Receive data (non-blocking)
        auto sample = subscriber.receive().value();
        
        if (sample.has_value()) {
            // Access payload directly from shared memory (zero-copy)
            
            // std::cout << "received: " << sample->payload() << std::endl;
           
            const auto& imu = sample->payload();

            // Display data
            std::cout << "   Roll:  " << std::fixed << std::setprecision(4) 
                      << imu.roll << " rad (" << imu.roll * 180.0 / M_PI << "°)" << std::endl;
            std::cout << "   Pitch: " << imu.pitch << " rad (" << imu.pitch * 180.0 / M_PI << "°)" << std::endl;
            std::cout << "   Yaw:   " << imu.yaw << " rad (" << imu.yaw * 180.0 / M_PI << "°)" << std::endl;
            
  
        } else {
            // No data available, sleep 10ms (100Hz check rate)
            usleep(10000);
        }
    }
    
    return 0;
}