#include "iox2/iceoryx2.hpp"
#include "iox2_msgs/iox_imu_msgs.hpp"
#include <iostream>
#include <iomanip>
#include <unistd.h>

using namespace iox2;

#define IOX_IMU_TOPIC "iox/imu_data"

int main() {
    std::cout << "📡 C++ iox2 IMU Subscriber Started" << std::endl;
    std::cout << "Connecting to: " << IOX_IMU_TOPIC << std::endl;
    std::cout << "Waiting for data from publisher...\n" << std::endl;

    set_log_level_from_env_or(LogLevel::Warn);

    // 1. Create node
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();

    // 2. Connect to service
    auto service_name = ServiceName::create(IOX_IMU_TOPIC);
    auto service_result = node.service_builder(service_name.value())
        .publish_subscribe<IoxImuData>()
        .open_or_create();

    if (!service_result.has_value()) {
        auto error = service_result.error();
        std::cerr << "Failed to open/create service! Error: "
                  << iox2::bb::from<iox2::PublishSubscribeOpenOrCreateError, const char*>(error)
                  << " (code: " << static_cast<int>(error) << ")" << std::endl;
        return 1;
    }

    auto& service = service_result.value();

    // 3. Create subscriber
    auto subscriber = service.subscriber_builder().create().value();

    std::cout << "✅ C++ Subscriber connected successfully!" << std::endl;
    std::cout << "Listening for IMU data (Ctrl+C to exit)\n" << std::endl;

    // 4. Main loop
    while (true) {
        auto sample = subscriber.receive().value();

        if (sample.has_value()) {
            const auto& imu = sample->payload();

            std::cout << std::fixed << std::setprecision(4)
                      << "   Orientation: [" << imu.orientation_x << ", " << imu.orientation_y
                      << ", " << imu.orientation_z << ", " << imu.orientation_w << "]" << std::endl
                      << "   Angular Vel: [" << imu.ang_vel_x << ", " << imu.ang_vel_y
                      << ", " << imu.ang_vel_z << "] rad/s" << std::endl
                      << "   Linear Acc:  [" << imu.lin_acc_x << ", " << imu.lin_acc_y
                      << ", " << imu.lin_acc_z << "] m/s²" << std::endl;
        } else {
            usleep(10000);
        }
    }

    return 0;
}
