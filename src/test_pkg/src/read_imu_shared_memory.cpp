// -----------------------------
// Read IMU data from shared memory
// -----------------------------




#include "rclcpp/rclcpp.hpp"
#include "test_pkg/robot_shared_memory.hpp"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <cstring>

namespace bip = boost::interprocess;
#define SHM_NAME "robot_control_shm"

class ShmReaderNode : public rclcpp::Node {
public:
    ShmReaderNode() : Node("shm_reader_node")
    {
        // Run loop at 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            std::bind(&ShmReaderNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "SHM Reader Started. Waiting for Writer...");
    }

private:
    // --- Connection Helper ---
    bool try_connect() {
        try {
            // 1. Try to open the shared memory object
            // We use 'open_only' because we are just a reader
            shm_obj_ = std::make_unique<bip::shared_memory_object>(
                bip::open_only, 
                SHM_NAME, 
                bip::read_only
            );

            // 2. Map the region
            mapped_region_ = std::make_unique<bip::mapped_region>(
                *shm_obj_, 
                bip::read_only
            );

            // 3. Get the pointer
            shm_ptr_ = static_cast<const RobotSharedMemory*>(mapped_region_->get_address());

            RCLCPP_INFO(this->get_logger(), "Successfully Connected to Shared Memory!");
            
            // Reset watchdog
            last_activity_time_ = this->now();
            return true;

        } catch (const bip::interprocess_exception &ex) {
            // This is expected if the Writer hasn't started yet
            // We don't print ERROR every time to avoid log spam, just DEBUG
            RCLCPP_DEBUG(this->get_logger(), "Waiting for SHM: %s", ex.what());
            return false;
        }
    }

    // --- Disconnection Helper ---
    void reset_connection() {
        shm_ptr_ = nullptr;
        mapped_region_.reset(); // Frees the memory mapping
        shm_obj_.reset();       // Closes the file handle
    }

    void timer_callback() {
        // --- STATE 1: Not Connected ---
        if (!shm_ptr_) {
            if (!try_connect()) {
                // If failed, just return and try again next tick
                return; 
            }
        }

        // --- STATE 2: Connected (Read & Watchdog) ---

        // 1. Read Sequence (Start)
        uint32_t seq_start = shm_ptr_->imu_seq;

        // --- WATCHDOG CHECK ---
        // If the sequence hasn't changed in >1.0 second, the Writer probably died/restarted.
        // We must close and reopen to find the "new" file.
        if (seq_start == last_seq_) {
            auto time_diff = this->now() - last_activity_time_;
            if (time_diff.seconds() > 1.0) {
                RCLCPP_WARN(this->get_logger(), "Stale Data Detected (Writer died?). Reconnecting...");
                reset_connection();
                return;
            }
        } else {
            // Sequence changed! Data is alive.
            last_seq_ = seq_start;
            last_activity_time_ = this->now();
        }

        // 2. Seqlock Logic: Is Writer Busy? (Odd number)
        if (seq_start % 2 != 0) {
            return; // Skip this cycle
        }

        // 3. Copy Data (Atomic-ish copy to local buffer)
        float local_imu[4];
        std::memcpy(local_imu, shm_ptr_->imu_data, sizeof(local_imu));

        // 4. Read Sequence (End)
        uint32_t seq_end = shm_ptr_->imu_seq;

        // 5. Verify Consistency
        if (seq_end != seq_start) {
            // Writer interrupted us. Data is torn.
            return; 
        }

        // --- SUCCESS ---
        // Only print every 50th message (approx 1 per second) to keep logs clean
        // or print every time if you debugging
        RCLCPP_INFO(this->get_logger(), "Safe Read | IMU: x=%.3f, y=%.3f, z=%.3f, w=%.3f", local_imu[0], local_imu[1], local_imu[2], local_imu[3]);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "Safe Read | IMU: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
        //     local_imu[0], local_imu[1], local_imu[2], local_imu[3]
        // );
    }

    // Boost Objects
    std::unique_ptr<bip::shared_memory_object> shm_obj_;
    std::unique_ptr<bip::mapped_region> mapped_region_;
    const RobotSharedMemory* shm_ptr_ = nullptr;

    // Watchdog variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_activity_time_;
    uint32_t last_seq_ = 0;
};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShmReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}