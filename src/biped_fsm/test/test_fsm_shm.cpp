#include <iostream>
#include <thread>
#include <chrono>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// Include your custom struct definition
#include "biped_fsm/biped_shared_memory_structs.hpp"

namespace bip = boost::interprocess;
using namespace std::chrono_literals;

#define SHM_NAME "biped_shm"

int main() {
    // Outer loop: Handles Reconnecting
    while (true) {
        try {
            // 1. Try to open the memory
            bip::shared_memory_object shm_obj(bip::open_only, SHM_NAME, bip::read_only);
            bip::mapped_region region(shm_obj, bip::read_only);
            const BipedSharedMemory* shm_ptr = static_cast<const BipedSharedMemory*>(region.get_address());

            uint32_t last_seq = 0;
            int freeze_count = 0;

            // Inner loop: Handles live reading
            while (true) {
                // Clear the terminal screen 
                std::cout << "\033[2J\033[1;1H"; 
                
                std::cout << "--- Biped Shared Memory Live Dashboard ---" << std::endl;
                
                std::cout << "\n[Robot State]" << std::endl;
                std::cout << "  Current State Enum: " << static_cast<int>(shm_ptr->robot_state.current_state) << std::endl;
                std::cout << "  Sequence Counter:   " << shm_ptr->robot_state.seq_counter << std::endl;

                std::cout << "\n[IMU Data]" << std::endl;
                std::cout << "  Roll:  " << shm_ptr->imu.orientation[0] << std::endl;
                std::cout << "  Pitch: " << shm_ptr->imu.orientation[1] << std::endl;
                std::cout << "  Yaw:   " << shm_ptr->imu.orientation[2] << std::endl;

                std::cout << "\n[Joy Command Velocity]" << std::endl;
                std::cout << "  Linear X:  " << shm_ptr->cmd_vel.linear_x << std::endl;
                std::cout << "  Angular Z: " << shm_ptr->cmd_vel.angular_z << std::endl;

                // --- HEARTBEAT CHECK ---
                if (shm_ptr->robot_state.seq_counter == last_seq) {
                    freeze_count++;
                    if (freeze_count > 10) { // If frozen for 1 second (10 loops of 100ms)
                        std::cout << "\n[WARNING] Main node disconnected. Dropping ghost memory..." << std::endl;
                        break; // Break inner loop to destroy pointers and reconnect
                    }
                } else {
                    freeze_count = 0; // Reset freeze counter
                    last_seq = shm_ptr->robot_state.seq_counter;
                }

                std::this_thread::sleep_for(100ms);
            }
        }
        catch (const bip::interprocess_exception& ex) {
            std::cout << "\033[2J\033[1;1H"; // Clear screen
            std::cout << "[WAITING] Looking for biped_shm... Start your state_manager_node." << std::endl;
            std::this_thread::sleep_for(1000ms);
        }
    }

    return 0;
}