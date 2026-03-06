#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN
// 1. You MUST define this before including spdlog to enable SPDLOG_DEBUG

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>
#include <cstdint>
#include <iomanip>
#include <fmt/core.h>
#include <array>
#include <vector>
#include <cmath>
#include <serial/serial.h>
#include <thread>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include "motor_structs.h"




#define MIT_SET_HEADER 0x00
#define MIT_MOVE_HEADER 0x02
#define MIT_RX_HEADER 0x78

#define SANPO_FRAME_SIZE 18


std::array<std::uint8_t, 2> SANPO_FRAME_TAIL = {0x0d, 0x0a};
std::array<std::uint8_t, 2> SANPO_FRAME_HEADER = {0x53, 0x54};
std::array<std::uint8_t, 8> ENABLE_MOTOR_BYTES {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};  
std::array<std::uint8_t, 8> DISABLE_MOTOR_BYTES {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};  
std::array<std::uint8_t, 8> ZERO_POS_BYTES {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};  
const std::string handshake_frame = "AT+ET\r\n";

const size_t bytes_to_keep = 360;
const size_t BUFFER_SIZE = 1024; // max buffer size

// --- CAN MOTOR PROTOCOL LIMITS ---
constexpr double P_MIN = -12.56;
constexpr double P_MAX = 12.56;
constexpr double V_MIN = -45.0;
constexpr double V_MAX = 45.0;
constexpr double KP_MIN = 0.0;
constexpr double KP_MAX = 500.0;
constexpr double KD_MIN = 0.0;
constexpr double KD_MAX = 5.0;
constexpr double T_MIN = -18.0;
constexpr double T_MAX = 18.0;


class SanpoProtocolBuilder {
public:
    // convert physical float to raw integer
    static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t num_bits) {
        // clamp x to the range [x_min, x_max]
        if (x > x_max) {
            x = x_max;
        } else if (x < x_min) {
            x = x_min;
        }

        // calculate the span of the range
        float span = x_max - x_min;
        
        // compute max bit value
        uint32_t bit_range = (1 << num_bits) - 1; // 2^num_bits - 1

        // scale and cast to uint32_t
        return static_cast<uint32_t>(((x - x_min) * bit_range) / span);
    }

    // convert raw integer to physical float
    static float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t num_bits) {
        // calculate the span of the range
        float span = x_max - x_min;

        // compute max bit value
        uint32_t bit_range = (1 << num_bits) - 1; // 2^num_bits - 1

        // scale and cast to float
        return static_cast<float>(((x_int * span) / bit_range) + x_min);
    }

    // form the MIT command payload bytes from the given physical float values
    static void form_mit_payload(std::array<uint8_t, 8>& payload_bytes, const MotorCommand& command) {
        // convert each float to uint16_t
        uint32_t p_int  = float_to_uint(command.position,  P_MIN, P_MAX, 16);
        uint32_t v_int  = float_to_uint(command.velocity,  V_MIN, V_MAX,  12);
        uint32_t kp_int = float_to_uint(command.kp, KP_MIN, KP_MAX, 12);
        uint32_t kd_int = float_to_uint(command.kd, KD_MIN, KD_MAX, 12);
        uint32_t t_int  = float_to_uint(command.torque,  T_MIN, T_MAX,  12);

        // pack bits into 64 bits integer
        uint64_t payload = 0;

        payload |= (static_cast<uint64_t>(p_int)  << 48); // bits 48-63 16 bits for position
        payload |= (static_cast<uint64_t>(v_int)  << 36); // bits 36-47 12 bits for velocity
        payload |= (static_cast<uint64_t>(kp_int) << 24); // bits 24-35 12 bits for kp
        payload |= (static_cast<uint64_t>(kd_int) << 12); // bits 12-23 12 bits for kd
        payload |= (static_cast<uint64_t>(t_int)  << 0);  // bits 0-11 12 bits for torque

        // convert the 64-bit payload into an array of 8 bytes
        for (int i = 0; i<8; ++i) {
            payload_bytes[i] = static_cast<uint8_t>((payload >> (56 - 8*i)) & 0xFF); // extract each byte
        }

    }

    // form the full SANPO frame given the command header, motor ID and can channel
    static void form_sanpo_frame(std::vector<uint8_t>& frame, const MotorCommand& command, const uint8_t mit_command_header, const uint8_t can_channel) {
        
        // form the payload bytes from the command
        std::array<uint8_t, 8> payload_bytes;
        form_mit_payload(payload_bytes, command);
        
        frame.reserve(18); //preallocate space for 18 bytes for sanpo frame

        // Add header
        frame.push_back(SANPO_FRAME_HEADER[0]);
        frame.push_back(SANPO_FRAME_HEADER[1]);

        // Add motor ID
        frame.push_back(can_channel); // CAN channel 

        //reserved 2 bytes
        frame.push_back(0x00);
        frame.push_back(0x00);

        // MIT command (header + motor_id)
        frame.push_back(mit_command_header);
        frame.push_back(command.id);

        // Data length
        frame.push_back(0x08); // 8 bytes of payload

        // data payload
        frame.insert(frame.end(), payload_bytes.begin(), payload_bytes.end());

        // Add footer
        frame.push_back(SANPO_FRAME_TAIL[0]);
        frame.push_back(SANPO_FRAME_TAIL[1]);

    }

    // form the full SANPO frame given the payload_bytes, motor ID and can channel
    static void form_sanpo_frame(std::vector<uint8_t>& frame, const std::array<uint8_t, 8>& payload_bytes, uint8_t motor_id, const uint8_t mit_command_header, const uint8_t can_channel) {
        
        frame.reserve(18); //preallocate space for 18 bytes for sanpo frame

        // Add header
        frame.push_back(SANPO_FRAME_HEADER[0]);
        frame.push_back(SANPO_FRAME_HEADER[1]);

        // Add motor ID
        frame.push_back(can_channel); // CAN channel 

        //reserved 2 bytes
        frame.push_back(0x00);
        frame.push_back(0x00);

        // MIT command (header + motor_id)
        frame.push_back(mit_command_header);
        frame.push_back(motor_id);

        // Data length
        frame.push_back(0x08); // 8 bytes of payload

        // data payload
        frame.insert(frame.end(), payload_bytes.begin(), payload_bytes.end());

        // Add footer
        frame.push_back(SANPO_FRAME_TAIL[0]);
        frame.push_back(SANPO_FRAME_TAIL[1]);

    }

    // get the motor state from the SANPO response frame
    static void get_motor_state_from_sanpo_frame(MotorState& state, const std::vector<uint8_t>& sanpo_response_frame, const uint8_t motor_id) {

        // motor_id
        state.id = motor_id;

        //extract raw integers from the frame
        uint32_t pos_int = (sanpo_response_frame[9] << 8) | sanpo_response_frame[10];                     // 16 bits
        uint32_t vel_int = (sanpo_response_frame[11] << 4) | (sanpo_response_frame[12] >> 4);             // 12 bits
        uint32_t tau_int = ((sanpo_response_frame[12] & 0x0F) << 8) | sanpo_response_frame[13]; 
        
        // convert raw integers back to physical floats
        state.raw_position = uint_to_float(pos_int, P_MIN, P_MAX, 16);
        state.velocity = uint_to_float(vel_int, V_MIN, V_MAX, 12);
        state.torque = uint_to_float(tau_int, T_MIN, T_MAX, 12); 
        
        // compute filtered position
        // state.filtered_position = (state.raw_position + M_PI) % (2 * M_PI) - M_PI; // wrap to [-pi, pi]
        const float two_pi = 2.0f * static_cast<float>(M_PI);
        state.filtered_position = std::remainder(state.raw_position, two_pi);
    }

    // extract the valid 18 bytes SANPO frame from the raw rx buffer/response
    static void unpack_response_and_get_motor_states(std::vector<MotorState>& motor_states, std::vector<uint8_t>& response, const std::vector<uint8_t>& motor_id) {
        // logging for debug
        SPDLOG_DEBUG("received buffer response size: {} bytes: ", response.size());
        
            if (response.size() < SANPO_FRAME_SIZE) {
            SPDLOG_WARN("Response size {} bytes is smaller than SANPO frame size, cannot extract motor states.", response.size());
            return;
        }

        // track which motor IDs have been found in the response
        std::unordered_map<int, bool> motor_id_frame_received;
        for (int id : motor_id) {
            motor_id_frame_received[id] = false;
        }


        while (true) {

            // break the loop if all the motor IDs have been found in the response
            bool all_frames_received = std::all_of(motor_id_frame_received.begin(), motor_id_frame_received.end(), [](const auto& pair) {
                return pair.second;
            });
            if (all_frames_received) break; 

            // break the loop if there is not enough data left in the buffer for a full frame
            if (response.size() < SANPO_FRAME_SIZE) {
                SPDLOG_WARN("invalid response length");
                break;
            }

            // check last occurence of tail frame 
            auto tail_it = std::find_end(response.begin(), response.end(), SANPO_FRAME_TAIL.begin(), SANPO_FRAME_TAIL.end());
            if (tail_it == response.end()) break;

            // check if there is enough data before the tail for a full frame or else exit the loop
            size_t response_length_till_tail = std::distance(response.begin(), tail_it); //SNAPO frame is 18 bytes long.. the tail contains 2 bytes, so the start of the frame is 16 bytes before the tail
            if (response_length_till_tail < (SANPO_FRAME_SIZE - 2)) {
                break; 
            }

            // check for header frame. if header does not match, remove the last tail and continue searching for the next tail
            size_t frame_start = response_length_till_tail - (SANPO_FRAME_SIZE - 2);
            if (response[frame_start] != SANPO_FRAME_HEADER[0] || response[frame_start + 1] != SANPO_FRAME_HEADER[1]) {
                response.resize(response_length_till_tail);
                continue; 
            }

            // extract the frame
            std::vector<uint8_t>sanpo_response_frame(response.begin() + frame_start, response.begin() + frame_start + SANPO_FRAME_SIZE);

            // check the rx code of the mit protocol. 
            // if the received motor id does not match, remove the last frame and continue searching for the next frame
            uint8_t rx_mit_header = ((response[frame_start + 5] & 0x0F) << 4) |
                                ((response[frame_start + 6] & 0xF0) >> 4);
            if (rx_mit_header != MIT_RX_HEADER) {
                response.resize(frame_start); // remove the last frame from the response and continue searching for the next frame
                continue;
            }

            // extract the motor id from the frame- last 4 bits of byte at index 6
            uint8_t rx_motor_id = response[frame_start + 6] & 0x0F;
            // uint8_t rx_motor_id = sanpo_response_frame[6] & 0x0F;
            // SPDLOG_DEBUG("Extracted motor ID from frame: {}", rx_motor_id);

            // check for motor_id that has already been received in the response
            auto map_it = motor_id_frame_received.find(rx_motor_id);
            if (map_it != motor_id_frame_received.end() && !map_it->second) {
                SPDLOG_DEBUG("Extracted frame: {:02x}", fmt::join(sanpo_response_frame, " "));
                MotorState received_motor_state;
                get_motor_state_from_sanpo_frame(received_motor_state, sanpo_response_frame, rx_motor_id);
                motor_states.push_back(received_motor_state);
                motor_id_frame_received[rx_motor_id] = true; // mark this motor ID as received
            }
            else {
                // check the another frame
                response.resize(frame_start); // remove the last frame from the response and continue searching for the next frame
            }
        }
    }
};


class SanpoHardwareInterface {
public:
    // constructor
    SanpoHardwareInterface(const std::string& port_name, 
                        const uint32_t baud_rate, 
                        const uint8_t& can_channel, 
                        const std::vector<uint8_t>& motor_ids, 
                        std::unordered_map<uint8_t, MotorState>& motor_states) : motor_states_(motor_states) 
        {

        sanpo_port_name_ = port_name;
        sanpo_baud_rate_ = baud_rate;
        sanpo_can_channel_ = can_channel;
        motor_ids_ = motor_ids;

        // reserve the space for the buffer response 
        buffer_response_.reserve(BUFFER_SIZE); // reserve space for 1024 bytes,

    }

    // connect to the serial port
    bool connect_sanpo() {
        try {
            sanpo_serial_port_.setPort(sanpo_port_name_);
            sanpo_serial_port_.setBaudrate(sanpo_baud_rate_);
            
            // Timeout: Wait up to 1000ms for a response line
            serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
            sanpo_serial_port_.setTimeout(time_out);

            // open the serial port
            sanpo_serial_port_.open();
        }
        catch (const std::exception& e) {
            SPDLOG_ERROR("Exception while connecting to serial port: {}", e.what());
            return false;
        }

        // check if the port is open
        if (!sanpo_serial_port_.isOpen()) {
            SPDLOG_ERROR("Failed to open serial port: {}", sanpo_port_name_);
            return false;
        }

        // performing handshake two times:
        //- 1st time to clear the stale data sent by the microcontroller. 
        //- 2nd time to check if the connection is successful by checking the response for the handshake frame
        for (int i = 0; i < 2; ++i) {
            flush_tx_rx_buffers(); // clear both tx and rx buffers to start fresh
            
            // send handshake frame to check if the connection is successful
            send_tx_frame(std::vector<uint8_t>(handshake_frame.begin(), handshake_frame.end()));
            
            // get response for handshake frame
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // wait for response to arrive
            get_rx_response(); 
        }


        if (buffer_response_.empty()) {
            SPDLOG_ERROR("No response received for handshake frame, connection may have failed.");
            return false;
        }

        // check for "OK" response  in the handshake response to confirm successful connection
        SPDLOG_DEBUG("Handshake response size {} bytes", buffer_response_.size());
        std::string handshake_response_str(buffer_response_.begin(), buffer_response_.end());       // convert response bytes to string for easier checking
        if (handshake_response_str.find("OK") != std::string::npos) {
            SPDLOG_INFO("Successfully connected to serial port: {}", sanpo_port_name_);
            // clean the buffer after successful handshake
            sanpo_serial_port_.flush(); // clear both tx and rx buffers to start fresh
            buffer_response_.clear();        // Reset the C++ buffer
            return true;
        } 
        else {
            SPDLOG_ERROR("Handshake failed on the port {}, received response: {}", sanpo_port_name_, handshake_response_str);
            // close the port if handshake failed
            sanpo_serial_port_.close();
            return false;
        }
    
    }

    // disconnect from the serial port
    bool disconnect_sanpo_and_motors() {
        // disable motors before disconnecting
        disable_motors();

        if (sanpo_serial_port_.isOpen()) {
            flush_tx_rx_buffers(); // clear both tx and rx buffers to ensure all pending data is sent and received before closing the port
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for flush to complete
            sanpo_serial_port_.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for flush to complete
            SPDLOG_INFO("Disconnected from serial port: {}", sanpo_port_name_);
            return true;
        }
        else {
            SPDLOG_WARN("Serial port {} is already closed.", sanpo_port_name_);
            return false;
        }
    }

    // enable motors
    std::vector<uint8_t> enable_motors() {
        std::vector<uint8_t> enabled_motors;

        for (auto motor_id : motor_ids_) {
            SPDLOG_INFO("Enabling motor {}...", motor_id);
            sanpo_serial_port_.flush(); // clear both tx and rx buffers to start fresh
           
            // form the enable command sanpo frame for this motor ID
            std::vector<uint8_t> enable_frame;
            SanpoProtocolBuilder::form_sanpo_frame(enable_frame, ENABLE_MOTOR_BYTES,  motor_id, MIT_SET_HEADER, sanpo_can_channel_);

            // send the enable command frame
            send_tx_frame(enable_frame);

            // wait for response 
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // DEBUG

            // get the response for the enable command
            std::vector<MotorState> received_motor_states;
            get_motor_states_from_rx_response(received_motor_states);

            // skip the loop if the received states are empty
            if (received_motor_states.empty()) {
                continue;
            }

            // check for motor id if the received motor states is not empty
            for (const auto& motor_state : received_motor_states) {
                if (motor_state.id == motor_id) {
                    enabled_motors.push_back(motor_id);
                    break;
                }
            }

            // update the motor states
            update_motor_states(received_motor_states);
            
        }

        // check if all motors are enabled successfully
        if (enabled_motors.size() == motor_ids_.size()) {
            SPDLOG_INFO("All motors enabled successfully.");
            // clear both tx and rx buffers 
            flush_tx_rx_buffers(); 
            return {};
        }
        else {
            std::vector<uint8_t> failed_motors;
            for (auto motor_id : motor_ids_) {
                if (std::find(enabled_motors.begin(), enabled_motors.end(), motor_id) == enabled_motors.end()) {
                    failed_motors.push_back(motor_id);
                }
            }   
            return failed_motors;
        }
    }

    void flush_tx_rx_buffers() {
        sanpo_serial_port_.flush(); // clear both tx and rx buffers to start fresh
        buffer_response_.clear();        // Reset the C++ buffer
    }

    // disable motors
    void disable_motors() {
        for (auto motor_id : motor_ids_) {
            // form the disable command sanpo frame for this motor ID
            std::vector<uint8_t> disable_frame;
            SanpoProtocolBuilder::form_sanpo_frame(disable_frame, DISABLE_MOTOR_BYTES,  motor_id, MIT_SET_HEADER, sanpo_can_channel_);
            // send the disable command frame
            send_tx_frame(disable_frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); 

            // logging for debug
            SPDLOG_INFO("Sent disable command for motor {}", motor_id);
        }
    }

    void set_zero_position() {
        for (auto motor_id : motor_ids_) {
            sanpo_serial_port_.flush(); // clear both tx and rx buffers to start fresh

            // form the zero position command sanpo frame for this motor ID
            std::vector<uint8_t> set_zero_pos_frame;
            SanpoProtocolBuilder::form_sanpo_frame(set_zero_pos_frame, ZERO_POS_BYTES,  motor_id, MIT_SET_HEADER, sanpo_can_channel_);

            // send the zero position command frame
            send_tx_frame(set_zero_pos_frame);

            // wait for response
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // DEBUG

            // update motor states
            update_motor_states();
        }
    }

    void write_motor_commands(const std::vector<MotorCommand>& motor_commands) {
        // flush the input buffer before sending new commands to avoid mixing old responses with new ones
        std::vector<std::vector<uint8_t>> sanpo_frames;

        for (auto& command : motor_commands) {
            // form the move command sanpo frame for this motor ID
            std::vector<uint8_t> move_frame;
            SanpoProtocolBuilder::form_sanpo_frame(move_frame, command, MIT_MOVE_HEADER, sanpo_can_channel_);
            sanpo_frames.push_back(move_frame);
        }

        // send all the move command frames -one by one
        for (size_t i = 0; i < motor_commands.size(); ++i) {
            send_tx_frame(sanpo_frames[i]);
            SPDLOG_DEBUG("Sent sanpo frame for motor {}: {:02x}", motor_commands[i].id, fmt::join(sanpo_frames[i], " "));
        }
    }

    void update_motor_states() {
        std::vector<MotorState> received_motor_states;
        get_motor_states_from_rx_response(received_motor_states);

        if (received_motor_states.empty()) {
            SPDLOG_WARN("No valid SANPO response frame found in the response when trying to update motor states.");
            return;
        }

        for (const auto& state : received_motor_states) {
            motor_states_[state.id] = state; // update the latest state for this motor ID
        }
    }

    void update_motor_states(std::vector<MotorState>& received_motor_states) {

        for (const auto& state : received_motor_states) {
            motor_states_[state.id] = state; // update the latest state for this motor ID
        }
    }

private:
    void send_tx_frame(const std::vector<uint8_t>& tx_frame) {
        sanpo_serial_port_.write(tx_frame);

        // log for debug
        SPDLOG_DEBUG("Sent TX frame: {:02x}", fmt::join(tx_frame, " "));
    }

    void get_rx_response(){
        size_t bytes_available = sanpo_serial_port_.available();

        if (bytes_available > 0) {
            std::vector<uint8_t> rx_response;
            sanpo_serial_port_.read(rx_response, bytes_available);
            SPDLOG_DEBUG("rx response bytes size: {} bytes", rx_response.size());
            buffer_response_.insert(buffer_response_.end(), rx_response.begin(), rx_response.end());
            
        }

        if (buffer_response_.size() > BUFFER_SIZE) {
            // keep only the last BUFFER_SIZE bytes in the buffer to prevent it from growing indefinitely
            buffer_response_.erase(buffer_response_.begin(), buffer_response_.end() - bytes_to_keep);
        }
    }

    void  get_motor_states_from_rx_response(std::vector<MotorState>& motor_states) {
        get_rx_response();
        
        if (buffer_response_.empty()) {
            return;
        }

        SanpoProtocolBuilder::unpack_response_and_get_motor_states(motor_states, buffer_response_, motor_ids_);
    }


    // global variables
    serial::Serial sanpo_serial_port_;
    std::string sanpo_port_name_;
    uint32_t sanpo_baud_rate_;
    uint8_t sanpo_can_channel_;
    std::vector<uint8_t> motor_ids_;
    std::unordered_map<uint8_t, MotorState>& motor_states_; // map of motor ID to its latest state
    std::vector<uint8_t> buffer_response_; // buffer to store raw response bytes read from the serial port
};
