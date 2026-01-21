#ifndef BIPED_HARDWARE_INTERFACE_MICROCONTROLLER_COMMS_HPP_
#define BIPED_HARDWARE_INTERFACE_MICROCONTROLLER_COMMS_HPP_

#include <string>
#include <sstream>
#include <vector>
#include <serial/serial.h>

namespace biped_hardware_interface
{
    class MicrocontrollerComms
    {
    public:
        MicrocontrollerComms() = default;

        // connect to the serial port
        bool connect(const std::string &serial_device,
                    unsigned long baud_rate,
                    unsigned int timeout_ms = 10)
        {
            serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
            serial_conn_.setPort(serial_device);
            serial_conn_.setBaudrate(baud_rate);
            serial_conn_.setTimeout(timeout);

            // open the serial port
            try {
                serial_conn_.open();
            } catch (const std::exception &e) {
                return false;
            }
            return serial_conn_.isOpen();
        }


        // disconnect
        void disconnect()
        {
            if (serial_conn_.isOpen()) {
                serial_conn_.close();
            }
        }

        // check status
        bool isConnected() const
        {
            return serial_conn_.isOpen();
        }

        // send motor commands
        // format : "M,val1,val2,val3\n"
        void send_motor_commands(const std::vector<double> &commands) 
        {
            if (!isConnected()) return;
            
            try {
                std::stringstream ss;
                ss << "M";                       //Motor command prefix
                for (double val : commands) {
                    ss << "," << val;
                }
                ss << "\n";     //End of command

                serial_conn_.write(ss.str());
            } catch (const serial::SerialException &e) {
                std::cerr << "[WRITE ERROR] Failed to send commands: " << e.what() << std::endl;
                // Optional: Trigger a safety stop flag here
            }
            catch (const serial::IOException &e) {
                std::cerr << "[WRITE ERROR] IO Exception: " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "[WRITE ERROR] Unknown error occurred." << std::endl;
            }
        }

        // read joint states but not gurantees that is latest. Return true if successful
        // format : "Header,Pos1,Pos2,Pos3,Vel1,Vel2,Vel3\r"
        bool read_joint_states (std::vector<double> &positions,
                                std::vector<double> &velocities,
                                char expected_header = 'H')
        {
            if (!isConnected() || serial_conn_.available() == 0) return false;

            try {
                // read the line
                std::string response = serial_conn_.readline();

                // validate the response
                if (response.empty() || response[0] != expected_header) return false;

                // parse the response
                // format: Header,Pos1,Pos2,Pos3,Vel1,Vel2,Vel3
                std::stringstream ss(response.substr(2)); // skip 'H,'
                std::string segment;
                std::vector<double> values;

                while (std::getline(ss, segment, ',')) {
                    try {
                        values.push_back(std::stod(segment));
                    } catch (const std::invalid_argument &e) {
                        return false;       // parsing error
                    }
                }

                // ensure we got exactly 6 values (3 positions + 3 velocities)
                if (values.size() != 6) return false;

                // fill the output vectors
                positions.assign(values.begin(), values.begin() + 3);
                velocities.assign(values.begin() + 3, values.end());
            } 
            catch(const serial::IOException &e) {
                std::cerr <<"[Hardware error] USB disconnected: " << e.what() << std::endl;
                return false;
            }
            catch (const std::exception &e) {
                std::cerr << "[SERIAL ERROR] " << e.what() << std::endl;
                return false;
            }
            return true;
        }


        // read latest joint states. Return true if successful
        bool read_latest_joint_states(std::vector<double> &positions,
                            std::vector<double> &velocities,
                            char expected_header = 'H')
        {
            // 1. Connection Check
            if (!isConnected()) return false;

            try {
                // 2. Check for Data
                size_t available_bytes = serial_conn_.available();
                if (available_bytes == 0) return false;

                // 3. Drain the Swamp: Read EVERYTHING currently in the buffer
                std::string buffer = serial_conn_.read(available_bytes);

                // 4. Find the Last Packet End
                // Search backwards for the last newline '\n'
                size_t last_newline = buffer.rfind('\n');

                // edge case: no newline found
                if (last_newline == std::string::npos) {
                    // No newline found at all. We have fragments but no complete packet.
                    return false;
                }

                // edge case: newline is at the start
                if (last_newline == 0) {
                    // This implies the packet is empty or just a newline.
                    return false; 
                }

                // 5. Find the Last Packet Start
                // Search backwards again from the last newline to find the previous one
                size_t second_last_newline = buffer.rfind('\n', last_newline - 1);
                
                std::string latest_packet;
                
                if (second_last_newline == std::string::npos) {
                    // Case A: Only one newline in the whole buffer.
                    // The packet starts at the beginning of the buffer.
                    latest_packet = buffer.substr(0, last_newline);
                } else {
                    // Case B: Multiple newlines found.
                    // The packet is sandwich'd between the last two newlines.
                    // We extract the string *between* them.
                    latest_packet = buffer.substr(second_last_newline + 1, last_newline - second_last_newline - 1);
                }

                // 6. Cleanup & Validation
                // Remove carriage return '\r' if it exists (common in serial)
                if (!latest_packet.empty() && latest_packet.back() == '\r') {
                    latest_packet.pop_back();
                }

                // Check Header (L or R)
                if (latest_packet.empty() || latest_packet[0] != expected_header) {
                    return false;
                }

                // 7. Parse the Numbers
                try {
                    // Skip the header and comma (e.g., "L," is 2 chars)
                    std::stringstream ss(latest_packet.substr(2)); 
                    std::string segment;
                    std::vector<double> values;

                    while (std::getline(ss, segment, ',')) {
                        values.push_back(std::stod(segment));
                    }

                    // We expect exactly 6 values (3 pos + 3 vel)
                    if (values.size() != 6) return false;

                    // Success! Fill the output vectors
                    positions.assign(values.begin(), values.begin() + 3);
                    velocities.assign(values.begin() + 3, values.end());

                    return true;
                } 
                catch (...) {
                    // If stod() fails (garbage characters), just ignore this packet
                    return false;
                }
            }
            catch(const serial::IOException &e) {
                std::cerr <<"[Hardware error] USB disconnected: " << e.what() << std::endl;
                return false;
            }
            catch (const std::exception &e) {
                std::cerr << "[SERIAL ERROR] " << e.what() << std::endl;
                return false;
            }
        }

    private:
        serial::Serial serial_conn_;
    };


} // namespace biped_hardware_interface

#endif // BIPED_HARDWARE_INTERFACE_MICROCONTROLLER_COMMS_HPP_
