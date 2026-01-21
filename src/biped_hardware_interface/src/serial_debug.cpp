#include <serial/serial.h>
#include <iostream>
#include <string>
#include <unistd.h> // for sleep

int main(int argc, char **argv)
{
    // --- CONFIGURATION ---
    // Allow user to pass port as argument, default to /dev/ttyUSB0
    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    
    // IMPORTANT: Match this to your ESP32 code! 
    unsigned long baud = 1000000; 

    serial::Serial my_serial;

    try {
        std::cout << "Attempting to open " << port << " at " << baud << " baud..." << std::endl;
        my_serial.setPort(port);
        my_serial.setBaudrate(baud);
        
        // Timeout: 100ms block
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        my_serial.setTimeout(to);
        
        my_serial.open();

        if (!my_serial.isOpen()) {
            std::cerr << "[ERROR] Port not opened." << std::endl;
            return 1;
        }

        std::cout << "--- CONNECTED ---" << std::endl;
        std::cout << "Listening for data... (Ctrl+C to quit)" << std::endl;

        // Flush old garbage
        my_serial.flushInput();

        // --- MAIN LOOP ---
        while (true) {
            // Check if data is available
            if (my_serial.available()) {
                // Read a full line (blocks until \n or timeout)
                std::string response = my_serial.readline();

                // Clean up whitespace for display
                std::string clean_response = response;
                if (!clean_response.empty() && clean_response.back() == '\n') clean_response.pop_back();
                if (!clean_response.empty() && clean_response.back() == '\r') clean_response.pop_back();

                if (clean_response.empty()) {
                    // Empty lines might happen if we time out partway
                    continue; 
                }

                std::cout << "[RAW]: " << clean_response << std::endl;
            } 
            else {
                // Sleep briefly to save CPU if no data
                usleep(1000); 
            }
        }

        my_serial.close();

    } catch (std::exception &e) {
        std::cerr << "[EXCEPTION] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}