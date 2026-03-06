#include <serial/serial.h>
#include <iostream>
#include <string>

int main()
{
    // --- CONFIGURATION ---
    std::string port = "/dev/ttyUSB0"; // CHECK YOUR PORT (ls /dev/tty*)
    unsigned long baud = 115200;

    serial::Serial my_serial;

    try {
        std::cout << "Attempting to open " << port << "..." << std::endl;
        my_serial.setPort(port);
        my_serial.setBaudrate(baud);
        
        // Timeout: Wait up to 1000ms for a response line
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        
        my_serial.open();

        if (!my_serial.isOpen()) {
            std::cerr << "[ERROR] Port not opened." << std::endl;
            return 1;
        }

        std::cout << "--- CONNECTED ---" << std::endl;
        std::cout << "Enter a number to multiply by 2." << std::endl;
        std::cout << "Type 'q' to quit." << std::endl;

        // --- MAIN LOOP ---
        while (true) {
            std::cout << "\n> Enter Number: ";
            std::string user_input;
            std::cin >> user_input;

            // Exit condition
            if (user_input == "q") break;

            // 1. Send number to ESP32 (add \n so ESP32 knows it's the end)
            my_serial.write(user_input + "\n");

            // 2. Read the response (readline waits for the '\n' from ESP32)
            std::string response = my_serial.readline();

            // 3. Print result
            if (response.empty()) {
                std::cout << "[TIMEOUT] ESP32 did not reply." << std::endl;
            } else {
                std::cout << "[ESP32 Reply]: " << response; 
                // Note: response already contains a newline from Serial.println
            }
        }

        my_serial.close();

    } catch (std::exception &e) {
        std::cerr << "[EXCEPTION] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}