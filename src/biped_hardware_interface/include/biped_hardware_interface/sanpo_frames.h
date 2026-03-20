#pragma once


#include <array>
#include <vector>
#include <string>
#include <cstdint>

#define MIT_SET_HEADER 0x00
#define MIT_MOVE_HEADER 0x02
#define MIT_RX_HEADER 0x78

#define SANPO_FRAME_SIZE 18


constexpr std::array<std::uint8_t, 2> SANPO_FRAME_TAIL = {0x0d, 0x0a};
constexpr std::array<std::uint8_t, 2> SANPO_FRAME_HEADER = {0x53, 0x54};
constexpr std::array<std::uint8_t, 8> ENABLE_MOTOR_BYTES {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};  
constexpr std::array<std::uint8_t, 8> DISABLE_MOTOR_BYTES {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};  
constexpr std::array<std::uint8_t, 8> ZERO_POS_BYTES {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};  
inline const std::string handshake_frame = "AT+ET\r\n";

constexpr size_t bytes_to_keep = 360;
constexpr size_t BUFFER_SIZE = 1024; // max buffer size

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

// --- CAN MOTOR INITIAL/DEFAULT DAMPING GAINS ---
constexpr float KD_DEAFULT = 1.0f; // DEAFULT velocity damping gain,
constexpr float KP_DEAFULT = 22.0f; // DEAFULT position stiffness gain, can be tuned later based on the hardware response



