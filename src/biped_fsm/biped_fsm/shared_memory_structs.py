import ctypes

# 1. State Block (64 bytes total)
class StateCommand(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("seq_counter", ctypes.c_uint32),  # 4 bytes
        ("current_state", ctypes.c_uint8), # 1 byte
        ("padding", ctypes.c_uint8 * 59)   # 59 bytes of padding (4+1+59 = 64)
    ]

# 2. Velocity Block (64 bytes total)
class VelocityCommand(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("seq_counter", ctypes.c_uint32),  # 4 bytes
        ("linear_x", ctypes.c_float),      # 4 bytes
        ("linear_y", ctypes.c_float),      # 4 bytes
        ("angular_z", ctypes.c_float),     # 4 bytes
        ("padding", ctypes.c_uint8 * 48)   # 48 bytes of padding (16+48 = 64)
    ]

# 3. IMU Block (64 bytes total)
class ImuData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("seq_counter", ctypes.c_uint32),         # 4 bytes
        ("orientation", ctypes.c_float * 3),      # 12 bytes: roll, pitch, yaw
        ("angular_velocity", ctypes.c_float * 3), # 12 bytes: x, y, z
        ("linear_acceleration", ctypes.c_float * 3), # 12 bytes: x, y, z
        ("padding", ctypes.c_uint8 * 24)          # 24 bytes of padding (40+24 = 64)
    ]

# 4. Joint Block (64 bytes total)
class JointData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("seq_counter", ctypes.c_uint32),   # 4 bytes
        ("position", ctypes.c_float * 6),   # 24 bytes: joint positions
        ("velocity", ctypes.c_float * 6),   # 24 bytes: joint velocities
        ("padding", ctypes.c_uint8 * 12)    # 12 bytes of padding (52+12 = 64)
    ]

# Main Memory Block
class BipedSharedMemory(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("state", StateCommand),
        ("cmd_vel", VelocityCommand),
        ("imu", ImuData),
        ("joints", JointData)
    ]