"""Shared IMU data structure for iceoryx2 (zero-copy)."""

import ctypes

class IoxImuData(ctypes.Structure):
    """
    Flat structure for zero-copy shared memory.
    """
    
    _fields_ = [
         
        # Euler angles (degrees)
        ("roll", ctypes.c_double),
        ("pitch", ctypes.c_double),
        ("yaw", ctypes.c_double),
        
    ]
    
    def __str__(self):
        return (f"IoxImuData: "
                f"RPY=({self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f})")

    @staticmethod
    def type_name():
        return "IoxImuData"