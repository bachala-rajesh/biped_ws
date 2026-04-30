"""Shared IMU data structure for iceoryx2 (zero-copy)."""

import ctypes

class IoxImuData(ctypes.Structure):
    """
    Flat structure for zero-copy shared memory.
    """
    
    _fields_ = [
         
        # orientation
        ("orientation_x", ctypes.c_double),
        ("orientation_y", ctypes.c_double),
        ("orientation_z", ctypes.c_double),
        ("orientation_w", ctypes.c_double),
        
        # angular velocity
        ("ang_vel_x", ctypes.c_double),
        ("ang_vel_y", ctypes.c_double),
        ("ang_vel_z", ctypes.c_double),
        
        # linear acceleration
        ("lin_acc_x", ctypes.c_double),
        ("lin_acc_y", ctypes.c_double),
        ("lin_acc_z", ctypes.c_double),
    ]
    
    def __str__(self):
        return (f"IoxImuData: "
                f"orientation=({self.orientation_x:.2f}, {self.orientation_y:.2f}, {self.orientation_z:.2f}, {self.orientation_w:.2f})"
                f"linear_acc=({self.lin_acc_x:.2f}, {self.lin_acc_y:.2f}, {self.lin_acc_z:.2f})"
                f"angular_vel=({self.ang_vel_x:.2f}, {self.ang_vel_y:.2f}, {self.ang_vel_z:.2f})")  

    @staticmethod
    def type_name():
        return "IoxImuData"