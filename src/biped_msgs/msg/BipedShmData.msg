# message for Shared memeory data

std_msgs/Header header

# IMU data from FSM
float64[3] imu_orientation      # [roll, pitch, yaw]
float64[3] imu_angular_velocity # [wx, wy, xz]

# joystick cmds
geometry_msgs/Twist joy_cmd

# robot state
string robot_state


