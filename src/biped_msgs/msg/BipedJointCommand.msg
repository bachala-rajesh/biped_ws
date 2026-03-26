# this message holds the joint names, positions, velocity, effort, Kp, Kd

std_msgs/Header header
string frame_id
string[] joint_name
float64[] position
float64[] velocity
float64[] effort
float64[] kp
float64[] kd