#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time

class SineWaveTester(Node):
    def __init__(self):
        super().__init__('sine_wave_tester')
        
        # The exact order must match the array in biped_controllers.yaml
        self.joint_names = [
            'left_hip_pitch_joint', 
            'left_hip_roll_joint', 
            'left_knee_joint',
            'right_hip_pitch_joint', 
            'right_hip_roll_joint', 
            'right_knee_joint'
        ]
        
        self.initial_positions = []
        self.is_initialized = False
        
        # Publisher to the controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/joints_position_controller/commands', 
            10
        )
        
        # Subscriber to read the safe starting posture
        self.subscription = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.state_callback, 
            10
        )
        
        # Motion Parameters
        self.amplitude = 1.57 # Radians (Keep this small for initial safety)
        self.frequency = 0.5 # Hertz (0.5 oscillations per second)
        self.start_time = 0.0
        
        # Run the control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback) 

    def state_callback(self, msg):
        if not self.is_initialized:
            # Map the incoming joint states to their names
            pos_dict = dict(zip(msg.name, msg.position))
            try:
                # Extract initial positions in the correct YAML order
                self.initial_positions = [pos_dict[name] for name in self.joint_names]
                self.is_initialized = True
                self.start_time = time.time()
                self.get_logger().info("Initial hardware positions recorded. Starting safe sine wave...")
            except KeyError:
                # Wait until all joint names are available in the topic
                pass 
        else:
            self.subscription.destroy()

    def timer_callback(self):
        if not self.is_initialized:
            return
            
        msg = Float64MultiArray()
        t = time.time() - self.start_time
        
        # Calculate smooth oscillation offset
        sine_offset = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
        
        # Apply offset to the safe starting position
        msg.data = [pos + sine_offset for pos in self.initial_positions]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SineWaveTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()