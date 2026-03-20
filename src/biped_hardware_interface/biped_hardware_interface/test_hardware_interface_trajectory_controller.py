#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryTestClient(Node):
    def __init__(self):
        super().__init__('trajectory_test_client')
        
        # Create an action client for the trajectory controller
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        self.get_logger().info('Waiting for trajectory controller server...')
        self._action_client.wait_for_server()

        # 1. Define the joints we want to move
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_knee_joint',
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_knee_joint'
        ]

        # 2. Create a single waypoint (target position)
        point = JointTrajectoryPoint()
        
        # Move all 6 joints to 0.1 radians (~5.7 degrees)
        point.positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        
        # Tell the joints to stop moving when they reach the target
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 0.0 extra effort (let the motor's MIT KP/KD handle the push)
        point.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

        # 3. Tell the robot to take exactly 2.0 seconds to reach this point
        point.time_from_start = Duration(sec=2, nanosec=0)

        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending safe trajectory goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by controller.')
            return

        self.get_logger().info('Goal accepted! Robot should be moving smoothly.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Trajectory finished with error code: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryTestClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()