#!/usr/bin/env python3


######### ToDo
# - fix convert quaternion to rpy
# - fix zero default joint positions- think of alternative rather than hardcoding
# - put proper scaling values. refer the author's code
# - implement policy loading and action inference using torch


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist

# import torch
import threading
import numpy as np
from collections import deque


class PolicyRunnerNode(Node):
    def __init__(self):
        super().__init__("policy_runner_node")

        # variables
        self.dt = 0.1
        self.history_len = 9
        self.joint_pos_scale = 1.0
        self.joint_vel_scale = 1.0
        self.lin_vel_x_scale = 1.0
        self.lin_vel_y_scale = 1.0
        self.ang_vel_scale = 1.0
        self.current_joints_pos = np.zeros(6)  # angles of joints
        self.current_joints_vel = np.zeros(6)  # angle velocities of joints
        self.current_rpy = np.zeros(3)  # roll, pitch, yaw
        self.current_ang_vel = np.zeros(3)  # angular velocity x, y, z
        self.commanded_vel = np.zeros(3)  # linear x, linear y, angular z
        self.last_action_taken = np.zeros(6)  # last action taken

        self.default_joints_pos = np.zeros(6)  # default joint positions

        # history deques
        self.hist_joints_pos = deque(
            [np.zeros(6) for _ in range(self.history_len)], maxlen=self.history_len
        )
        self.hist_joints_vel = deque(
            [np.zeros(6) for _ in range(self.history_len)], maxlen=self.history_len
        )
        self.hist_rpy = deque(
            [np.zeros(3) for _ in range(self.history_len)], maxlen=self.history_len
        )
        self.hist_ang_vel = deque(
            [np.zeros(3) for _ in range(self.history_len)], maxlen=self.history_len
        )
        self.hist_actions = deque(
            [np.zeros(6) for _ in range(self.history_len)], maxlen=self.history_len
        )

        # parameters
        self.declare_parameter("policy_model_path", "path/to/policy_model.pt")
        self.declare_parameter("joint_pos_scale", 1.0)
        self.declare_parameter("joint_vel_scale", 1.0)
        self.declare_parameter("lin_vel_x_scale", 1.0)
        self.declare_parameter("lin_vel_y_scale", 1.0)
        self.declare_parameter("ang_vel_scale", 1.0)
        self.declare_parameter("history_len", 9)
        self.declare_parameter("dt", 0.1)

        # get the parameters
        self.get_parameters()

        ###callback groups
        self.sensors_callback_group = ReentrantCallbackGroup()
        self.policy_callback_group = MutuallyExclusiveCallbackGroup()

        ### topics
        # subscirbe to joint states
        self.joint_state_subscriber = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self.joint_state_callback,
            qos_profile=10,
            callback_group=self.sensors_callback_group,
        )
        # subscribe to imu data
        self.imu_subscriber = self.create_subscription(
            msg_type=Imu,
            topic="/imu",
            callback=self.imu_callback,
            qos_profile=10,
            callback_group=self.sensors_callback_group,
        )

        # subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self.cmd_vel_callback,
            qos_profile=10,
            callback_group=self.policy_callback_group,
        )

        # subcribe to diagnostics
        self.diagnostics_subscriber = self.create_subscription(
            msg_type=Int32,
            topic="/diagnostics",
            callback=self.diagnostics_callback,
            qos_profile=10,
            callback_group=self.sensors_callback_group,
        )

        # timer
        self.timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.timer_callback,
            callback_group=self.policy_callback_group,
        )

        # publisher
        self.joint_command_publisher = self.create_publisher(
            msg_type=JointState,
            topic="/joint_commands",
            qos_profile=10,
            callback_group=self.policy_callback_group,
        )

        # thread lock
        self.joint_state_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        self.cmd_vel_lock = threading.Lock()

        # logging
        self.get_logger().info("Biped RL policy runner node has been started.")

    def get_parameters(self):
        self.policy_model_path = (
            self.get_parameter("policy_model_path").get_parameter_value().string_value
        )
        self.joint_pos_scale = (
            self.get_parameter("joint_pos_scale").get_parameter_value().double_value
        )
        self.joint_vel_scale = (
            self.get_parameter("joint_vel_scale").get_parameter_value().double_value
        )
        self.lin_vel_x_scale = (
            self.get_parameter("lin_vel_x_scale").get_parameter_value().double_value
        )
        self.lin_vel_y_scale = (
            self.get_parameter("lin_vel_y_scale").get_parameter_value().double_value
        )
        self.ang_vel_scale = (
            self.get_parameter("ang_vel_scale").get_parameter_value().double_value
        )
        self.history_len = (
            self.get_parameter("history_len").get_parameter_value().integer_value
        )
        self.dt = self.get_parameter("dt").get_parameter_value().double_value

    def joint_state_callback(self, msg: JointState):
        # Process joint state data
        with self.joint_state_lock:
            self.current_joints_pos = np.array(msg.position)
            self.current_joints_vel = np.array(msg.velocity)

    def imu_callback(self, msg: Imu):
        # Process IMU data
        with self.imu_lock:
            # Convert quaternion to roll, pitch, yaw
            q = msg.orientation
            # Placeholder conversion, replace with actual conversion
            self.current_rpy = np.array(
                [0.0, 0.0, 0.0]
            )  # Replace with actual conversion
            self.current_ang_vel = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            )

    def cmd_vel_callback(self, msg: Twist):
        # Process cmd_vel data
        with self.cmd_vel_lock:
            self.commanded_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def diagnostics_callback(self, msg: Int32):
        # Process diagnostics data
        self.get_logger().info(f"Received Diagnostics data: {msg.data}")
        pass

    def timer_callback(self):
        self.form_obs_group()

    def form_obs_group(self):
        """
        Structure: Current(t) + History(t-1...t-9)
        """
        obs_list = []

        # --- 1. Thread-Safe Read & Normalize ---
        with self.joint_state_lock:
            # Position: (Real - Default) * Scale
            norm_pos = (
                self.current_joints_pos - self.default_joints_pos
            ) * self.joint_pos_scale
            # Velocity: Real * Scale
            norm_vel = self.current_joints_vel * self.joint_vel_scale

        with self.imu_lock:
            raw_rpy = self.current_rpy.copy()
            # Gyro: Real * Scale
            norm_ang_vel = self.current_ang_vel * self.ang_vel_scale

        with self.cmd_vel_lock:
            cmds = self.commanded_vel.copy()
            cmds[0] *= self.lin_vel_x_scale  # lin_x
            cmds[1] *= self.lin_vel_y_scale  # lin_y
            cmds[2] *= self.ang_vel_scale  # ang_z

        # --- 2. Build Observation Vector ---
        # A. Euler Angles (RPY) [Current + 9 History]
        obs_list.extend(raw_rpy)  # Current
        for h in self.hist_rpy:
            obs_list.extend(h)  # History

        # B. Angular Velocity [Current + 9 History]
        obs_list.extend(norm_ang_vel)  # Current
        for h in self.hist_ang_vel:
            obs_list.extend(h)  # History

        # C. Commands [Current Only]
        obs_list.extend(cmds)

        # D. Joint Positions [Current + 9 History]
        obs_list.extend(norm_pos)  # Current
        for h in self.hist_joints_pos:
            obs_list.extend(h)  # History

        # E. Joint Velocities [Current + 9 History]
        obs_list.extend(norm_vel)  # Current
        for h in self.hist_joints_vel:
            obs_list.extend(h)  # History

        # F. Last Actions [Previous Step]
        obs_list.extend(self.hist_actions[0])  # Last action only

        # --- 3. Update History (The "Shift") ---
        self.hist_rpy.appendleft(raw_rpy)
        self.hist_ang_vel.appendleft(norm_ang_vel)
        self.hist_joints_pos.appendleft(norm_pos)
        self.hist_joints_vel.appendleft(norm_vel)
        self.hist_actions.appendleft(self.last_action_taken)

        # --- Debug Verification ---
        # Total size should be 189
        # Calculation:
        # RPY(30) + AngVel(30) + Cmd(3) + Pos(60) + Vel(60) + Act(6) = 189
        if len(obs_list) != 189:
            self.get_logger().error(
                f"Observation size mismatch! Expected 189, got {len(obs_list)}"
            )
        else:
            # Just print once to confirm, then comment out to avoid spam
            self.get_logger().info(f"Observation Vector Built. Size: {len(obs_list)}")
            pass

        return obs_list


def main(args=None):
    rclpy.init(args=args)
    biped_rl_control_node = PolicyRunnerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(biped_rl_control_node)
    try:
        executor.spin()
    finally:
        biped_rl_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
