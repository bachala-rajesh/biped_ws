#!/usr/bin/env python3

"""
ROS2 node that subscribes topics and timer.
It is attached to the robot states.
It shares the common blackboard with the robot states and is used to update the state of the robot.
"""

# TODO:
#    - add controller change service to the timer callback
#    - add a state check in the standby where it comes to intial pose in the standby state


import signal
import sys
import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import time
import threading
import math
import os
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np

# yasmin imports
from yasmin import Blackboard
from yasmin_factory import YasminFactory
from yasmin_viewer import YasminViewerPub
from yasmin_ros import set_ros_loggers

import biped_fsm
from multiprocessing import shared_memory, resource_tracker
import ctypes
from biped_fsm import shared_memory_structs
from biped_fsm.robot_states_enum import RobotState


SHUTTING_DOWN = False

# put the value of the joints as per the walking policy
joint_name_to_index = {
    "left_hip_pitch": 0,
    "left_hip_roll": 1,
    "left_knee": 2,
    "right_hip_pitch": 3,
    "right_hip_roll": 4,
    "right_knee": 5,
}


# get path to robot_states.xml

robot_states_xml_path = os.path.join(
    os.path.dirname(biped_fsm.__file__), "fsm_states_transistions.xml"
)


class StateManager(Node):
    """ROS2 node that subscribes to IMU topic and converts quaternion to RPY."""

    def __init__(self, blackboard):
        super().__init__("state_manager_node")

        # callback groups
        self.imu_callback_group = MutuallyExclusiveCallbackGroup()
        self.joy_cmd_callback_group = MutuallyExclusiveCallbackGroup()
        self.joy_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_states_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # imu subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10,
            callback_group=self.imu_callback_group,
        )

        # joy teleop subscriber
        self.joy_cmd_sub = self.create_subscription(
            TwistStamped,
            "/joy_cmd",
            self.joy_cmd_callback,
            10,
            callback_group=self.joy_cmd_callback_group,
        )

        # joy state subscriber
        self.joy_state_sub = self.create_subscription(
            String,
            "/joy_state",
            self.joy_state_callback,
            10,
            callback_group=self.joy_state_callback_group,
        )

        # joint states subscriber
        self.joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            10,
            callback_group=self.joint_states_callback_group,
        )

        # timer
        self.timer = self.create_timer(
            0.1, self.timer_callback, callback_group=self.timer_callback_group
        )  # 10 Hz

        self.blackboard = blackboard

        # Initialize watchdog
        self.blackboard["last_imu_time"] = time.time()
        self.blackboard["imu_roll"] = 0.0
        self.blackboard["imu_pitch"] = 0.0
        self.blackboard["imu_yaw"] = 0.0
        self.blackboard["joy_state_status"] = False
        self.blackboard["joy_cmd_status"] = False
        self.blackboard["imu_status"] = False
        self.blackboard["joint_states_status"] = False

        # variables
        self.imu_data: list[float] = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.joy_cmd_data: list[float] = [
            0.0,
            0.0,
            0.0,
        ]  # linear x, linear y, angular z
        self.joy_state_data: str = (
            "none"  # start, standby, stop, emergency_stop, test_check, none
        )
        self.joint_states_dict: dict[
            str, dict[str, float]
        ] = {}  # joint name: {position, velocity}

        # shared memory
        self.start_shared_memory()

        self.get_logger().info("Supervisor node started...")

    def start_shared_memory(self):
        mem_size = ctypes.sizeof(biped_shared_memory_structs.BipedSharedMemory)

        # Create the memory
        # We use a try-except block in case it already exists from a previous crash
        try:
            self.shm = shared_memory.SharedMemory(
                name="biped_shm", create=True, size=mem_size
            )
            self.get_logger().info("Created new shared memory block: biped_shm")
        except FileExistsError:
            self.shm = shared_memory.SharedMemory(name="biped_shm", create=False)
            self.get_logger().info("Connected to existing shared memory block")

        # Unregister from the resource tracker.
        # This prevents the "leaked shared_memory" warning and prevents
        # Python from deleting the memory if the node restarts.
        resource_tracker.unregister(self.shm._name, "shared_memory")

        # Map the struct
        self.shared_data = biped_shared_memory_structs.BipedSharedMemory.from_buffer(
            self.shm.buf
        )

        # Clear all existing data in the RAM block (sets all bytes to 0)
        ctypes.memset(
            ctypes.addressof(self.shared_data), 0, ctypes.sizeof(self.shared_data)
        )

        # Set the initial state to the init state explicitly
        self.shared_data.state.current_state = int(RobotState.INIT)

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        try:
            roll, pitch, yaw = euler_from_quaternion(orientation_list)

            # convert to degree
            pitch_deg = math.degrees(pitch)
            roll_deg = math.degrees(roll)
            yaw_deg = math.degrees(yaw)

            self.blackboard["last_imu_time"] = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            )
            self.blackboard["imu_roll"] = roll_deg
            self.blackboard["imu_pitch"] = pitch_deg
            self.blackboard["imu_yaw"] = yaw_deg

            self.imu_data = [roll, pitch, yaw]

            self.blackboard["imu_status"] = True

            # Shared Memory Write
            self.shared_data.imu.seq_counter += 1  # LOCK

            # Orientation (radians)
            self.shared_data.imu.orientation[0] = roll
            self.shared_data.imu.orientation[1] = pitch
            self.shared_data.imu.orientation[2] = yaw

            # Angular Velocity
            self.shared_data.imu.angular_velocity[0] = msg.angular_velocity.x
            self.shared_data.imu.angular_velocity[1] = msg.angular_velocity.y
            self.shared_data.imu.angular_velocity[2] = msg.angular_velocity.z

            # Linear Acceleration
            self.shared_data.imu.linear_acceleration[0] = msg.linear_acceleration.x
            self.shared_data.imu.linear_acceleration[1] = msg.linear_acceleration.y
            self.shared_data.imu.linear_acceleration[2] = msg.linear_acceleration.z

            self.shared_data.imu.seq_counter += 1  # UNLOCK

        except Exception as e:
            self.get_logger().error(f"Error converting quaternion to RPY: {e}")
            self.blackboard["imu_status"] = False

    def joy_cmd_callback(self, msg: TwistStamped) -> None:
        self.joy_cmd_data = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
        ]

        # Shared Memory Write
        self.shared_data.cmd_vel.seq_counter += 1  # LOCK

        self.shared_data.cmd_vel.linear_x = msg.twist.linear.x
        self.shared_data.cmd_vel.linear_y = msg.twist.linear.y
        self.shared_data.cmd_vel.angular_z = msg.twist.angular.z

        self.shared_data.cmd_vel.seq_counter += 1  # UNLOCK

    def joy_state_callback(self, msg: String) -> None:
        self.joy_state_data = msg.data

        self.blackboard["joy_state"] = self.joy_state_data
        self.blackboard["joy_state_status"] = True

    def joint_states_callback(self, msg: JointState) -> None:
        global joint_name_to_index

        self.blackboard["last_joint_states_time"] = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

        # create a dictionary
        self.joint_states_dict = {
            name: {"position": position, "velocity": velocity}
            for name, position, velocity in zip(msg.name, msg.position, msg.velocity)
        }

        self.blackboard["joint_states_status"] = True

        # Shared Memory Write
        self.shared_data.joints.seq_counter += 1  # LOCK

        # Match each incoming joint name to our strict memory array index
        for i, name in enumerate(msg.name):
            if name in joint_name_to_index:
                idx = joint_name_to_index[name]
                self.shared_data.joints.position[idx] = msg.position[i]
                self.shared_data.joints.velocity[idx] = msg.velocity[i]

        self.shared_data.joints.seq_counter += 1  # UNLOCK

    def timer_callback(self) -> None:
        global SHUTTING_DOWN
        # If the shutdown loop has started, ABORT this callback immediately
        if SHUTTING_DOWN:
            return
        
        # fake data and calls for testing
        self.fake_data_and_calls()

        # get robot state from blackboard
        state = self.blackboard["robot_state"]

        # We use RobotState[state_str] to look up the enum by its name
        try:
            current_enum_val = RobotState[state]
        except KeyError:
            current_enum_val = RobotState.ERROR
            self.get_logger().error(f"Unknown state string: {state}")

        # 3. Write to shared memory with the Sequence Lock
        self.shared_data.state.seq_counter += 1
        self.shared_data.state.current_state = int(current_enum_val)
        self.shared_data.state.seq_counter += 1

        # write these data to the shared memory
        return

    def destroy_node(self):
        # Gracefully disconnect from shared memory without deleting it
        if hasattr(self, "shm"):
            self.shm.close()
            # self.shm.unlink()
            self.get_logger().info("Disconnected from shared memory safely.")
        super().destroy_node()

    def fake_data_and_calls(self):
        # for testing use the fake data
        imu_msg = Imu()
        imu_msg.header.stamp.sec = int(time.time())
        imu_msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        joint_states_msg = JointState()
        joint_states_msg.header.stamp.sec = int(time.time())
        joint_states_msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        joint_states_msg.name = [
            "left_hip_pitch",
            "left_hip_roll",
            "left_knee",
            "right_hip_pitch",
            "right_hip_roll",
            "right_knee",
        ]

        joint_states_msg.position = np.random.uniform(-1.0, 1.0, size=6).tolist()
        joint_states_msg.velocity = np.random.uniform(-0.1, 0.1, size=6).tolist()

        # joy_cmd_msg = TwistStamped()
        # joy_cmd_msg.twist.linear.x = np.random.uniform(-1.0, 1.0)
        # joy_cmd_msg.twist.linear.y = np.random.uniform(-1.0, 1.0)
        # joy_cmd_msg.twist.angular.z = np.random.uniform(-0.1, 0.1)
        
        # joy_state_msg = String()
        # joy_state_msg.data = "standby"

        # call the callbacks with fake data to test the shared memory writing
        self.imu_callback(imu_msg)
        # self.joy_cmd_callback(joy_cmd_msg)
        self.joint_states_callback(joint_states_msg)
        # self.joy_state_callback(joy_state_msg)
        # ------------------------------


def main(args=None):
    rclpy.init(args=args)
    set_ros_loggers()

    # intiiliaze blackboard
    blackboard = Blackboard()
    blackboard.set("last_imu_time", time.time())
    blackboard.set("last_joint_states_time", time.time())
    blackboard.set("imu_roll", 0.0)
    blackboard.set("imu_pitch", 0.0)
    blackboard.set("imu_yaw", 0.0)
    blackboard.set("imu_status", False)
    blackboard.set("joy_state_status", False)
    blackboard.set("joy_cmd_status", False)
    blackboard.set("joint_states_status", False)
    blackboard.set("joy_state", "none")
    blackboard.set("robot_state", "none")

    node = StateManager(blackboard)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # create FSM from xml
    factory = YasminFactory()
    state_machine = factory.create_sm_from_file(robot_states_xml_path)
    state_machine.set_sigint_handler(False)

    # publish FSM to viewer
    YasminViewerPub(state_machine, "robot_states")

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()


    # 2. Define an unstoppable Python signal interrupt
    def hard_shutdown(sig, frame):
        global SHUTTING_DOWN
        SHUTTING_DOWN = True # Lock out the background timer immediately
        
        print("\n[EMERGENCY] Ctrl+C detected! Executing hardware E-STOP...")
        
        if hasattr(node, 'shm'):
            # Inject the STOP command directly into RAM
            print("[EMERGENCY] Shared memory set to STOP. Motors should power down.")
            for _ in range (5):
                node.shared_data.state.seq_counter += 1
                node.shared_data.state.current_state = int(RobotState.STOP)
                node.shared_data.state.seq_counter += 1
                time.sleep(0.05)

            print("[EMERGENCY] STOP command broadcasted redundantly.")
            if hasattr(node, 'shared_data'):
                del node.shared_data
                
            node.shm.close()
            print("[EMERGENCY] Shared memory disconnected safely.")
            
            
        #  Assassinate the Python process
        print("[EMERGENCY] Terminating Python bridge.")
        os._exit(0)
    # 3. Tell the OS to trigger our hard_shutdown function when Ctrl+C is pressed
    signal.signal(signal.SIGINT, hard_shutdown)

    # Now run the FSM
    try:
        outcome = state_machine(blackboard)
    except Exception as e:
        print(f"FSM exited with error: {e}")
    finally:
        # This will only run if the FSM finishes naturally, not on Ctrl+C
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            


if __name__ == "__main__":
    main()
