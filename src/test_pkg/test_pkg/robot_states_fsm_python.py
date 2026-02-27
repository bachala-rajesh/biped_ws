#!/usr/bin/env python3

TODO: check the fsm_client_node and switch_client are working correctly...

"""
ROS2 node that subscribes topics and timer.
It is attached to the robot states.
It shares the common blackboard with the robot states and is used to update the state of the robot.
"""

from geometry_msgs.msg import TwistStamped
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
from controller_manager_msgs.srv import SwitchController

# yasmin imports
from yasmin import Blackboard
from yasmin_factory import YasminFactory
from yasmin_viewer import YasminViewerPub
from yasmin_ros import set_ros_loggers


# get path to robot_states.xml
import test_pkg

robot_states_xml_path = os.path.join(
    os.path.dirname(test_pkg.__file__), "robot_states.xml"
)


class StateBridgeNode(Node):
    """ROS2 node that subscribes to IMU topic and converts quaternion to RPY."""

    def __init__(self, blackboard):
        super().__init__("state_bridge_node")

        # callback groups
        self.imu_callback_group = MutuallyExclusiveCallbackGroup()
        self.joy_cmd_callback_group = MutuallyExclusiveCallbackGroup()
        self.joy_state_callback_group = MutuallyExclusiveCallbackGroup()
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

        # timer
        self.timer = self.create_timer(
            1.0, self.timer_callback, callback_group=self.timer_callback_group
        )  # 10 Hz

        self.blackboard = blackboard

        # Initialize watchdog
        self.blackboard["last_imu_time"] = time.time()
        self.blackboard["imu_roll"] = 0.0
        self.blackboard["imu_pitch"] = 0.0
        self.blackboard["imu_yaw"] = 0.0
        self.blackboard["joy_status"] = False
        self.blackboard["imu_status"] = False

        # variables
        self.imu_data = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.joy_cmd_data = [0.0, 0.0, 0.0]  # linear x, linear y, angular z
        self.joy_state_data = (
            "none"  # start, standby, stop, emergency_stop, test_check, none
        )

        self.get_logger().info("Supervisor node started...")

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

            self.imu_data = [roll_deg, pitch_deg, yaw_deg]

            self.blackboard["imu_status"] = True
            
            # TODO: write the imu data to the shared memory

        except Exception as e:
            self.get_logger().error(f"Error converting quaternion to RPY: {e}")
            self.blackboard["imu_status"] = False

    def joy_cmd_callback(self, msg: TwistStamped) -> None:
        self.joy_cmd_data = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
        ]
        
        # TODO: write the joy cmd data to the shared memory
        # the cmd data is depend upon the robot state. 

    def joy_state_callback(self, msg: String) -> None:
        self.joy_state_data = msg.data

        self.blackboard["joy_state"] = self.joy_state_data
        self.blackboard["joy_status"] = True
        
        # TODO: write the joy state data to the shared memory


    def timer_callback(self) -> None:
        # rclpy.get_logger().info(f"robot state: {self.blackboard['state']}")
        state = self.blackboard["robot_state"]

        # self.get_logger().info(f"robot state: {state}")
        
        # TODO: write the robot state data to the shared memory. 
        
        return


def main(args=None):
    rclpy.init(args=args)
    set_ros_loggers()

    # intiiliaze blackboard
    blackboard = Blackboard()
    blackboard.set("last_imu_time", time.time())
    blackboard.set("imu_roll", 0.0)
    blackboard.set("imu_pitch", 0.0)
    blackboard.set("imu_yaw", 0.0)
    blackboard.set("robot_state", "none")
    
    fsm_client_node = rclpy.create_node("fsm_service_client")
    switch_client = fsm_client_node.create_client(SwitchController, "/controller_manager/switch_controller")
    
    # Add them to the blackboard so your Python states can use them
    blackboard.set("fsm_node", fsm_client_node)
    blackboard.set("switch_client", switch_client)

    node = StateBridgeNode(blackboard)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # create FSM from xml
    factory = YasminFactory()
    state_machine = factory.create_sm_from_file(robot_states_xml_path)
    state_machine.set_sigint_handler(True)

    # publish FSM to viewer
    YasminViewerPub(state_machine, "robot_states")

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        outcome = state_machine(blackboard)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # Catches the YASMIN C++ panic message and silences it
        if "canceled state machine" in str(e):
            print("\n[INFO] FSM interrupted and closed safely.")
        else:
            # If it's a real error, print it!
            print(f"FSM CRASHED: {e}")
    finally:
        executor.shutdown()
        node.destroy_node()
        fsm_client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
