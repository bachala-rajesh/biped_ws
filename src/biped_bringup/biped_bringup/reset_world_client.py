#!/usr/bin/env python3
"""
ROS 2 node that calls the /mujoco_ros2_control_node/reset_world service.
"""

import sys

import rclpy
from rclpy.node import Node
from mujoco_ros2_control_msgs.srv import ResetWorld


class ResetWorldClient(Node):
    def __init__(self, keyframe: str = "") -> None:
        super().__init__("reset_world_client")
        self._keyframe = keyframe

        self._client = self.create_client(ResetWorld, "/mujoco_ros2_control_node/reset_world")
        
        self.get_logger().info("Waiting for /mujoco_ros2_control_node/reset_world service...")
        
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available.")
            raise RuntimeError("Service /mujoco_ros2_control_node/reset_world not available")

    def call_reset(self) -> bool:
        request = ResetWorld.Request()
        request.keyframe = self._keyframe

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Success: {response.message}")
            else:
                self.get_logger().warn(f"Failed: {response.message}")
            return response.success
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return False


def main(args=None) -> int:
    rclpy.init(args=args)

    keyframe = "home"
    if args is not None and len(args) > 1:
        keyframe = args[1]

    try:
        node = ResetWorldClient(keyframe=keyframe)
        success = node.call_reset()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))





