#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random
import time

# ### SHM 1: Imports for Shared Memory
from multiprocessing import shared_memory
import struct


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")

        self.publisher_ = self.create_publisher(Imu, "imu/data", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("IMU Publisher Node Started")

        # ### SHM 2: Initialize Shared Memory
        self.shm_name = "imu_orientation_shm"
        # 4 floats (x,y,z,w) * 4 bytes each = 16 bytes
        self.data_size = 16

        try:
            # Try to create new memory space
            self.shm = shared_memory.SharedMemory(
                name=self.shm_name, create=True, size=self.data_size
            )
            self.get_logger().info(f"Created Shared Memory: {self.shm_name}")
        except FileExistsError:
            # If it already exists (from a previous run that didn't close properly), connect to it
            self.shm = shared_memory.SharedMemory(name=self.shm_name)
            self.get_logger().info(
                f"Connected to existing Shared Memory: {self.shm_name}"
            )

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Generate random values
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(-1.0, 1.0)
        w = random.uniform(-1.0, 1.0)

        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        # ### SHM 3: Write to Shared Memory
        # '4f' means pack 4 floats.
        # We pack x, y, z, w into binary data
        packed_data = struct.pack("4f", x, y, z, w)

        # Write binary data directly into the memory buffer
        self.shm.buf[: self.data_size] = packed_data

        self.publisher_.publish(msg)

    def cleanup_shm(self):
        """Clean up shared memory resources"""
        self.shm.close()
        try:
            self.shm.unlink()  # Delete the memory file from the OS
            self.get_logger().info("Shared Memory Unlinked")
        except FileNotFoundError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ### SHM 4: Ensure cleanup
        node.cleanup_shm()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
