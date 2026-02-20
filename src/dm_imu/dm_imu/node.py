#! /usr/bin/env python3

"""
This node is a ROS 2 node that publishes the IMU data from the DM IMU.

Modifications done by Bachala Rajesh
- Added treconnect functionality to the node.
"""

import math
import threading
from typing import Optional, Tuple
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PoseStamped

# Use serial implementation (keep path)
from .modules.dm_serial import DM_Serial

TIMEOUT_THRESHOLD = 0.5


def euler_rpy_to_quat(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """ZYX intrinsic (yaw->pitch->roll); roll/pitch/yaw in radians."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class DmImuNode(Node):
    def __init__(self):
        super().__init__("dm_imu")

        # ---------- Parameters ----------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rpy_in_degree", True)
        self.declare_parameter("verbose", True)  # Terminal logging
        self.declare_parameter(
            "qos_reliable", True
        )  # Publisher QoS (default Reliable, visible in RViz)
        # Topic switches (all enabled by default)
        self.declare_parameter("publish_imu_data", False)  # /imu/data
        self.declare_parameter("publish_rpy", True)  # /imu/rpy
        self.declare_parameter("publish_pose", False)  # /imu/pose
        self.declare_parameter("reconnect_interval_sec", 1.0)

        def _p(name, default=None):
            try:
                v = self.get_parameter(name).value
                return default if v in (None, "") else v
            except Exception:
                return default

        self.port = _p("port", "/dev/ttyACM0")
        self.frame_id = _p("frame_id", "imu_link")
        self.publish_rpy = bool(_p("publish_rpy", True))
        self.publish_rpy_in_degree = bool(_p("publish_rpy_in_degree", True))
        self.verbose = bool(_p("verbose", True))
        qos_reliable = bool(_p("qos_reliable", True))
        # Topic switches
        self.publish_imu_data = bool(_p("publish_imu_data", False))
        self.publish_rpy = bool(_p("publish_rpy", True))
        self.publish_pose = bool(_p("publish_pose", False))
        self.reconnect_interval = float(_p("reconnect_interval_sec", 1.0))

        baud = _p("baudrate", 921600)
        try:
            self.baudrate = int(baud)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid baudrate "{baud}", fallback to 921600')
            self.baudrate = 921600

        # ---------- QoS ----------
        if qos_reliable:
            from rclpy.qos import (
                QoSProfile,
                ReliabilityPolicy,
                HistoryPolicy,
                DurabilityPolicy,
            )

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            from rclpy.qos import qos_profile_sensor_data  # BestEffort

            qos = qos_profile_sensor_data

        # ---------- Publishers ----------
        self.pub_imu = (
            self.create_publisher(Imu, "imu/data", qos)
            if self.publish_imu_data
            else None
        )
        self.pub_rpy = (
            self.create_publisher(Vector3Stamped, "imu/rpy", qos)
            if self.publish_rpy
            else None
        )
        self.pub_pose = (
            self.create_publisher(PoseStamped, "imu/pose", 10)
            if self.publish_pose
            else None
        )

        # ---------- Serial ----------
        self.ser = None
        self.connect_to_imu()

        # ---------- Timers ----------
        # Deduplication by timestamp; skip if unavailable
        self._last_stamp_ts: Optional[float] = None
        self._closing = threading.Event()
        self._logged_bad_fmt_once = False
        self._no_frame_ticks = 0
        self._pub_count = 0

        # 200 Hz polling
        self.timer_pub = self.create_timer(0.005, self._on_timer_publish)
        # Stats every 2s (if class provides get_stats)
        self.timer_stat = self.create_timer(2.0, self._on_timer_stats)

        self.last_frame_time = time.time()

    def connect_to_imu(self):
        try:
            self.ser = DM_Serial(self.port, baudrate=self.baudrate)
            if self.ser.start_reader():
                self.get_logger().info(
                    f"connected to imu serial {self.port} @ {self.baudrate}"
                )
            else:
                self.get_logger().error(f"Failed to open imu serial {self.port}")
        except Exception as e:
            self.get_logger().warn(f"Init serial failed: {e}")
            raise

    def reconnect_to_imu(self):
        while True:
            try:
                if self.ser is not None:
                    try:
                        self.ser.stop_reader()
                        self.ser.destory()
                    except Exception:
                        pass

                self.ser = DM_Serial(self.port, baudrate=self.baudrate)
                self.ser.start_reader()
                if self.ser.is_open:
                    self.get_logger().info(
                        f"Reconnected to imu serial {self.port} @ {self.baudrate}"
                    )
                    self.last_frame_time = time.time()
                    break
                else:
                    self.get_logger().error(
                        "Failed reconnection to IMU serial.. trying again."
                    )
            except Exception as e:
                self.get_logger().error(f"Reconnection failed: {e}")

            time.sleep(self.reconnect_interval)

    # ----------- Timers -----------
    def _on_timer_publish(self):
        if self.ser.is_open is False:
            self.get_logger().error("imu serial not connected!!!")
            self.reconnect_to_imu()

        # latest = None
        try:
            latest = self.ser.get_latest()
            self.last_frame_time = latest[1]
        except Exception as e:
            if self.verbose:
                self.get_logger().warn(f"get_latest() exception: {e}")
            return

        # check for timeout
        current_time = time.time()
        if current_time - self.last_frame_time > TIMEOUT_THRESHOLD:
            self.get_logger().error("Timeout: no frame received from serial")
            self.reconnect_to_imu()
            return

        ok, stamp_ts, r_deg, p_deg, y_deg = self._extract_latest(latest)  # Extract data
        if not ok:
            if not self._logged_bad_fmt_once and self.verbose:
                self.get_logger().warn(
                    f"Unknown latest frame format; example: {repr(latest)}"
                )
                self._logged_bad_fmt_once = True
            return

        # Deduplication (prefer timestamp; skip if none)
        if stamp_ts is not None and stamp_ts == self._last_stamp_ts:
            return
        self._last_stamp_ts = stamp_ts

        # Degrees to radians
        r = r_deg * math.pi / 180.0
        p = p_deg * math.pi / 180.0
        y = y_deg * math.pi / 180.0

        stamp = self.get_clock().now().to_msg()

        # /imu/data
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        qx, qy, qz, qw = euler_rpy_to_quat(r, p, y)

        # /imu/rpy
        if self.pub_rpy is not None:
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = stamp
            rpy_msg.header.frame_id = self.frame_id
            if self.publish_rpy_in_degree:
                rpy_msg.vector.x = float(r_deg)
                rpy_msg.vector.y = float(p_deg)
                rpy_msg.vector.z = float(y_deg)
            else:
                rpy_msg.vector.x = float(r)
                rpy_msg.vector.y = float(p)
                rpy_msg.vector.z = float(y)
            # /imu/rpy
            self.pub_rpy.publish(rpy_msg)

        if self.publish_imu_data == False and self.publish_pose == False:
            return

        # Validity check + normalization (prevents Unvisualizable)
        def _finite(*vals):
            return all(math.isfinite(v) for v in vals)

        if not _finite(qx, qy, qz, qw):
            if self.verbose:
                self.get_logger().warn(
                    "Quaternion has NaN/Inf, publishing identity (0,0,0,1)"
                )
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
            if n < 1e-6:
                if self.verbose:
                    self.get_logger().warn(
                        "Quaternion norm ~0, publishing identity (0,0,0,1)"
                    )
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

        # /imu/data
        if self.pub_imu is not None:
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            imu.orientation_covariance[0] = 0.02
            imu.orientation_covariance[4] = 0.02
            imu.orientation_covariance[8] = 0.02
            for i in range(9):
                imu.angular_velocity_covariance[i] = -1.0
                imu.linear_acceleration_covariance[i] = -1.0
            self.pub_imu.publish(imu)

        # /imu/pose (origin + IMU pose) for RViz visualization
        if self.pub_pose is not None:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pub_pose.publish(pose)

        # Terminal logging
        self._pub_count += 1
        self._no_frame_ticks = 0
        if self.verbose:
            self.get_logger().info(
                f"#{self._pub_count} RPY(deg)=({r_deg:.2f}, {p_deg:.2f}, {y_deg:.2f}) "
                f"RPY(rad)=({r:.3f}, {p:.3f}, {y:.3f})"
            )

    def _on_timer_stats(self):
        try:
            if hasattr(self.ser, "get_stats"):
                stats = self.ser.get_stats()
                msg = (
                    " ".join([f"{k}={v}" for k, v in stats.items()])
                    if isinstance(stats, dict)
                    else str(stats)
                )
                if self.verbose:
                    self.get_logger().info(f"[stats] {msg}")
        except Exception:
            pass  # Stats exception does not affect main flow

    # ----------- Helpers -----------
    def _extract_latest(
        self, latest
    ) -> Tuple[bool, Optional[float], float, float, float]:
        """
        Returns (ok, stamp_ts, roll_deg, pitch_deg, yaw_deg)
        - Compatible with nested format: ((rid, (r,p,y)), ts, extra)
        - Also supports dict / flat tuple / object attributes
        """
        try:
            # dict
            if isinstance(latest, dict):
                r = latest.get("roll") or latest.get("r") or latest.get("Roll")
                p = latest.get("pitch") or latest.get("p") or latest.get("Pitch")
                y = latest.get("yaw") or latest.get("y") or latest.get("Yaw")
                ts = (
                    latest.get("ts")
                    or latest.get("timestamp")
                    or latest.get("time")
                    or None
                )
                if r is not None and p is not None and y is not None:
                    return (
                        True,
                        (float(ts) if ts is not None else None),
                        float(r),
                        float(p),
                        float(y),
                    )

            # tuple/list
            if isinstance(latest, (tuple, list)):
                # ((rid, (r,p,y)), ts, extra)
                if len(latest) >= 2 and isinstance(latest[0], (tuple, list)):
                    rid_part = latest[0]
                    ts = latest[1] if isinstance(latest[1], (int, float)) else None
                    if (
                        len(rid_part) == 2
                        and isinstance(rid_part[1], (tuple, list))
                        and len(rid_part[1]) >= 3
                    ):
                        r, p, y = rid_part[1][0], rid_part[1][1], rid_part[1][2]
                        return (
                            True,
                            (float(ts) if ts is not None else None),
                            float(r),
                            float(p),
                            float(y),
                        )
                # (rid, r, p, y)
                if len(latest) >= 4 and not isinstance(latest[0], (tuple, list)):
                    _, r, p, y = latest[0], latest[1], latest[2], latest[3]
                    return True, None, float(r), float(p), float(y)
                # (r, p, y)
                if len(latest) == 3:
                    r, p, y = latest[0], latest[1], latest[2]
                    return True, None, float(r), float(p), float(y)

            # Object attributes
            r = getattr(latest, "roll", None)
            p = getattr(latest, "pitch", None)
            y = getattr(latest, "yaw", None)
            ts = (
                getattr(latest, "ts", None)
                or getattr(latest, "timestamp", None)
                or None
            )
            if r is not None and p is not None and y is not None:
                return (
                    True,
                    (float(ts) if ts is not None else None),
                    float(r),
                    float(p),
                    float(y),
                )

            return False, None, 0.0, 0.0, 0.0
        except Exception as e:
            if self.verbose:
                self.get_logger().debug(f"_extract_latest exception: {e}")
            return False, None, 0.0, 0.0, 0.0

    # ----------- Shutdown -----------
    def destroy_node(self):
        if getattr(self, "_closing", None) is None or self._closing.is_set():
            try:
                super().destroy_node()
            except Exception:
                pass
            return
        self._closing.set()
        try:
            if hasattr(self.ser, "stop_reader"):
                self.ser.stop_reader()
        except Exception:
            pass
        try:
            if hasattr(self.ser, "close"):
                self.ser.close()
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main():
    rclpy.init()
    node = DmImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
