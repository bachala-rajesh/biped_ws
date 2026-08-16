import threading
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import mujoco
import mujoco.viewer
from ament_index_python.packages import get_package_share_directory


JOINT_NAMES = ['left_knee_joint', 'right_knee_joint', 'left_wheel_joint', 'right_wheel_joint']

# ctrl indices: [left_knee_act, right_knee_act, left_wheel_act, right_wheel_act]
CTRL_LEFT_KNEE  = 0
CTRL_RIGHT_KNEE = 1
CTRL_LEFT_WHEEL  = 2
CTRL_RIGHT_WHEEL = 3


class MujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_sim_node')

        xml_path = os.path.join(
            get_package_share_directory('mujoco_wheeled'),
            'models', 'wheeled_robot.xml'
        )
        self.get_logger().info(f'Loading model: {xml_path}')

        self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._data  = mujoco.MjData(self._model)

        # Reset to home keyframe
        mujoco.mj_resetDataKeyframe(self._model, self._data, 0)

        self._lock = threading.Lock()

        # Build joint-name → qpos index map
        self._joint_qpos_idx: dict[str, int] = {}
        self._joint_qvel_idx: dict[str, int] = {}
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self._joint_qpos_idx[name] = self._model.jnt_qposadr[jid]
            self._joint_qvel_idx[name] = self._model.jnt_dofadr[jid]

        # Publishers / subscribers
        self._js_pub = self.create_publisher(JointState, '/wheeled_robot/joint_states', 10)
        self.create_subscription(Float64MultiArray, '/wheeled_robot/knee_commands',
                                 self._knee_cb, 10)
        self.create_subscription(Float64MultiArray, '/wheeled_robot/wheel_commands',
                                 self._wheel_cb, 10)

        # 200 Hz sim loop, 50 Hz publish
        self.create_timer(0.005, self._sim_step)
        self._pub_counter = 0

        # Passive viewer in background thread
        self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
        self._viewer_thread = threading.Thread(target=self._viewer_loop, daemon=True)
        self._viewer_thread.start()

        self.get_logger().info('MujocoSimNode ready.')

    # ------------------------------------------------------------------ #

    def _sim_step(self) -> None:
        with self._lock:
            mujoco.mj_step(self._model, self._data)
            self._pub_counter += 1
            if self._pub_counter >= 4:   # 200 Hz / 4 = 50 Hz publish
                self._pub_counter = 0
                self._publish_joint_states()

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(self._data.qpos[self._joint_qpos_idx[n]]) for n in JOINT_NAMES]
        msg.velocity = [float(self._data.qvel[self._joint_qvel_idx[n]]) for n in JOINT_NAMES]
        msg.effort   = []
        self._js_pub.publish(msg)

    def _knee_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        with self._lock:
            self._data.ctrl[CTRL_LEFT_KNEE]  = msg.data[0]
            self._data.ctrl[CTRL_RIGHT_KNEE] = msg.data[1]

    def _wheel_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        with self._lock:
            self._data.ctrl[CTRL_LEFT_WHEEL]  = msg.data[0]
            self._data.ctrl[CTRL_RIGHT_WHEEL] = msg.data[1]

    def _viewer_loop(self) -> None:
        # Sync viewer at ~60 Hz without touching mujoco data directly;
        # the viewer reads model+data shared with sim thread.
        import time
        while self._viewer.is_running():
            with self._lock:
                self._viewer.sync()
            time.sleep(1.0 / 60.0)

    def destroy_node(self) -> None:
        self._viewer.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
