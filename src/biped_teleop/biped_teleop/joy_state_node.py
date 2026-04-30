#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class JOYSTATE:
    START = "start"
    STOP = "stop"
    STANDBY = "standby"
    ACTIVE = "active"
    EMERGENCY_STOP = "emergency_stop"
    TEST_CHECK = "test_check"
    NONE = "none"  # no button pressed


class GAMEPAD_DEADZONE:
    """Deadzone thresholds and timing for gamepad axes, buttons, and restart hold."""

    JOYSTICK_DEADZONE = 0.2
    BUTTON_DEADZONE = 0.5
    RESTART_BUTTON_HOLD_TIME = 3.0  # seconds
    DEBOUNCE_TIME = 0.05  # seconds for debouncing


class TeleopNode(Node):
    def __init__(self):
        """Initialize the node, load parameters, and create subscriptions/timer."""
        super().__init__(
            "joy_state_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        # last published state and current state
        self.last_published_state = String()
        self.current_state = String()
        self.last_state = String()

        self.last_state.data = JOYSTATE.NONE
        self.current_state.data = JOYSTATE.NONE
        self.last_published_state.data = JOYSTATE.NONE

        # debounce variables
        self.state_changed = True
        self.current_state_init_time = self.get_clock().now()

        # intialize joystick mapping parameters
        self.hw_axes = {}
        self.hw_buttons = {}
        self.cmd_map = {}

        # read parameters
        self.load_joystick_layout()
        self.load_control_map()

        # joystick data
        self.joy_msg_timestamp = None

        # callback groups
        self.joy_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # Topics
        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self.joy_callback, 10, callback_group=self.joy_callback_group
        )
        self.joy_state_publisher = self.create_publisher(String, "/joy/state", 10)

        # timer
        self.timer_ = self.create_timer(0.05, self.timer_callback)  # 50ms or 20Hz callback

        # initial publish
        self.initial_publish()

        # Logging
        self.get_logger().info("joy state node Ready")

    def initial_publish(self):
        """Publish the initial state."""
        self.joy_state_publisher.publish(self.current_state)

    def load_joystick_layout(self):
        """Load joystick hardware layout from ROS parameters.

        Reads all parameters under the "joy_layout" prefix and populates
        self.hw_axes and self.hw_buttons with mappings from logical names
        (e.g. "left_stick_x") to hardware indices.
        """
        joy_params = self.get_parameters_by_prefix("joy_layout")

        for name, param_obj in joy_params.items():
            # split names
            split_name = name.split(".")
            if len(split_name) < 2:
                continue

            category = split_name[0]  # "axes" or "buttons"
            key = split_name[1]  # The name of the axis/button
            val = param_obj.value  # The ID number

            if category == "axes":
                self.hw_axes[key] = val
            elif category == "buttons":
                self.hw_buttons[key] = val

    def load_control_map(self):
        """Load control map from parameters and bind actions to hardware IDs.

        Reads parameters under "teleop_control_map" and builds self.cmd_map:
        each action (e.g. linear_x, speed_scale) is mapped to a category
        ("axes" or "buttons") and the corresponding hardware ID from the
        joystick layout. Values are filled in by joy_callback.
        """
        ctrl_params = self.get_parameters_by_prefix("functional_control_map")

        for name, param_object in ctrl_params.items():
            # split names
            split_name = name.split(".")
            if len(split_name) < 2:
                continue

            category = split_name[0]  # "locomotion_control" or "scale_control"
            action_name = split_name[1]  # The name of the axis/button
            physical_name = param_object.value  # The ID number

            # Find the ID for this physical_name
            real_id = None
            if physical_name in self.hw_axes:
                real_id = self.hw_axes[physical_name]
                self.cmd_map[action_name] = {
                    "category": "axes",
                    "id": real_id,
                }
            elif physical_name in self.hw_buttons:
                real_id = self.hw_buttons[physical_name]
                self.cmd_map[action_name] = {
                    "category": "buttons",
                    "id": real_id,
                }

            # store the real id
            if real_id is not None:
                self.get_logger().info(
                    f"Mapped action '{action_name}' -> ID {real_id} ('{physical_name}')"
                )
            else:
                self.get_logger().warn(
                    f"Physical control '{physical_name}' not found in axes or buttons."
                )

    def joy_callback(self, msg):
        """Process incoming joystick messages and update command values.

        Applies deadzone to axes and buttons, then stores the resulting
        values in self.cmd_map[action_name]["value"] for each mapped action.
        Also updates self.joy_msg_timestamp from the message header.

        Args:
            msg: sensor_msgs/Joy message; may be None (treated as no data).
        """
        # get the action data
        if msg is None:
            self.get_logger().warn("No joystick data received.")
            self.joy_msg_timestamp = None
            return

        self.joy_msg_timestamp = msg.header.stamp

        for action_name, action_data in self.cmd_map.items():
            if action_data["category"] == "axes":
                action_value = msg.axes[action_data["id"]]
                if abs(action_value) < GAMEPAD_DEADZONE.JOYSTICK_DEADZONE:
                    action_value = 0.0
            elif action_data["category"] == "buttons":
                action_value = msg.buttons[action_data["id"]]
                if abs(action_value) < GAMEPAD_DEADZONE.BUTTON_DEADZONE:
                    action_value = 0.0
            else:
                self.get_logger().warn(
                    f"Invalid category for action {action_name}: {action_data['category']}"
                )
                continue

            self.cmd_map[action_name]["value"] = action_value

    def timer_callback(self):
        """Publish velocity command from current joystick state.

        Runs at a fixed interval (0.1 s). Reads values from self.cmd_map,
        applies speed scaling and limits, and publishes a TwistStamped
        on cmd_vel. If no joystick data has been received yet, publishes
        an empty twist and returns.
        """
        if self.joy_msg_timestamp is None:
            self.get_logger().warn("No joystick data received.")
            return

        self.current_state.data = JOYSTATE.NONE
        if self.cmd_map[JOYSTATE.START]["value"] == 1:
            self.current_state.data = JOYSTATE.START
        elif self.cmd_map[JOYSTATE.STANDBY]["value"] == 1:
            self.current_state.data = JOYSTATE.STANDBY
        elif self.cmd_map[JOYSTATE.STOP]["value"] == 1:
            self.current_state.data = JOYSTATE.STOP
        elif self.cmd_map[JOYSTATE.EMERGENCY_STOP]["value"] == 1:
            self.current_state.data = JOYSTATE.EMERGENCY_STOP
        elif self.cmd_map[JOYSTATE.TEST_CHECK]["value"] == 1:
            self.current_state.data = JOYSTATE.TEST_CHECK
        elif self.cmd_map[JOYSTATE.ACTIVE]["value"] == 1:
            self.current_state.data = JOYSTATE.ACTIVE
            
        if self.current_state.data == JOYSTATE.NONE:
            self.last_state.data = JOYSTATE.NONE
            self.joy_state_publisher.publish(self.current_state)
            return

        # check for state change
        if self.current_state.data != self.last_state.data:
            self.last_state.data = self.current_state.data
            self.current_state_init_time = self.get_clock().now()
            return

        # check for debounce
        now = self.get_clock().now()
        dt = (now - self.current_state_init_time).nanoseconds / 1e6
        if dt >= GAMEPAD_DEADZONE.DEBOUNCE_TIME:
            # state is stable, publish it
            self.joy_state_publisher.publish(self.current_state)
            self.last_published_state.data = self.current_state.data
            self.state_changed = False
            self.last_state.data = JOYSTATE.NONE


def main(args=None):
    """Create the teleop node and run it with a multi-threaded executor."""
    rclpy.init(args=args)
    so101_teleop_node = TeleopNode()

    executor = MultiThreadedExecutor()
    executor.add_node(so101_teleop_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    so101_teleop_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
