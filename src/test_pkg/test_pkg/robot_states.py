#!/usr/bin/env python3

# TODO: check the switch_controllers function is working correctly...
# TODO: Add the action client in the passive state to move the robot to the standby position.
# TODO: add the action client in the stop state to move the robot to the zero position.
"""
Finite State Machine for the robot states implemented for Biped robot. 
Common blackboard is shared with the ros2 node implemented in robot_states_fsm_python.py
"""

import time
import yasmin
from yasmin import State, Blackboard
import rclpy
from controller_manager_msgs.srv import SwitchController


ROLL_FALL_DETECTION_THRESHOLD = 15.0  # degrees
PITCH_FALL_DETECTION_THRESHOLD = 15.0  # degrees
IMU_FAILURE_TIMEOUT = 1.0  # seconds
INIT_DELAY = 1.0  # seconds
JOY_START_BUTTON_HOLD = 2.0  # seconds
WAIT_TIME_INTERVAL = 3.0  # seconds


# controller names
TRAJECTORY_CONTROLLER = "trajectory_controller"
POLICY_CONTROLLER = "joints_position_controller"


def switch_controllers(blackboard, activate_list, deactivate_list):
    """
    Switches ROS 2 controllers safely.
    """
    blackboard["controllers_switched"] = False

    client = blackboard.get("switch_client")
    fsm_node = blackboard.get("fsm_node")

    # Safety check: ensure client exists
    if not client or not fsm_node:
        yasmin.YASMIN_LOG_ERROR("Switch client missing from blackboard!")
        return False

    if not client.wait_for_service(timeout_sec=2.0):
        yasmin.YASMIN_LOG_ERROR("Controller Manager service is dead!")
        return False

    # Create the request
    req = SwitchController.Request()
    req.activate_controllers = activate_list
    req.deactivate_controllers = deactivate_list
    req.strictness = SwitchController.Request.STRICT
    
    # logging the controllers to be switched
    yasmin.YASMIN_LOG_INFO(f"Switching controllers: ON: {activate_list}, OFF: {deactivate_list}")

    # Call the service and wait for it to finish
    future = client.call_async(req)
    rclpy.spin_until_future_complete(fsm_node, future)

    # Return True if successful
    if future.result() is not None and future.result().ok:
        yasmin.YASMIN_LOG_INFO(
            f"Controllers Switched! ON: {activate_list}, OFF: {deactivate_list}"
        )
        blackboard["controllers_switched"] = True
        return True

    yasmin.YASMIN_LOG_ERROR("Failed to switch controllers.")
    return False




# --- HELPER: SENSOR WATCHDOG ---
def check_imu_health(blackboard):
    """
    Checks if the IMU is alive.
    Returns: 'imu_failure' if dead, None if healthy.
    """

    if not blackboard["imu_status"]:
        yasmin.YASMIN_LOG_ERROR("IMU is not connected")
        return "imu_failure"

    last_time = blackboard["last_imu_time"]
    current_time = time.time()

    # If no data for more than timeout -> FAILURE
    if (current_time - last_time) > IMU_FAILURE_TIMEOUT:
        yasmin.YASMIN_LOG_ERROR(
            f"IMU TIMEOUT! Last seen {current_time - last_time:.2f}s ago"
        )
        return "imu_failure"
    return None


def check_joy_health(blackboard):
    """
    Checks if the joystick is connected.
    Returns: 'joy_failure' if not connected, None if connected.
    """
    if not blackboard.get("joy_status"):
        yasmin.YASMIN_LOG_ERROR("Joystick is not connected")
        return "joy_failure"
    return None


def fall_detection(blackboard):
    """
    Checks if the robot has fallen.
    Returns: 'fall_detected' if fallen, None if not fallen.
    """
    if abs(blackboard.get("imu_roll")) > ROLL_FALL_DETECTION_THRESHOLD:
        yasmin.YASMIN_LOG_ERROR("Fall detected: roll angle is too high")
        return "fall_detected"
    if abs(blackboard.get("imu_pitch")) > PITCH_FALL_DETECTION_THRESHOLD:
        yasmin.YASMIN_LOG_ERROR("Fall detected: pitch angle is too high")
        return "fall_detected"

    return None


class InitState(State):
    def __init__(self):
        super().__init__(outcomes=["ready", "imu_failure", "joy_failure", "switch_controller_failure"])

    def execute(self, blackboard: Blackboard):

        blackboard["robot_state"] = "init"

        time.sleep(INIT_DELAY)

        last_time = time.time()

        # wait for sensors to be healthy or timeout
        while True:
            yasmin.YASMIN_LOG_INFO("Checking health of IMU and joystick...")

            # check health of IMU and joystick
            imu_health = check_imu_health(blackboard)
            joy_health = check_joy_health(blackboard)

            if imu_health != "imu_failure" and joy_health != "joy_failure":
                break

            # wait for timeout
            if time.time() - last_time > WAIT_TIME_INTERVAL:
                break

            time.sleep(0.1)

        if imu_health == "imu_failure":
            return "imu_failure"
        if joy_health == "joy_failure":
            return "joy_failure"

        yasmin.YASMIN_LOG_INFO("sensors are healthy, switching controllers...")
        # switch to trajectory controller
        switch_controller_status = switch_controllers(blackboard, [TRAJECTORY_CONTROLLER], [POLICY_CONTROLLER])
        
        if not switch_controller_status:
            return "switch_controller_failure"
        else:
            yasmin.YASMIN_LOG_INFO("controllers switched successfully")
            return "ready"
        

    

class PassiveState(State):
    def __init__(self):
        super().__init__(outcomes=["user_start", "imu_failure", "fall_detected", "start_position_failure", "switch_controller_failure"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "passive"

        yasmin.YASMIN_LOG_INFO("Robot is in passive state")
        
        time.sleep(1)

        start_button_hold = False
        start_button_status = False

        while True:
            # check for imu failure
            imu_health = check_imu_health(blackboard)
            if imu_health == "imu_failure":
                return "imu_failure"

            # check for fall detection
            fall_detection_state = fall_detection(blackboard)
            if fall_detection_state == "fall_detected":
                return "fall_detected"

            # check for user start
            if blackboard.get("joy_state") == "start":
                if not start_button_hold:
                    last_joy_start_time = time.time()
                    start_button_hold = True
                else:
                    if time.time() - last_joy_start_time > JOY_START_BUTTON_HOLD:
                        yasmin.YASMIN_LOG_INFO("User started the robot")
                        start_button_status = True
                        break
            else:
                start_button_hold = False

            time.sleep(0.1)
        
        if start_button_status:
            # TODO: add feature to move the robot to the start position
            # switch to policy controller
            switch_controller_status = switch_controllers(blackboard, [POLICY_CONTROLLER], [TRAJECTORY_CONTROLLER])
            if not switch_controller_status:
                return "switch_controller_failure"
            else:
                return "user_start"


class StandbyState(State):
    def __init__(self):
        super().__init__(
            outcomes=[
                "user_active",
                "user_stop",
                "imu_failure",
                "fall_detected",
                "emergency_stop",
            ]
        )

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "standby"
        yasmin.YASMIN_LOG_INFO("Robot is in standby state")
        time.sleep(1)
        while True:
            # check for imu failure
            imu_health = check_imu_health(blackboard)
            if imu_health == "imu_failure":
                return "imu_failure"

            # check for user stop
            if blackboard.get("joy_state") == "stop":
                return "user_stop"

            # check for fall detection
            fall_detection_state = fall_detection(blackboard)
            if fall_detection_state == "fall_detected":
                return "fall_detected"

            # check for user active
            if blackboard.get("joy_state") == "active":
                return "user_active"

            # check for emergency stop
            if blackboard.get("joy_state") == "emergency_stop":
                return "emergency_stop"

            time.sleep(0.1)


class ActiveState(State):
    def __init__(self):
        super().__init__(
            outcomes=[
                "fall_detected",
                "imu_failure",
                "user_standby",
                "emergency_stop",
            ]
        )

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "active"
        yasmin.YASMIN_LOG_INFO("Robot is in active state")
        time.sleep(1)

        while True:
            # check for imu failure
            imu_health = check_imu_health(blackboard)
            if imu_health == "imu_failure":
                return "imu_failure"

            # check for fall detection
            fall_detection_state = fall_detection(blackboard)
            if fall_detection_state == "fall_detected":
                return "fall_detected"

            # check for user standby
            if blackboard.get("joy_state") == "standby":
                return "user_standby"

            # check for emergency stop
            if blackboard.get("joy_state") == "emergency_stop":
                return "emergency_stop"

            time.sleep(0.1)


class FallenState(State):
    def __init__(self):
        super().__init__(outcomes=["reset", "emergency_stop", "stop", "switch_controller_failure"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "fallen"
        yasmin.YASMIN_LOG_INFO("Robot is in fallen state")
        time.sleep(1)
        
        # deactivate all the controllers
        switch_controller_status = switch_controllers(blackboard, [], [TRAJECTORY_CONTROLLER, POLICY_CONTROLLER])
        if not switch_controller_status:
            return "switch_controller_failure"

        while True:
            # check for reset
            if blackboard.get("joy_state") == "start":
                return "reset"

            # check for emergency stop
            if blackboard.get("joy_state") == "emergency_stop":
                return "emergency_stop"

            # check for stop
            if blackboard.get("joy_state") == "stop":
                return "stop"

            time.sleep(0.1)


class ErrorState(State):
    def __init__(self):
        super().__init__(outcomes=["recovered", "stop", "emergency_stop", "switch_controller_failure"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "error"
        yasmin.YASMIN_LOG_INFO("Robot is in error state")
        time.sleep(1)
        
        # deactivate all the controllers
        switch_controller_status = switch_controllers(blackboard, [], [TRAJECTORY_CONTROLLER, POLICY_CONTROLLER])
        if not switch_controller_status:
            return "switch_controller_failure"

        while True:
            if blackboard.get("joy_state") == "start":
                return "recovered"

            if blackboard.get("joy_state") == "stop":
                return "stop"

            if blackboard.get("joy_state") == "emergency_stop":
                return "emergency_stop"

            time.sleep(0.1)

        return "recovered"


class StopState(State):
    def __init__(self):
        super().__init__(outcomes=["reset", "switch_controller_failure"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "emergency"
        yasmin.YASMIN_LOG_INFO("Robot is in emergency state")
        time.sleep(1)
        
        # deactivate all the controllers
        switch_controller_status = switch_controllers(blackboard, [], [TRAJECTORY_CONTROLLER, POLICY_CONTROLLER])
        if not switch_controller_status:
            return "switch_controller_failure"

        while True:
            if blackboard.get("joy_state") == "start":
                return "reset"

            time.sleep(0.1)
