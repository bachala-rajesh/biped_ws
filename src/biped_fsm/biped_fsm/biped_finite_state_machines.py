#!/usr/bin/env python3
"""
Finite State Machine for the robot states implemented for Biped robot. 
Common blackboard is shared with the ros2 node implemented in robot_states_fsm_python.py
"""

#TODO:
# - add joint states timer check


import time
import yasmin
from yasmin import State, Blackboard


ROLL_FALL_DETECTION_THRESHOLD = 15.0  # degrees
PITCH_FALL_DETECTION_THRESHOLD = 15.0  # degrees
IMU_FAILURE_TIMEOUT = 0.5  # seconds
JOINT_STATES_FAILURE_TIMEOUT = 0.5  # seconds
INIT_DELAY = 1.0  # seconds
JOY_START_BUTTON_HOLD = 2.0  # seconds
WAIT_TIME_INTERVAL = 3.0  # seconds


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

def check_joint_states_health(blackboard):
    """
    Checks if the joint states are recent
    Returns: 'joint_states_failure' if not recent, None if recent.
    """
    if not blackboard["joint_states_status"]:
        yasmin.YASMIN_LOG_ERROR("Joint states are not being received")
        return "joint_states_failure"

    last_time = blackboard["last_joint_states_time"]
    current_time = time.time()

    # If no data for more than timeout -> FAILURE
    if (current_time - last_time) > JOINT_STATES_FAILURE_TIMEOUT:
        yasmin.YASMIN_LOG_ERROR(
            f"Joint states TIMEOUT! Last seen {current_time - last_time:.2f}s ago"
        )
        return "joint_states_failure"
    return None


def check_joy_health(blackboard):
    """
    Checks if the joystick is connected.
    Returns: 'joy_failure' if not connected, None if connected.
    """
    if not blackboard.get("joy_state_status"):
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
        super().__init__(outcomes=["ready", "imu_failure", "joy_failure", "joint_states_failure"])

    def execute(self, blackboard: Blackboard):

        blackboard["robot_state"] = "INIT"

        time.sleep(INIT_DELAY)

        last_time = time.time()

        # wait for sensors to be healthy or timeout
        while True:
            yasmin.YASMIN_LOG_INFO("Checking health of IMU and joystick...")

            # check health of IMU and joystick
            imu_health = check_imu_health(blackboard)
            joy_health = check_joy_health(blackboard)
            joint_states_health = check_joint_states_health(blackboard)

            if imu_health != "imu_failure" and joy_health != "joy_failure" and joint_states_health != "joint_states_failure":
                break

            if time.time() - last_time > WAIT_TIME_INTERVAL:
                break

            time.sleep(0.1)

        if imu_health == "imu_failure":
            return "imu_failure"
        if joy_health == "joy_failure":
            return "joy_failure"
        if joint_states_health == "joint_states_failure":
            return "joint_states_failure"

        yasmin.YASMIN_LOG_INFO("sensors are healthy, initializing robot...")
        time.sleep(INIT_DELAY)

        return "ready"


class PassiveState(State):
    def __init__(self):
        super().__init__(outcomes=["user_start", "imu_failure", "fall_detected", "joint_states_failure", "stop"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "PASSIVE"

        yasmin.YASMIN_LOG_INFO("Robot is in passive state")
        time.sleep(1)

        start_button_hold = False

        while True:
            # check for imu failure
            imu_health = check_imu_health(blackboard)
            if imu_health == "imu_failure":
                return "imu_failure"
            
            # check for joint states failure
            joint_states_health = check_joint_states_health(blackboard)
            if joint_states_health == "joint_states_failure":
                return "joint_states_failure"

            # check for fall detection
            fall_detection_state = fall_detection(blackboard)
            if fall_detection_state == "fall_detected":
                return "fall_detected"
            
            # check for user stop
            if blackboard.get("joy_state") == "stop":
                return "stop"

            # check for user start
            if blackboard.get("joy_state") == "start":
                if not start_button_hold:
                    last_joy_start_time = time.time()
                    start_button_hold = True
                else:
                    if time.time() - last_joy_start_time > JOY_START_BUTTON_HOLD:
                        yasmin.YASMIN_LOG_INFO("User started the robot")
                        return "user_start"
            else:
                start_button_hold = False
                    
            time.sleep(0.05)


class StandbyState(State):
    def __init__(self):
        super().__init__(
            outcomes=[
                "user_active",
                "user_stop",
                "imu_failure",
                "joint_states_failure",
                "fall_detected",
                "emergency_stop",
            ]
        )

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "STANDBY"
        yasmin.YASMIN_LOG_INFO("Robot is in standby state")
        time.sleep(1)
        while True:
            # check for imu failure
            imu_health = check_imu_health(blackboard)
            if imu_health == "imu_failure":
                return "imu_failure"
            
            # check for joint states failure
            joint_states_health = check_joint_states_health(blackboard)
            if joint_states_health == "joint_states_failure":
                return "joint_states_failure"

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

            time.sleep(0.05)


class ActiveState(State):
    def __init__(self):
        super().__init__(
            outcomes=[
                "fall_detected",
                "imu_failure",
                "user_standby",
                "emergency_stop",
                "joint_states_failure",
            ]
        )

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "ACTIVE"
        yasmin.YASMIN_LOG_INFO("Robot is in active state")
        time.sleep(1)

        while True:
            # check for imu failure
            imu_health = check_imu_health(blackboard)
            if imu_health == "imu_failure":
                return "imu_failure"
            
            # check for joint states failure
            joint_states_health = check_joint_states_health(blackboard)
            if joint_states_health == "joint_states_failure":
                return "joint_states_failure"

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
        super().__init__(outcomes=["reset", "emergency_stop", "stop"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "FALLEN"
        yasmin.YASMIN_LOG_INFO("Robot is in fallen state")
        time.sleep(1)

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
        super().__init__(outcomes=["recovered", "stop", "emergency_stop"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "ERROR"
        yasmin.YASMIN_LOG_INFO("Robot is in error state")
        time.sleep(1)

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
        super().__init__(outcomes=["reset"])

    def execute(self, blackboard: Blackboard):
        blackboard["robot_state"] = "STOP"
        yasmin.YASMIN_LOG_INFO("Robot is in stop state")
        time.sleep(1)

        while True:
            if blackboard.get("joy_state") == "start":
                return "reset"

            time.sleep(0.1)
