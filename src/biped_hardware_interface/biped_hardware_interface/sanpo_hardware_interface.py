"""
This test script is to test the code of v7_sanpo_position_control_oops.py.
The test includes a sine wave position command to verify smooth control.
"""



from dataclasses import dataclass
from typing import Optional

from bitstring import BitArray
import time
from serial import Serial
import math

# logger related
import sys
from loguru import logger

logger.remove()  # Remove default logger
logger.add(
    sys.stderr,
    level="DEBUG",
    format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <level>{message}</level>",
)

# --- CAN MOTOR PROTOCOL LIMITS ---
P_MIN = -12.56
P_MAX = 12.56
V_MIN = -45.0
V_MAX = 45.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0
# -----------------------------------

# --- predefined bit arrays- MIT protocol and MIT headers ----
ENABLE_MOTOR_BIT_ARRAY = BitArray(bytes=b"\xff\xff\xff\xff\xff\xff\xff\xfc", length=64)
DISABLE_MOTOR_BIT_ARRAY = BitArray(bytes=b"\xff\xff\xff\xff\xff\xff\xff\xfd", length=64)
ENABLE_ZERO_POS_BIT_ARRAY = BitArray(
    bytes=b"\xff\xff\xff\xff\xff\xff\xff\xfe", length=64
)
SET_HEADER = BitArray(bytes=b"\x00", length=8)
MOVE_HEADER = BitArray(bytes=b"\x02", length=8)
GET_HEADER = BitArray(bytes=b"\x78", length=8)


# --- SANPO frame related ---
handshake_frame = b"AT+ET\r\n"
BYTE_SIZE = 8
SANPO_NORMAL_FRAME_HEADER = BitArray(bytes=b"\x53\x54", length=16)  # 2 bytes
DATA_LENGTH = BitArray(uint=8, length=8)  # 1 byte
SANPO_FRAME_TAIL = BitArray(bytes=b"\x0d\x0a", length=16)  # 2 bytes


@dataclass
class MotorState:
    id: int
    raw_position: float
    velocity: float
    torque: float
    filtered_position: float = 0.0


class SanpoProtocol:
    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, numBits: int) -> int:
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min

        span = x_max - x_min
        offset = x_min
        bitRange = 2**numBits - 1
        return int(((x - offset) * bitRange) / span)

    @staticmethod
    def uint_to_float(x_int: int, x_min: float, x_max: float, numBits: int) -> float:
        span = x_max - x_min
        offset = x_min
        bitRange = 2**numBits - 1
        return ((x_int * span) / bitRange) + offset

    @staticmethod
    def form_sanpo_frame(
        data_payload: BitArray, header: BitArray, motor_id: int, can_channel: int
    ) -> bytes:
        SANPO_write_BitArray = BitArray(
            uint=0, length=144
        )  # 18 bytes total frame size (144 bits)
        SANPO_write_BitArray.bin = (
            SANPO_NORMAL_FRAME_HEADER.bin
            + BitArray(uint=can_channel, length=8).bin  # CAN channel 1 byte
            + BitArray(uint=0, length=16).bin  # Reserved 2 bytes
            + header.bin
            + BitArray(uint=motor_id, length=8).bin  # motor id
            + DATA_LENGTH.bin  # data length
            + data_payload.bin  # data payload
            + SANPO_FRAME_TAIL.bin
        )
        return SANPO_write_BitArray.tobytes()

    @staticmethod
    def form_mit_payload(p, v, kp, kd, t):
        # Calculate integers
        p_int = SanpoProtocol.float_to_uint(p, P_MIN, P_MAX, 16)
        v_int = SanpoProtocol.float_to_uint(v, V_MIN, V_MAX, 12)
        kp_int = SanpoProtocol.float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = SanpoProtocol.float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t_int = SanpoProtocol.float_to_uint(t, T_MIN, T_MAX, 12)

        # Create NEW BitArrays locally so they never overwrite each other
        p_bits = BitArray(uint=p_int, length=16)
        v_bits = BitArray(uint=v_int, length=12)
        kp_bits = BitArray(uint=kp_int, length=12)
        kd_bits = BitArray(uint=kd_int, length=12)
        t_bits = BitArray(uint=t_int, length=12)

        # Combine them and return a new object
        return p_bits + v_bits + kp_bits + kd_bits + t_bits

    @staticmethod
    def get_motor_state_from_sanpo_frame(
        sanpo_response_frame: bytes, motor_id: int
    ) -> MotorState:

        # motor state object that stores the received data
        received_motor_state = MotorState(motor_id, 0.0, 0.0, 0.0, 0.0)

        sanpo_response_BitArray = BitArray(sanpo_response_frame)

        # 1. The payload starts exactly at Byte 8 (Index 64)
        payload_start_bit = 8 * BYTE_SIZE

        # MIT Motorstate response format (Total 6 bytes = 48 bits):
        # bits 1-8: None
        # bits 9-24: Position (16 bits)
        # bits 25-36: Velocity (12 bits)
        # bits 37-48: Torque (12 bits)

        # Position is the 2nd and 3rd bytes (16 bits)
        pos_start = payload_start_bit + 8
        pos_int = sanpo_response_BitArray[pos_start : pos_start + 16].uint

        # Velocity is the next 12 bits
        vel_start = pos_start + 16
        vel_int = sanpo_response_BitArray[vel_start : vel_start + 12].uint

        # Torque is the next 12 bits
        tau_start = vel_start + 12
        tau_int = sanpo_response_BitArray[tau_start : tau_start + 12].uint

        # 2. Convert raw integers to physical values
        pos = SanpoProtocol.uint_to_float(pos_int, P_MIN, P_MAX, 16)
        vel = SanpoProtocol.uint_to_float(vel_int, V_MIN, V_MAX, 12)
        tau = SanpoProtocol.uint_to_float(tau_int, T_MIN, T_MAX, 12)

        normalized_pos = (pos + math.pi) % (2 * math.pi) - math.pi

        # update the motor state in the dictionary
        received_motor_state.raw_position = pos
        received_motor_state.velocity = vel
        received_motor_state.torque = tau
        received_motor_state.filtered_position = normalized_pos

        logger.info(f"Motor ID: {motor_id}")
        logger.info(f"Current Position: {math.degrees(pos):.4f} deg")
        logger.info(f"Normalized Position: {math.degrees(normalized_pos):.4f} deg")
        logger.info(f"Velocity: {vel:.4f} rad/s")
        logger.info(f"Torque:   {tau:.4f} Nm")

        return received_motor_state

    @staticmethod
    def unpack_response_and_get_motor_states(
        response, motor_ids: list
    ) -> list[MotorState]:
        # ------------------ Thik whether to add delay or not
        time.sleep(0.01)  # DEBUG

        # length of respponse received
        logger.debug(f"Response length: {len(response)} bytes")
        motor_id_frame_received = {
            motor_id: False for motor_id in motor_ids
        }  # Track which motor IDs have received frames

        motor_states: list[MotorState] = []

        while True:
            if all(motor_id_frame_received.values()):
                break

            if len(response) < 18:  # Minimum length for a valid response
                logger.warning("Invalid response length")
                break

            # extract frame
            frame_end = response.rfind(SANPO_FRAME_TAIL.tobytes())
            if frame_end == -1:
                logger.warning("SANPO_FRAME_TAIL not found in response")
                break

            # check for the frame start bytes
            frame_start = frame_end - 16  # SANPO frame is 18 bytes long
            if (
                response[frame_start : frame_start + 2]
                != SANPO_NORMAL_FRAME_HEADER.tobytes()
            ):
                logger.warning("Invalid frame start bytes")
                break

            # extract the frame
            sanpo_response_frame = response[frame_start : frame_end + 2]

            # check for the motor ID in the extracted frame and extract motor state if it matches
            sanpo_response_BitArray = BitArray(sanpo_response_frame)

            # check the rx code of the MIT protocol
            rx_mit_header = sanpo_response_BitArray[
                5 * BYTE_SIZE + 4 : 6 * BYTE_SIZE + 4
            ].uint
            # logger.debug(f"Received MIT code: 0x{rx_mit_header:x}")

            if rx_mit_header != GET_HEADER.uint:
                response = response[
                    :frame_start
                ]  # Remove the processed frame and check for another one
                continue

            # Extract the last 4 bits for motor ID (since motor ID is 2 bits, it occupies the last 2 bits of the byte, and we need to ignore the preceding 6 bits)
            rx_motor_id = sanpo_response_BitArray[
                6 * BYTE_SIZE + 4 : 7 * BYTE_SIZE
            ].uint

            # check for the motor ID
            if rx_motor_id in motor_ids and not motor_id_frame_received[rx_motor_id]:
                logger.debug(f"Extracted frame: {sanpo_response_frame.hex(' ')}")
                recieved_motor_state = SanpoProtocol.get_motor_state_from_sanpo_frame(
                    sanpo_response_frame, rx_motor_id
                )  # update the motor state for the corresponding motor ID
                motor_id_frame_received[rx_motor_id] = True
                motor_states.append(recieved_motor_state)
            else:
                # check the another frame
                response = response[
                    :frame_start
                ]  # Remove the processed frame and check for another one

        return motor_states


class SanpoCommunication:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1000000,
        can_channel: int = 1,
        motor_ids: Optional[list[int]] = None,
    ):
        self.port_name = port
        self.baudrate = baudrate
        self.can_channel = can_channel
        self.motor_ids = motor_ids if motor_ids is not None else []
        self.serial_port = None

        # create motor states for each motor ID
        self.motor_states = {
            motor_id: MotorState(motor_id, 0.0, 0.0, 0.0) for motor_id in self.motor_ids
        }

    def connect_sanpo(
        self,
    ) -> bool:
        # try to connect to the serial port
        try:
            self.serial_port = Serial(self.port_name, self.baudrate, timeout=0.1)
        except Exception as e:
            logger.error(f"Error connecting to serial port {self.port_name}: {e}")
            return False

        # check if the serial port is open
        if not self.serial_port.is_open:
            logger.error(f"Failed to open serial port {self.port_name}.")
            return False

        # if it is successfully connected, send handshake frame
        self.__send_tx_frame(handshake_frame)
        # get response
        response = self.__get_rx_response(
            wait_time_s=1.0
        )  # Wait up to 1 second for the handshake response

        # check for "OK" in the received data
        if b"OK" in response:
            logger.info(f"sanpo board with {self.port_name} is connected successfully.")
            return True
        else:
            logger.error(
                f"Handshake failed, could not connect to SANPO board at port {self.port_name}."
            )
            self.serial_port.close()
            return False

    def disconnect_sanpo_and_motors(self):
        # disable motors before disconnecting
        self.disable_motors()
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info(f"Disconnected from SANPO board at port {self.port_name}.")
        else:
            logger.warning(f"Serial port at {self.port_name} was not open.")

    def enable_motors(self) -> list[int]:
        failed_motors = []
        for motor_id in self.motor_ids:
            self.serial_port.reset_input_buffer()  # Clear any existing data in the buffer

            enable_frame = SanpoProtocol.form_sanpo_frame(
                ENABLE_MOTOR_BIT_ARRAY, SET_HEADER, motor_id, self.can_channel
            )

            # send the enable command frame
            self.__send_tx_frame(enable_frame)
            time.sleep(0.01)  # DEBUG

            # get the first motor state received as we are checking the response for each motor ID separately here.
            received_motor_states = self.__get_motor_states_from_rx_response(
                wait_time_s=0.5
            )
            if len(received_motor_states) == 0:
                logger.info(
                    f"No response received for motor {motor_id}. Marking as failed."
                )
                failed_motors.append(motor_id)
            else:
                motor_states = received_motor_states[0]
                if motor_states.id == motor_id:
                    logger.info(f"Motor {motor_id} enabled successfully.")
                else:
                    logger.warning(
                        f"Received response for motor {motor_states.id} instead of motor {motor_id}. Marking as failed."
                    )
                    failed_motors.append(motor_id)

        if len(failed_motors) == 0:
            logger.info("All motors enabled successfully.")
            return []
        else:
            logger.error("Failed to enable all motors. Check logs for details.")
            return failed_motors

    def disable_motors(self) -> None:
        for motor_id in self.motor_ids:
            disable_frame = SanpoProtocol.form_sanpo_frame(
                DISABLE_MOTOR_BIT_ARRAY, MOVE_HEADER, motor_id, self.can_channel
            )
            self.__send_tx_frame(disable_frame)
            time.sleep(0.3)
            logger.info(f"Sent disable command to motor: {motor_id}")

    def set_zero_position(self) -> None:

        for motor_id in self.motor_ids:
            self.serial_port.reset_input_buffer()  # Clear any existing data in the buffer

            set_zero_pos_frame = SanpoProtocol.form_sanpo_frame(
                ENABLE_ZERO_POS_BIT_ARRAY, SET_HEADER, motor_id, self.can_channel
            )

            # send the enable command frame
            self.__send_tx_frame(set_zero_pos_frame)
            time.sleep(0.5)  # DEBUG

            # read motor states
            self.update_motor_states(wait_time_s=0.5)

    def write_motor_commands(
        self,
        target_joint_positions: list[float],
        target_joint_velocities: list[float],
        kp_values: list[float],
        kd_values: list[float],
        torque_values: list[float],
    ) -> None:
        self.serial_port.reset_input_buffer()  # Clear any existing data in the buffer

        motor_frames = {
            m_id: None for m_id in self.motor_ids
        }  # Store frames for each motor ID
        for motor_id, target_pos, target_vel, kp, kd, torque in zip(
            self.motor_ids,
            target_joint_positions,
            target_joint_velocities,
            kp_values,
            kd_values,
            torque_values,
        ):
            # Create the MIT payload for the target position with default values for velocity, kp, kd, and torque
            mit_payload = SanpoProtocol.form_mit_payload(
                target_pos, target_vel, kp, kd, torque
            )

            # Form the complete SANPO frame with the MIT payload
            move_frame = SanpoProtocol.form_sanpo_frame(
                mit_payload, MOVE_HEADER, motor_id, self.can_channel
            )

            motor_frames[motor_id] = move_frame

        # send frames for all motors
        for motor_id in self.motor_ids:
            self.__send_tx_frame(motor_frames[motor_id])

            ######################
            # DEBUG - add a small delay between sending commands and receiving responses to allow the SANPO board to process the command and respond

    def update_motor_states(self, wait_time_s=None) -> None:
        motor_states = self.__get_motor_states_from_rx_response(wait_time_s)

        if len(motor_states) == 0:
            return

        for state in motor_states:
            # update the motor state in the dictionary
            self.motor_states[state.id] = state

    def __send_tx_frame(self, tx_frame: bytes):
        self.serial_port.write(tx_frame)
        self.serial_port.flush()

        logger.debug(f"Sent frame: {tx_frame.hex(' ')}")

    def __get_rx_response(self, wait_time_s: float = None) -> bytes:
        if wait_time_s is None:
            waiting = self.serial_port.in_waiting
            if waiting:
                return self.serial_port.read(waiting)
            else:
                return b""
        else:
            deadline = time.monotonic() + wait_time_s
            last_data_time = time.monotonic()
            chunks = []
            while time.monotonic() < deadline:
                waiting = self.serial_port.in_waiting
                data = self.serial_port.read(waiting or 1)
                if data:
                    chunks.append(data)
                    last_data_time = time.monotonic()
                elif chunks and time.monotonic() - last_data_time > 0.2:
                    break

            full_response = b"".join(chunks)
            return full_response

    def __get_motor_states_from_rx_response(self, wait_time_s=None) -> list[MotorState]:
        response = self.__get_rx_response(wait_time_s)

        if not response:
            return []

        motor_states: list[MotorState] = (
            SanpoProtocol.unpack_response_and_get_motor_states(response, self.motor_ids)
        )

        if len(motor_states) == 0:
            return []

        return motor_states


# -------------Findings and TODOs----------------
# by adding a little sleep between sending and receiving, the rx buffer is able to capture the response for the enable command.
