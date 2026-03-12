#!/usr/bin/env python3

import time
from multiprocessing import shared_memory, resource_tracker
import ctypes

from biped_fsm import biped_shared_memory_structs
from biped_fsm.robot_states_enum import RobotState

def safe_read_state(state_struct):
    """Safely reads the FSM state using sequence locks."""
    while True:
        seq1 = state_struct.seq_counter
        # If odd, writer is busy. Spin and wait.
        if seq1 % 2 != 0:
            continue
            
        # Copy the data
        state_val = state_struct.current_state
        
        seq2 = state_struct.seq_counter
        # If they match, the read was clean.
        if seq1 == seq2:
            return state_val

def safe_read_imu(imu_struct):
    """Safely reads the IMU orientation using sequence locks."""
    while True:
        seq1 = imu_struct.seq_counter
        if seq1 % 2 != 0:
            continue
            
        roll = imu_struct.orientation[0]
        pitch = imu_struct.orientation[1]
        yaw = imu_struct.orientation[2]
        
        seq2 = imu_struct.seq_counter
        if seq1 == seq2:
            return roll, pitch, yaw

def safe_read_joints(joints_struct):
    """Safely reads the joint positions using sequence locks."""
    while True:
        seq1 = joints_struct.seq_counter
        if seq1 % 2 != 0:
            continue
            
        l_hip_pitch = joints_struct.position[0]
        l_knee = joints_struct.position[2]
        
        seq2 = joints_struct.seq_counter
        if seq1 == seq2:
            return l_hip_pitch, l_knee
        
def safe_read_joy_cmd(joy_cmd_struct):
    """Safely reads the joystick command using sequence locks."""
    while True:
        seq1 = joy_cmd_struct.seq_counter
        if seq1 % 2 != 0:
            continue
            
        linear_x = joy_cmd_struct.linear_x
        linear_y = joy_cmd_struct.linear_y
        angular_z = joy_cmd_struct.angular_z
        
        seq2 = joy_cmd_struct.seq_counter
        if seq1 == seq2:
            return linear_x, linear_y, angular_z

def main():
    print("[INFO] Connecting to shared memory 'biped_shm'...")
    try:
        shm = shared_memory.SharedMemory(name="biped_shm", create=False)
        resource_tracker.unregister(shm._name, "shared_memory")
    except FileNotFoundError:
        print("[ERROR] 'biped_shm' not found. Is your node running?")
        return

    shared_data = biped_shared_memory_structs.BipedSharedMemory.from_buffer(shm.buf)
    print("[INFO] Connected! Reading live safe data...\n")
    
    try:
        while True:
            # 1. Safely read all structs
            state_val = safe_read_state(shared_data.state)
            roll, pitch, yaw = safe_read_imu(shared_data.imu)
            l_hip_pitch, l_knee = safe_read_joints(shared_data.joints)
            linear_x, linear_y, angular_z = safe_read_joy_cmd(shared_data.cmd_vel)

            # 2. Parse state enum
            try:
                state_name = RobotState(state_val).name
            except ValueError:
                state_name = f"UNKNOWN ({state_val})"

            # 3. Print
            print(f"--- FSM STATE: {state_name} ---")
            print(f"IMU (rad)  -> Roll: {roll:.3f} | Pitch: {pitch:.3f} | Yaw: {yaw:.3f}")
            print(f"Joints (rad)-> L_Hip_P: {l_hip_pitch:.3f} | L_Knee: {l_knee:.3f}")
            print(f"Joy Cmd     -> Linear X: {linear_x:.3f} | Linear Y: {linear_y:.3f} | Angular Z: {angular_z:.3f}")
            print("-" * 40)
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n[INFO] Stopping test reader...")
    finally:
        if 'shared_data' in locals():
            del shared_data
        shm.close()
        print("[INFO] Disconnected safely.")

if __name__ == "__main__":
    main()