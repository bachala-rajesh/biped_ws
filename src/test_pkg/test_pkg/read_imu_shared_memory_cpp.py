#! /usr/bin/env python3

"""
Read IMU data from shared memory. th shared mory data is implemented in dummy_imu_shared_memory.cpp
"""

import mmap
import struct
import time
import os

# confgiuration
SHM_NAME = "/dev/shm/robot_control_shm"
CACHE_LINE = 64
TOTAL_SIZE = 3 * CACHE_LINE  # 192 bytes


# offset
OFFSET_HEADER = 0
OFFSET_IMU_SEQ = 1 * CACHE_LINE  # 64 bytes
OFFSET_IMU_DATA = OFFSET_IMU_SEQ + 4  # 68 bytes


def read_imu_safely(mm):
    """Read IMU data from shared memory safely."""

    mm.seek(OFFSET_IMU_SEQ)

    # read seq (start)
    seq_bytes_start = mm.read(4)
    seq_start = struct.unpack("I", seq_bytes_start)[0]

    # check is writer busy (if seq is odd, writer is busy)
    if seq_start % 2 != 0:
        return None

    # read imu data (4 floats)
    data_bytes = mm.read(16)
    imu_data = struct.unpack("4f", data_bytes)

    # read seq(end)
    mm.seek(OFFSET_IMU_SEQ)
    seq_bytes_end = mm.read(4)
    seq_end = struct.unpack("I", seq_bytes_end)[0]

    # check if seq is consistent
    if seq_end != seq_start:
        return None, "Sequence is inconsistent"

    return imu_data


def main():
    print(f"Waiting for Shared Memory at {SHM_NAME}...")

    # Wait loop until C++ node starts and creates the file
    while not os.path.exists(SHM_NAME):
        time.sleep(0.5)

    print("Found Shared Memory! Connecting...")

    try:
        # Open the file in Read-Only mode
        with open(SHM_NAME, "r+b") as f:
            # Memory Map the file
            mm = mmap.mmap(f.fileno(), TOTAL_SIZE)

            while True:
                # Attempt to read
                data = read_imu_safely(mm)

                if data is not None:
                    x, y, z, w = data
                    # Print formatted output
                    print(
                        f"Safe Read | IMU: x={x:.3f}, y={y:.3f}, z={z:.3f}, w={w:.3f}"
                    )
                else:
                    # This happens if we caught the C++ writer mid-write
                    print("(!) Seqlock Collision - Retrying...")

                # Run at approx 50Hz (Policy Rate)
                time.sleep(0.02)

            mm.close()

    except FileNotFoundError:
        print("Error: Shared memory file not found. Is the C++ node running?")
    except KeyboardInterrupt:
        print("\nStopping reader.")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
