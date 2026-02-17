import time
import struct
from multiprocessing import shared_memory

def main():
    # 1. Configuration (Must match the Writer!)
    shm_name = 'imu_orientation_shm'
    data_size = 16  # 4 floats * 4 bytes each
    
    print(f"Waiting for Shared Memory: '{shm_name}'...")

    # 2. Connect to the Shared Memory
    shm = None
    while shm is None:
        try:
            # Try to attach to the existing memory block
            shm = shared_memory.SharedMemory(name=shm_name)
            print(f"Connected to '{shm_name}'!")
        except FileNotFoundError:
            # If writer hasn't started yet, wait and retry
            time.sleep(0.5)

    try:
        while True:
            # 3. Read raw bytes from the buffer
            # We read exactly 16 bytes (size of 4 floats)
            bytes_data = shm.buf[:data_size]
            
            # 4. Unpack bytes back into numbers
            # '4f' unpacks 4 floats (x, y, z, w)
            x, y, z, w = struct.unpack('4f', bytes_data)
            
            # 5. Display
            # \r overwrites the current line so the terminal doesn't scroll like crazy
            print(f"Read from RAM -> x: {x: .2f} | y: {y: .2f} | z: {z: .2f} | w: {w: .2f}", end='\r')
            
            # Read at 50Hz (0.02s) to simulate a fast controller
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping Reader...")
    finally:
        # 6. Cleanup
        if shm:
            shm.close()  # Close the connection
            # Do NOT call unlink() here. The Writer (Publisher) owns the memory.
            print("Connection closed.")

if __name__ == "__main__":
    main()