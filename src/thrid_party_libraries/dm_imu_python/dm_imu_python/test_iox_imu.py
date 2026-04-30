#!/usr/bin/env python3
"""
Test Subscriber for IMU Data
Subscribes to iceoryx2 topic published by DmImuNode
"""

import iceoryx2 as iox2
import time

# Import the same data structure you defined
from iox2_msgs.iox2_imu_msgs import IoxImuData

def main():
    print("📡 IMU Test Subscriber Started")
    print("Connecting to: iox/imu_data")
    print("Waiting for IMU data...\n")
    
    # Optional: suppress debug logs
    iox2.set_log_level_from_env_or(iox2.LogLevel.Warn)
    
    # 1. Create node
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    
    # 2. Connect to the same service your IMU node is publishing to
    service = (
        node.service_builder(iox2.ServiceName.new("iox/imu_data"))
        .publish_subscribe(IoxImuData)
        .open_or_create()
    )
    
    # 3. Create subscriber
    subscriber = service.subscriber_builder().create()
    
    print("✅ Subscriber connected!")
    print("Listening for IMU data (Ctrl+C to exit)\n")
    
    # Statistics
    msg_count = 0
    last_print_time = time.time()
    
    try:
        while True:
            # Non-blocking receive
            sample = subscriber.receive()
            
            if sample is not None:
                # Extract data
                imu_data = sample.payload().contents
                msg_count += 1
                
                # Display data
                print(f"📊 IMU Data Received:")
                print(f"   Roll:  {imu_data.roll:.4f} rad ({imu_data.roll * 180 / 3.14159:.2f}°)")
                print(f"   Pitch: {imu_data.pitch:.4f} rad ({imu_data.pitch * 180 / 3.14159:.2f}°)")
                print(f"   Yaw:   {imu_data.yaw:.4f} rad ({imu_data.yaw * 180 / 3.14159:.2f}°)")
                print(f"   Messages received: {msg_count}")
                print("-" * 40)
                
            else:
                # No data available yet, sleep briefly
                node.wait(iox2.Duration.from_millis(10))
                

                
    except KeyboardInterrupt:
        print(f"\n🛑 Subscriber stopped. Total messages: {msg_count}")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    main()