# Bipedal Robot Workspace

## Overview

This workspace contains a complete ROS 2-based software stack for a bipedal robot, providing support for simulation, control, and real-world operation. The stack includes packages for robot description, control, hardware interface, teleoperation, and state management.

## Packages

### 1. biped_bringup
- **Description**: Contains launch files for bringing up the robot in different environments
- **Key Features**:
  - Launch files for Gazebo simulation
  - Launch files for IsaacSim simulation
  - Launch files for real robot operation
  - GUI sliders for joint control
- **Launch Files**:
  - `biped_bringup_gazebo.launch.py` - Launch robot in Gazebo
  - `biped_bringup_isaacsim.launch.py` - Launch robot in IsaacSim
  - `biped_bringup_real_robot_with_position_control.launch.py` - Launch real robot with position control
  - `biped_bringup_real_robot_with_trajectory_control.launch.py` - Launch real robot with trajectory control

### 2. biped_control
- **Description**: Custom ROS 2 controllers for the bipedal robot
- **Key Features**:
  - Blind walk controller implementation
  - Position control
  - Trajectory control
- **Controllers**:
  - `blind_walk_controller` - A controller for basic walking functionality

### 3. biped_description
- **Description**: Robot URDF models and visualization files
- **Key Components**:
  - URDF and Xacro files for different environments
  - Mesh files for visualization
  - RViz configuration files
- **Models**:
  - `biped.urdf` - Basic robot model
  - `biped_gazebo_ros2_control.urdf` - Model for Gazebo with ROS 2 control
  - `biped_isaacsim_ros2_control.urdf` - Model for IsaacSim with ROS 2 control
  - `biped_real_ros2_control.urdf` - Model for real robot with ROS 2 control

### 4. biped_fsm
- **Description**: Finite State Machine for robot state management
- **Key Features**:
  - State machine implementation using YASMIN
  - Shared memory structures for inter-process communication
  - State transition management
- **Components**:
  - Python state machine implementation
  - C++ shared memory structures
  - State manager node

### 5. biped_hardware_interface
- **Description**: Hardware interface for real robot communication
- **Key Features**:
  - Multi-threaded and single-threaded hardware interface implementations
  - Serial communication with robot hardware
  - Motor control and feedback
- **Interfaces**:
  - `biped_sanpo_hardware_interface_multi_threaded`
  - `biped_sanpo_hardware_interface_single_threaded`

### 6. biped_isaacsim
- **Description**: Robot description for IsaacSim environment
- **Key Components**:
  - USD files for IsaacSim
  - Robot configuration files

### 7. biped_msgs
- **Description**: Custom ROS 2 messages for the bipedal robot
- **Messages**:
  - `BipedJointCommand.msg` - Joint command message
  - `BipedShmData.msg` - Shared memory data message
  - `Example.msg` - Example message

### 8. biped_teleop
- **Description**: Teleoperation functionality for the robot
- **Key Features**:
  - Joystick control
  - Teleoperation node
  - Joy state node

### 9. dm_imu
- **Description**: IMU sensor interface
- **Key Features**:
  - IMU data publishing
  - Serial communication with IMU sensor

### 10. test_pkg
- **Description**: Test utilities and scripts
- **Key Components**:
  - Dummy IMU shared memory
  - Policy runner
  - Shared memory readers

### 11. thrid_party_libraries
- **Description**: Third-party libraries used in the project
- **Libraries**:
  - `serial-ros2` - Serial communication library

## System Architecture

The system architecture consists of several layers:

1. **Hardware Layer**: `biped_hardware_interface` provides the interface to the physical robot
2. **Control Layer**: `biped_control` implements the control algorithms
3. **State Management**: `biped_fsm` manages the robot's state transitions
4. **Communication Layer**: `biped_msgs` defines the message formats
5. **User Interface**: `biped_teleop` and `biped_bringup` provide user interaction
6. **Simulation Layer**: `biped_bringup` and `biped_isaacsim` provide simulation environments

## Prerequisites

- ROS 2 Humble Hawksbill or later
- Gazebo (for simulation)
- IsaacSim (optional, for high-fidelity simulation)
- Python 3.8+
- C++17 compatible compiler
- Dependencies specified in each package's `package.xml`

## Installation

1. **Clone the repository**:
   ```bash
   cd ~/workspaces
   git clone <repository-url> biped_ws
   cd biped_ws
   ```

2. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Usage

### Simulation in Gazebo

```bash
ros2 launch biped_bringup biped_bringup_gazebo_with_control.launch.py
```

### Simulation in IsaacSim

```bash
ros2 launch biped_bringup biped_bringup_isaacsim_with_control.launch.py
```

### Real Robot Operation

```bash
# With position control
ros2 launch biped_bringup biped_bringup_real_robot_with_position_control.launch.py

# With trajectory control
ros2 launch biped_bringup biped_bringup_real_robot_with_trajectory_control.launch.py
```

### Teleoperation

```bash
ros2 launch biped_teleop biped_teleop_launch.py
```

### Blind Walk Controller

```bash
ros2 launch biped_control blind_walk_controller.launch.py
```

## Development

### Adding a New Controller

1. Create a new controller class in `biped_control/src/`
2. Update the `biped_control.xml` plugin file
3. Create a launch file for the new controller
4. Build and test the controller

### Modifying the Robot Description

1. Edit the URDF/Xacro files in `biped_description/urdf/`
2. Update mesh files in `biped_description/meshes/` if needed
3. Rebuild the workspace

### Extending the State Machine

1. Add new states in `biped_fsm/biped_fsm/`
2. Update the state transition XML file
3. Rebuild the workspace

## Testing

Run the tests for each package:

```bash
colcon test --packages-select <package-name>
```

## Contributors

- Bachala Rajesh
- mira

## License

TODO: License declaration

## Acknowledgments

- YASMIN for finite state machine implementation
- ROS 2 control framework
- serial-ros2 library for serial communication