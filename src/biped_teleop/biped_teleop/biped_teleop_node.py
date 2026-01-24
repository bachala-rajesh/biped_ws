#!/usr/bin/env python3
#TODO: deadzone handling
#TODO: implement velocity smoothing
#TODO: button debounce handling
#TODO: error handling in parameter loading
#TODO: docstring in all functions


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Biped States struct
class BIPEDSTATE:
    SIT = "SIT"
    STAND = "STAND"
    EMERGENCY = "EMERGENCY"
    START = "START"



class BipedTeleopNode(Node):

    def __init__(self):
        super().__init__(
            "biped_teleop_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
            )
        self.get_logger().info("Biped Teleop Node has been started.")
        
        # intialize joystick mapping parameters
        self.hw_axes = {}
        self.hw_buttons = {}
        self.scales = {}
        self.cmd_map = {}
        
        # important IDS
        self.deadmen_id = None
        self.stand_id = None
        self.sit_id = None
        self.joy_walk_x_id = None
        self.joy_walk_y_id = None
        self.button_walk_x_id = None
        self.button_walk_y_id = None
        self.turn_id = None
        self.emergency_stop_id = None
        self.restart_id = None
        
        # default 
        self.pose = BIPEDSTATE.START
        self.emergency_status = False
        
        # read parameters
        self.load_joystick_layout()
        self.load_control_map()
        self.load_scales()
        
        # map important IDS
        self.map_IDS()
        
        
        # Topics
        self.joy_subscriber = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            10
        )
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_publisher = self.create_publisher(String, "high_level_command", 10)
        
        #Logging
        self.get_logger().info("Biped Teleop node Ready")
        
    def load_joystick_layout(self):
        """
        Reads everything under "joy_layout" in the yaml file
        """
        joy_params = self.get_parameters_by_prefix("joy_layout")
        
        for name, param_obj in joy_params.items():
            #split names
            split_name = name.split('.')
            if len(split_name) < 2: continue
            
            category = split_name[0] # "axes" or "buttons"
            key = split_name[1]      # The name of the axis/button
            val = param_obj.value    # The ID number 

            if category == "axes":
                self.hw_axes[key] = val
            elif category == "buttons":
                self.hw_buttons[key] = val
            
        
    
    def load_control_map(self):
        """
        Read control map ans link it to actual IDs
        """
        ctrl_params = self.get_parameters_by_prefix("control_map")
        
        for name, param_object in ctrl_params.items():
            action_name = name
            physical_name = param_object.value
            
            # Find the ID for this physical_name
            real_id = None
            if physical_name in self.hw_axes:
                real_id = self.hw_axes[physical_name]
            elif physical_name in self.hw_buttons:
                real_id = self.hw_buttons[physical_name]
            
                
            # store the real id
            if real_id is not None:
                self.cmd_map[action_name] = real_id
                self.get_logger().info(f"Mapped action '{action_name}' -> ID {real_id} ('{physical_name}')")
            else:
                self.get_logger().warn(f"Physical control '{physical_name}' not found in axes or buttons.")
                
            
            
    
    def load_scales(self):
        """
        Read scale parameters for different controls
        """
        scale_params = self.get_parameters_by_prefix("scales")
        
        for scale_name, param_object in scale_params.items():
            self.scales[scale_name] = param_object.value
  
  
    def map_IDS(self):
        """
        Maps important IDS to class variables for easy access
        """
        self.deadmen_id = self.cmd_map.get("deadman_switch")
        self.stand_id = self.cmd_map.get("stand_cmd")
        self.sit_id = self.cmd_map.get("sit_cmd")
        self.emergency_stop_id = self.cmd_map.get("emergency_stop_cmd")
        self.restart_id = self.cmd_map.get("restart_cmd")
        self.joy_walk_x_id = self.cmd_map.get("joy_walk_x")
        self.joy_walk_y_id = self.cmd_map.get("joy_walk_y")
        self.button_walk_x_id = self.cmd_map.get("button_walk_x")
        self.button_walk_y_id = self.cmd_map.get("button_walk_y")
        self.turn_id = self.cmd_map.get("turn_z")
        
    

    def joy_callback(self, msg):
        """
        Runs every time the joystick sends data.
        """
        # check emergency status
        if self.emergency_status:
            # check for restart command
            if self.restart_id is not None and msg.buttons[self.restart_id] == 1:
                self.get_logger().info("Restarting from Emergency Stop.")
                self.emergency_status = False
                self.pose = BIPEDSTATE.START
            else:
                self.vel_publisher.publish(Twist())  # Publish zero velocities
                self.cmd_publisher.publish(String(data=self.pose))
                self.get_logger().warn("Emergency Stop Activated!")
            return
        
        # check emergency stop
        if self.emergency_stop_id is not None and msg.buttons[self.emergency_stop_id] == 1:
            self.get_logger().warn("Emergency Stop Activated!")
            self.vel_publisher.publish(Twist())  # Publish zero velocities
            self.pose = BIPEDSTATE.EMERGENCY
            self.cmd_publisher.publish(String(data=self.pose))
            self.emergency_status = True
            return
        
        #deadman switch check
        if self.deadmen_id is not None and msg.buttons[self.deadmen_id] == 0:
            self.vel_publisher.publish(Twist())  # Publish zero velocities
            return
        
        # check pose commands
        if self.stand_id is not None and msg.buttons[self.stand_id] == 1:
            self.pose = BIPEDSTATE.STAND
        elif self.sit_id is not None and msg.buttons[self.sit_id] == 1:
            self.pose = BIPEDSTATE.SIT
            
        # create velocity command
        vel_cmd = Twist()
        # continous velocity value
        if self.joy_walk_x_id is not None:
            vel_cmd.linear.x = msg.axes[self.joy_walk_x_id] * self.scales.get("walk_x_scale", 1.0)
        if self.joy_walk_y_id is not None:
            vel_cmd.linear.y = msg.axes[self.joy_walk_y_id] * self.scales.get("walk_y_scale", 1.0)
        if self.turn_id is not None:
            vel_cmd.angular.z = msg.axes[self.turn_id] * self.scales.get("turn_z_scale", 1.0)
        
        # discrete velocity value. This will override the joystick value if both are set
        if self.button_walk_x_id is not None and msg.axes[self.button_walk_x_id] != 0:
            vel_cmd.linear.x = msg.axes[self.button_walk_x_id] * self.scales.get("walk_x_scale", 1.0)
        if self.button_walk_y_id is not None and msg.axes[self.button_walk_y_id] != 0:
            vel_cmd.linear.y = msg.axes[self.button_walk_y_id] * self.scales.get("walk_y_scale", 1.0)
        
        
        # publish velocity and high level command
        self.vel_publisher.publish(vel_cmd)
        self.cmd_publisher.publish(String(data=self.pose))
            
        
    


def main(args=None):
    rclpy.init(args=args)

    biped_teleop_node = BipedTeleopNode()

    try:
        rclpy.spin(biped_teleop_node)
    except KeyboardInterrupt:
        pass

    biped_teleop_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
