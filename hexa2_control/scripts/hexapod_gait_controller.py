#!/usr/bin/env python3
# hexapod_gait_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import time
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class HexapodGaitController(Node):
    def __init__(self):
        super().__init__('hexapod_gait_controller')
        
        # Robot dimensions
        self.body_length = 0.22  # Length of the body (front to back)
        self.body_width = 0.16   # Width of the body (left to right)
        self.stand_height = 0.10  # Default standing height
        
        # Define leg positions relative to body center (same as in kinematics node)
        self.leg_positions = {
            'front_left':  [ 0.110,  0.080, 0.0],  # FL
            'middle_left': [ 0.000,  0.130, 0.0],  # ML
            'back_left':   [-0.110,  0.080, 0.0],  # BL
            'back_right':  [-0.110, -0.080, 0.0],  # BR
            'middle_right':[ 0.000, -0.130, 0.0],  # MR
            'front_right': [ 0.110, -0.080, 0.0],  # FR
        }
        
        # Gait parameters
        self.step_height = 0.04  # Height of leg lift during step
        self.stride_length = 0.08  # Length of stride
        self.gait_frequency = 1.0  # Cycles per second
        self.current_gait = 'tripod'  # Default gait
        
        # Define leg groupings for different gaits
        self.tripod_groups = [
            ['front_left', 'middle_right', 'back_left'],  # Group 1
            ['front_right', 'middle_left', 'back_right']   # Group 2
        ]
        
        self.ripple_groups = [
            ['front_left', 'back_right'],   # Group 1
            ['middle_left', 'middle_right'],  # Group 2
            ['back_left', 'front_right']     # Group 3
        ]
        
        self.wave_sequence = [
            'front_right',
            'middle_right',
            'back_right',
            'back_left',
            'middle_left',
            'front_left'
        ]
        
        # Publishers for each leg
        self.leg_publishers = {}
        for leg in self.leg_positions.keys():
            self.leg_publishers[leg] = self.create_publisher(
                Point,
                f'/hexapod/leg/{leg}/target_position',
                10
            )
        
        # Create a subscriber for velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Variables to store current movement commands
        self.linear_vel_x = 0.0  # Forward/backward
        self.linear_vel_y = 0.0  # Left/right
        self.angular_vel_z = 0.0  # Rotation
        
        # Timer for gait generation
        self.gait_timer = self.create_timer(0.02, self.gait_step)  # 50Hz update rate
        
        # Time tracking
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.is_moving = False
        self.cycle_phase = 0.0  # 0.0 to 1.0 representing where in the gait cycle we are
        
        # Standing offsets for each leg (neutral position)
        self.standing_offsets = {leg: [0.0, 0.0, -self.stand_height] for leg in self.leg_positions.keys()}
        
        self.get_logger().info('Hexapod Gait Controller initialized')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for handling velocity commands
        
        Args:
            msg (Twist): Velocity command
        """
        # Store the velocity commands
        self.linear_vel_x = msg.linear.x
        self.linear_vel_y = msg.linear.y
        self.angular_vel_z = msg.angular.z
        
        # Check if we should be moving
        self.is_moving = abs(self.linear_vel_x) > 0.01 or abs(self.linear_vel_y) > 0.01 or abs(self.angular_vel_z) > 0.01
        
        if self.is_moving:
            self.get_logger().info(f"Moving with vx={self.linear_vel_x:.2f}, vy={self.linear_vel_y:.2f}, wz={self.angular_vel_z:.2f}")
        
    def set_gait(self, gait_name):
        """
        Set the current gait pattern
        
        Args:
            gait_name (str): Name of the gait ('tripod', 'ripple', 'wave')
        """
        if gait_name in ['tripod', 'ripple', 'wave']:
            self.current_gait = gait_name
            self.get_logger().info(f"Switched to {gait_name} gait")
        else:
            self.get_logger().warning(f"Unknown gait: {gait_name}")
    
    def get_leg_phase(self, leg_name):
        """
        Get the phase (0.0 to 1.0) for a specific leg in the current gait
        
        Args:
            leg_name (str): Name of the leg
            
        Returns:
            float: Phase value from 0.0 to 1.0
        """
        if self.current_gait == 'tripod':
            # Determine which group the leg is in
            if leg_name in self.tripod_groups[0]:
                return self.cycle_phase
            else:
                return (self.cycle_phase + 0.5) % 1.0
        
        elif self.current_gait == 'ripple':
            # Determine which group the leg is in
            if leg_name in self.ripple_groups[0]:
                return self.cycle_phase
            elif leg_name in self.ripple_groups[1]:
                return (self.cycle_phase + 0.33) % 1.0
            else:
                return (self.cycle_phase + 0.67) % 1.0
        
        elif self.current_gait == 'wave':
            # Get the index in the sequence
            leg_index = self.wave_sequence.index(leg_name)
            # Each leg has a phase offset based on its position in the sequence
            return (self.cycle_phase + leg_index/6.0) % 1.0
            
        return 0.0
    
    def calculate_foot_position(self, leg_name, phase):
        """
        Calculate foot position for a leg based on the current phase
        
        Args:
            leg_name (str): Name of the leg
            phase (float): Current phase of the leg (0.0 to 1.0)
            
        Returns:
            list: Target foot position [x, y, z]
        """
        # Get the base position of the leg
        base_pos = self.leg_positions[leg_name]
        
        # Apply standing offset (neutral position)
        offset = self.standing_offsets[leg_name]
        
        # If we're not moving, just return the standing position
        if not self.is_moving:
            return [base_pos[0] + offset[0], base_pos[1] + offset[1], offset[2]]
        
        # Calculate movement vector based on velocity commands
        # Forward/backward component (X axis)
        movement_x = self.linear_vel_x
        
        # Left/right component (Y axis)
        movement_y = self.linear_vel_y
        
        # Rotation component (around Z)
        # For rotation, each leg moves perpendicular to the line from the center
        leg_angle = math.atan2(base_pos[1], base_pos[0])
        rot_x = -self.angular_vel_z * base_pos[1]  # -r * sin(theta) * omega
        rot_y = self.angular_vel_z * base_pos[0]   # r * cos(theta) * omega
        
        # Combine linear and rotational movements
        total_x = movement_x + rot_x
        total_y = movement_y + rot_y
        
        # Normalize the direction if needed
        movement_mag = math.sqrt(total_x**2 + total_y**2)
        if movement_mag > 0.001:  # Non-zero movement
            normalized_x = total_x / movement_mag
            normalized_y = total_y / movement_mag
        else:
            normalized_x = 0.0
            normalized_y = 0.0
        
        # Calculate stride vector
        stride_x = normalized_x * self.stride_length
        stride_y = normalized_y * self.stride_length
        
        # Based on the phase, calculate where in the step cycle this leg should be
        # Support phase (foot on ground, moving backwards): phase 0.5-1.0
        # Swing phase (foot in air, moving forwards): phase 0.0-0.5
        
        if phase < 0.5:  # Swing phase (moving foot forward in the air)
            # Normalize to 0-1 range for swing phase
            swing_phase = phase * 2.0
            
            # X and Y move from -stride/2 to +stride/2
            x_offset = stride_x * (swing_phase - 0.5)
            y_offset = stride_y * (swing_phase - 0.5)
            
            # Z follows a sine curve for lift
            z_offset = -self.stand_height + self.step_height * math.sin(swing_phase * math.pi)
            
        else:  # Support phase (foot on ground, moving backward)
            # Normalize to 0-1 range for support phase
            support_phase = (phase - 0.5) * 2.0
            
            # X and Y move from +stride/2 to -stride/2
            x_offset = stride_x * (0.5 - support_phase)
            y_offset = stride_y * (0.5 - support_phase)
            
            # Z is at the standing height (foot on ground)
            z_offset = -self.stand_height
        
        # Combined foot position
        foot_x = base_pos[0] + offset[0] + x_offset
        foot_y = base_pos[1] + offset[1] + y_offset
        foot_z = offset[2] + z_offset
        
        return [foot_x, foot_y, foot_z]
    
    def gait_step(self):
        """
        Execute a step in the gait cycle
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        # Update cycle phase based on elapsed time and gait frequency
        if self.is_moving:
            self.cycle_phase = (elapsed * self.gait_frequency) % 1.0
        
        # Process each leg
        for leg_name in self.leg_positions.keys():
            # Get the phase for this leg in the current gait
            leg_phase = self.get_leg_phase(leg_name)
            
            # Calculate target foot position
            foot_position = self.calculate_foot_position(leg_name, leg_phase)
            
            # Publish target position
            point_msg = Point()
            point_msg.x = foot_position[0]
            point_msg.y = foot_position[1]
            point_msg.z = foot_position[2]
            
            self.leg_publishers[leg_name].publish(point_msg)
    
    def move_to_stand_position(self):
        """
        Move all legs to the default standing position
        """
        for leg_name in self.leg_positions.keys():
            base_pos = self.leg_positions[leg_name]
        
            point_msg = Point()
            point_msg.x = base_pos[0]
            point_msg.y = base_pos[1]
            # Set a fixed height for stability
            point_msg.z = -0.10  # 10cm below the body
        
            self.leg_publishers[leg_name].publish(point_msg)
            
        self.get_logger().info('Moved to standing position')

def main(args=None):
    rclpy.init(args=args)
    node = HexapodGaitController()
    
    # First move to standing position
    node.move_to_stand_position()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Move to a safe position before shutting down
        node.move_to_stand_position()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()