#!/usr/bin/env python3
# hexapod_gait_controller_enhanced.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import time
from enum import Enum, auto

class GaitType(Enum):
    TRIPOD = auto()
    RIPPLE = auto()
    WAVE = auto()

class RobotState(Enum):
    STANDING = auto()
    MOVING = auto()
    ADAPTIVE = auto()
    TURNING = auto()
    CLIMBING = auto()

class HexapodGaitController(Node):
    def __init__(self):
        super().__init__('hexapod_gait_controller')
        
        # Robot dimensions and parameters
        self.body_length = 0.22  # Length of the body (front to back)
        self.body_width = 0.16   # Width of the body (left to right)
        self.stand_height = 0.10  # Default standing height
        
        # Define leg positions relative to body center
        self.leg_positions = {
            'front_left':  [ 0.110,  0.080, 0.0],  # FL
            'middle_left': [ 0.000,  0.130, 0.0],  # ML
            'back_left':   [-0.110,  0.080, 0.0],  # BL
            'back_right':  [-0.110, -0.080, 0.0],  # BR
            'middle_right':[ 0.000, -0.130, 0.0],  # MR
            'front_right': [ 0.110, -0.080, 0.0],  # FR
        }
        
        # Gait parameters
        self.step_height = 0.04    # Height of leg lift during step
        self.stride_length = 0.08  # Length of stride
        self.gait_frequency = 1.0  # Cycles per second
        self.current_gait = GaitType.TRIPOD  # Default gait
        self.current_state = RobotState.STANDING
        
        # Define leg groupings for different gaits
        self.tripod_groups = [
            ['front_left', 'middle_right', 'back_left'],  # Group 1
            ['front_right', 'middle_left', 'back_right']   # Group 2
        ]
        
        self.ripple_groups = [
            ['front_left', 'back_right'],     # Group 1
            ['middle_left', 'middle_right'],  # Group 2
            ['back_left', 'front_right']      # Group 3
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
        
        # Create a subscriber for IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/hexapod/imu/data',
            self.imu_callback,
            10
        )
        
        # Create a service for changing gait type
        self.gait_service = self.create_subscription(
            String,
            '/hexapod/set_gait',
            self.set_gait_callback,
            10
        )
        
        # Variables to store current movement commands
        self.linear_vel_x = 0.0  # Forward/backward
        self.linear_vel_y = 0.0  # Left/right
        self.angular_vel_z = 0.0  # Rotation
        
        # Variables to store IMU data
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.has_imu_data = False
        
        # Body posture adjustment for balance
        self.body_roll_adjustment = 0.0
        self.body_pitch_adjustment = 0.0
        
        # For terrain adaptation
        self.terrain_height_map = {leg: 0.0 for leg in self.leg_positions.keys()}
        self.enable_terrain_adaptation = False
        
        # Timer for gait generation
        self.gait_timer = self.create_timer(0.02, self.gait_step)  # 50Hz update rate
        
        # Time tracking
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.is_moving = False
        self.cycle_phase = 0.0  # 0.0 to 1.0 representing where in the gait cycle we are
        
        # Standing offsets for each leg (neutral position)
        self.standing_offsets = {leg: [0.0, 0.0, -self.stand_height] for leg in self.leg_positions.keys()}
        
        # For path planning and navigation
        self.target_position = None  # Target position to move to
        self.enable_path_planning = False
        
        self.get_logger().info('Enhanced Hexapod Gait Controller initialized')
    
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
        moving = abs(self.linear_vel_x) > 0.01 or abs(self.linear_vel_y) > 0.01 or abs(self.angular_vel_z) > 0.01
        
        if moving and not self.is_moving:
            self.is_moving = True
            self.current_state = RobotState.MOVING
            self.get_logger().info(f"Started moving with vx={self.linear_vel_x:.2f}, vy={self.linear_vel_y:.2f}, wz={self.angular_vel_z:.2f}")
        elif not moving and self.is_moving:
            self.is_moving = False
            self.current_state = RobotState.STANDING
            self.get_logger().info("Stopped moving")
    
    def imu_callback(self, msg):
        """
        Callback for handling IMU data
        
        Args:
            msg (Imu): IMU message
        """
        # Convert quaternion to Euler angles
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Get roll, pitch, yaw (in radians)
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        
        # Update body adjustments for balance (if enabled)
        if self.current_state in [RobotState.MOVING, RobotState.ADAPTIVE]:
            self.update_body_balance()
        
        self.has_imu_data = True
    
    def update_body_balance(self):
        """Update body balance based on IMU feedback"""
        # Simple proportional control for body adjustment
        # Adjust these gains as needed
        roll_gain = 0.05
        pitch_gain = 0.05
        
        # Calculate adjustments to counteract roll and pitch
        # Negative feedback to counteract the tilt
        self.body_roll_adjustment = -roll_gain * self.roll  # Convert to meters
        self.body_pitch_adjustment = -pitch_gain * self.pitch  # Convert to meters
    
    def set_gait_callback(self, msg):
        """
        Callback for changing gait type
        
        Args:
            msg (String): Gait type name
        """
        gait_name = msg.data.lower()
        if gait_name == 'tripod':
            self.current_gait = GaitType.TRIPOD
            self.get_logger().info('Switched to tripod gait')
        elif gait_name == 'ripple':
            self.current_gait = GaitType.RIPPLE
            self.get_logger().info('Switched to ripple gait')
        elif gait_name == 'wave':
            self.current_gait = GaitType.WAVE
            self.get_logger().info('Switched to wave gait')
        else:
            self.get_logger().warning(f'Unknown gait type: {gait_name}')
    
    def enable_adaptive_walking(self, enable=True):
        """Enable or disable adaptive walking mode"""
        self.enable_terrain_adaptation = enable
        if enable:
            self.current_state = RobotState.ADAPTIVE
            self.get_logger().info('Adaptive walking enabled')
        else:
            self.current_state = RobotState.STANDING if not self.is_moving else RobotState.MOVING
            self.get_logger().info('Adaptive walking disabled')
    
    def get_leg_phase(self, leg_name):
        """
        Get the phase (0.0 to 1.0) for a specific leg in the current gait
        
        Args:
            leg_name (str): Name of the leg
            
        Returns:
            float: Phase value from 0.0 to 1.0
        """
        if self.current_gait == GaitType.TRIPOD:
            # Determine which group the leg is in
            if leg_name in self.tripod_groups[0]:
                return self.cycle_phase
            else:
                return (self.cycle_phase + 0.5) % 1.0
        
        elif self.current_gait == GaitType.RIPPLE:
            # Determine which group the leg is in
            if leg_name in self.ripple_groups[0]:
                return self.cycle_phase
            elif leg_name in self.ripple_groups[1]:
                return (self.cycle_phase + 0.33) % 1.0
            else:
                return (self.cycle_phase + 0.67) % 1.0
        
        elif self.current_gait == GaitType.WAVE:
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
        
        # If we're not moving, just return the standing position with IMU adjustments
        if not self.is_moving and self.current_state != RobotState.ADAPTIVE:
            # Apply body balance adjustments
            z_adjustment = 0.0
            
            if self.has_imu_data:
                # Calculate adjustments based on roll and pitch
                # For roll, legs on the left side go up when roll is positive
                # For pitch, front legs go up when pitch is positive
                
                # Determine if this leg is on the left or right
                left_side = 'left' in leg_name
                right_side = 'right' in leg_name
                
                # Determine if this leg is at the front, middle or back
                front = 'front' in leg_name
                back = 'back' in leg_name
                
                # Apply roll adjustment
                if left_side:
                    z_adjustment += self.body_roll_adjustment
                elif right_side:
                    z_adjustment -= self.body_roll_adjustment
                
                # Apply pitch adjustment
                if front:
                    z_adjustment += self.body_pitch_adjustment
                elif back:
                    z_adjustment -= self.body_pitch_adjustment
            
            return [base_pos[0] + offset[0], base_pos[1] + offset[1], offset[2] + z_adjustment]
        
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
        
        # Z adjustment for body balance (from IMU)
        z_adjustment = 0.0
        
        if self.has_imu_data:
            # Same as above, but applied during movement
            left_side = 'left' in leg_name
            right_side = 'right' in leg_name
            front = 'front' in leg_name
            back = 'back' in leg_name
            
            if left_side:
                z_adjustment += self.body_roll_adjustment
            elif right_side:
                z_adjustment -= self.body_roll_adjustment
            
            if front:
                z_adjustment += self.body_pitch_adjustment
            elif back:
                z_adjustment -= self.body_pitch_adjustment
        
        # Terrain adaptation adjustment
        terrain_adjustment = 0.0
        if self.enable_terrain_adaptation:
            terrain_adjustment = self.terrain_height_map[leg_name]
        
        if phase < 0.5:  # Swing phase (moving foot forward in the air)
            # Normalize to 0-1 range for swing phase
            swing_phase = phase * 2.0
            
            # X and Y move from -stride/2 to +stride/2
            x_offset = stride_x * (swing_phase - 0.5)
            y_offset = stride_y * (swing_phase - 0.5)
            
            # Z follows a sine curve for lift, plus adjustments
            z_offset = (-self.stand_height + self.step_height * math.sin(swing_phase * math.pi) 
                        + z_adjustment + terrain_adjustment)
            
        else:  # Support phase (foot on ground, moving backward)
            # Normalize to 0-1 range for support phase
            support_phase = (phase - 0.5) * 2.0
            
            # X and Y move from +stride/2 to -stride/2
            x_offset = stride_x * (0.5 - support_phase)
            y_offset = stride_y * (0.5 - support_phase)
            
            # Z is at the standing height plus adjustments
            z_offset = -self.stand_height + z_adjustment + terrain_adjustment
        
        # Combined foot position
        foot_x = base_pos[0] + offset[0] + x_offset
        foot_y = base_pos[1] + offset[1] + y_offset
        foot_z = offset[2] + z_offset
        
        return [foot_x, foot_y, foot_z]
    
    def detect_terrain(self, leg_name, foot_position):
        """
        Detect terrain height changes during step cycle
        This is a simplified version - in a real robot you'd use sensors
        
        Args:
            leg_name (str): Name of the leg
            foot_position (list): Current foot position
        """
        # This is where you would integrate actual terrain sensing
        # For now, it's just a placeholder function
        pass
    
    def gait_step(self):
        """
        Execute a step in the gait cycle
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        # Update cycle phase based on elapsed time and gait frequency
        if self.is_moving or self.current_state == RobotState.ADAPTIVE:
            self.cycle_phase = (elapsed * self.gait_frequency) % 1.0
        
        # Process each leg
        for leg_name in self.leg_positions.keys():
            # Get the phase for this leg in the current gait
            leg_phase = self.get_leg_phase(leg_name)
            
            # Calculate target foot position
            foot_position = self.calculate_foot_position(leg_name, leg_phase)
            
            # If in adaptive mode, detect terrain during step
            if self.enable_terrain_adaptation and leg_phase > 0.4 and leg_phase < 0.6:
                # This is where the foot touches down - detect terrain
                self.detect_terrain(leg_name, foot_position)
            
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
            offset = self.standing_offsets[leg_name]
            
            point_msg = Point()
            point_msg.x = base_pos[0] + offset[0]
            point_msg.y = base_pos[1] + offset[1]
            point_msg.z = offset[2]
            
            self.leg_publishers[leg_name].publish(point_msg)
            
        self.get_logger().info('Moved to standing position')
        self.current_state = RobotState.STANDING
    
    def path_planning(self, target_x, target_y):
        """
        Simple path planning to reach a target position
        
        Args:
            target_x (float): Target X position
            target_y (float): Target Y position
        """
        # This is a simplified path planner
        # In a real robot, you would implement A* or other path planning algorithms
        
        # For now, just calculate the direction vector
        current_x = 0.0  # Assume robot is at origin
        current_y = 0.0
        
        # Direction vector
        dx = target_x - current_x
        dy = target_y - current_y
        
        # Distance
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Normalize
        if distance > 0.001:
            dx /= distance
            dy /= distance
        
        # Set velocity commands based on direction
        # Scale by a speed factor
        speed_factor = 0.2  # m/s
        self.linear_vel_x = dx * speed_factor
        self.linear_vel_y = dy * speed_factor
        
        # Start moving
        self.is_moving = True
        
        return distance
    
    def navigate_to_target(self, target_x, target_y, tolerance=0.05):
        """
        Navigate to a target position
        
        Args:
            target_x (float): Target X position
            target_y (float): Target Y position
            tolerance (float): Distance tolerance for reaching the target
        """
        self.enable_path_planning = True
        self.target_position = (target_x, target_y)
        
        # Initial path planning
        distance = self.path_planning(target_x, target_y)
        
        # Log the target
        self.get_logger().info(f'Navigating to target: ({target_x:.2f}, {target_y:.2f}), distance: {distance:.2f}m')
        
        # The actual navigation happens in the gait_step method
        # This method just sets up the target
        
        return distance < tolerance

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