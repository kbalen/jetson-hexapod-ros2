#!/usr/bin/env python3
# hexapod_kinematics.py

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
# Try to import tf_transformations, or use transforms3d as fallback
try:
    import tf_transformations
except ImportError:
    import transforms3d as tf_transformations
    # Or simply comment out if not needed immediately

class HexapodKinematics(Node):
    def __init__(self):
        super().__init__('hexapod_kinematics')
        
        # Robot parameters (update these based on your URDF)
        self.coxa_length = 0.05  # Length of the coxa segment (hip to first joint)
        self.femur_length = 0.10  # Length of the femur segment
        self.tibia_length = 0.12  # Length of the tibia segment
        
        # Define leg mounting positions relative to body center
        # Format: [x, y, z] where:
        # x: forward (+) / backward (-)
        # y: left (+) / right (-)
        # z: up (+) / down (-)
        self.leg_positions = {
            'front_left':  [ 0.110,  0.080, 0.0],  # FL
            'middle_left': [ 0.000,  0.130, 0.0],  # ML
            'back_left':   [-0.110,  0.080, 0.0],  # BL
            'back_right':  [-0.110, -0.080, 0.0],  # BR
            'middle_right':[ 0.000, -0.130, 0.0],  # MR
            'front_right': [ 0.110, -0.080, 0.0],  # FR
        }
        
        # Leg orientation offsets in radians
        self.leg_orientations = {
            'front_left':   math.pi/4,    # 45 degrees
            'middle_left':  math.pi/2,    # 90 degrees
            'back_left':    3*math.pi/4,  # 135 degrees
            'back_right':   -3*math.pi/4, # -135 degrees
            'middle_right': -math.pi/2,   # -90 degrees
            'front_right':  -math.pi/4,   # -45 degrees
        }
        
        # Joint to ID mapping
        self.joint_mapping = {
            'front_left': ['j1', 'j2', 'j3'],      # coxa, femur, tibia
            'middle_left': ['j4', 'j5', 'j6'],
            'back_left': ['j7', 'j8', 'j9'],
            'back_right': ['j10', 'j11', 'j12'],
            'middle_right': ['j13', 'j14', 'j15'],
            'front_right': ['j16', 'j17', 'j18']
        }
        
        # Mark the joints that have inverted axes (using negative z-axis in URDF)
        # From the URDF, these are j11, j12, j14, j15, j17, j18
        self.inverted_joints = {
            'back_right': [True, True, True],     # j10, j11, j12
            'middle_right': [False, True, True],  # j13, j14, j15
            'front_right': [False, True, True]    # j16, j17, j18
        }
        
        # Service to compute IK for a leg
        # For now using publishers/subscribers, could be changed to services
        self.ik_publishers = {}
        for leg in self.leg_positions.keys():
            self.ik_publishers[leg] = self.create_publisher(
                Float64MultiArray, 
                f'/hexapod/leg/{leg}/joint_commands', 
                10
            )
        
        # Create subscribers for individual leg targets
        self.leg_target_subscribers = {}
        for leg in self.leg_positions.keys():
            self.leg_target_subscribers[leg] = self.create_subscription(
                Point,
                f'/hexapod/leg/{leg}/target_position',
                lambda msg, leg_name=leg: self.leg_target_position_callback(msg, leg_name),
                10
            )
            
        # Create a subscriber for the general leg target (for testing)
        self.target_subscription = self.create_subscription(
            Point,
            '/hexapod/leg_target',
            self.leg_target_callback,
            10
        )
        
        # Publisher for all joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.get_logger().info('Hexapod Kinematics node initialized')
        
    def forward_kinematics(self, leg_name, joint_angles):
        """
        Compute forward kinematics for a leg
        
        Args:
            leg_name (str): Name of the leg
            joint_angles (list): List of 3 joint angles [coxa, femur, tibia] in radians
            
        Returns:
            Point: Foot tip position in the robot's body frame
        """
        # Extract joint angles
        coxa_angle = joint_angles[0]
        femur_angle = joint_angles[1]
        tibia_angle = joint_angles[2]
        
        # Get the leg's base position and orientation
        base_pos = self.leg_positions[leg_name]
        orientation = self.leg_orientations[leg_name]
        
        # Apply coxa rotation (around z-axis)
        # First compute the coxa endpoint
        coxa_x = self.coxa_length * math.cos(coxa_angle + orientation)
        coxa_y = self.coxa_length * math.sin(coxa_angle + orientation)
        
        # Apply femur rotation (around y-axis in the leg's coordinate frame)
        # Compute femur endpoint
        femur_x = coxa_x + self.femur_length * math.cos(femur_angle) * math.cos(coxa_angle + orientation)
        femur_y = coxa_y + self.femur_length * math.cos(femur_angle) * math.sin(coxa_angle + orientation)
        femur_z = -self.femur_length * math.sin(femur_angle)
        
        # Apply tibia rotation (around y-axis in the femur's coordinate frame)
        # Compute tibia endpoint (foot tip)
        tibia_x = femur_x + self.tibia_length * math.cos(femur_angle + tibia_angle) * math.cos(coxa_angle + orientation)
        tibia_y = femur_y + self.tibia_length * math.cos(femur_angle + tibia_angle) * math.sin(coxa_angle + orientation)
        tibia_z = femur_z - self.tibia_length * math.sin(femur_angle + tibia_angle)
        
        # Add the base position offset
        foot_pos_x = base_pos[0] + tibia_x
        foot_pos_y = base_pos[1] + tibia_y
        foot_pos_z = base_pos[2] + tibia_z
        
        return [foot_pos_x, foot_pos_y, foot_pos_z]
    
    def inverse_kinematics(self, leg_name, target_position):
        """
        Compute inverse kinematics for a leg
        
        Args:
            leg_name (str): Name of the leg
            target_position (list): Target foot position [x, y, z] in the robot's body frame
            
        Returns:
            list: Joint angles [coxa, femur, tibia] in radians
        """
        # Get the leg's base position and orientation
        base_pos = self.leg_positions[leg_name]
        orientation = self.leg_orientations[leg_name]
        
        # Translate target position to the leg's local frame
        local_x = target_position[0] - base_pos[0]
        local_y = target_position[1] - base_pos[1]
        local_z = target_position[2] - base_pos[2]
        
        # Calculate leg length in the XY plane
        len_xy = math.sqrt(local_x**2 + local_y**2)
        
        # Calculate coxa angle
        coxa_angle = math.atan2(local_y, local_x) - orientation
        
        # Adjust for coxa length to get the femur base to foot distance
        foot_offset_x = len_xy - self.coxa_length
        
        # Calculate leg length from femur joint to foot
        leg_length = math.sqrt(foot_offset_x**2 + local_z**2)
        
        # Check if the target is reachable
        if leg_length > (self.femur_length + self.tibia_length):
            self.get_logger().warning(f"Target position unreachable for leg {leg_name}")
            # Return the closest approximation
            leg_length = self.femur_length + self.tibia_length - 0.001
        
        # Use the law of cosines to find femur and tibia angles
        # Angle between femur and tibia
        cos_angle = (self.femur_length**2 + self.tibia_length**2 - leg_length**2) / (2 * self.femur_length * self.tibia_length)
        
        # Ensure cos_angle is in the valid range [-1, 1]
        cos_angle = max(-1.0, min(1.0, cos_angle))
        
        # Tibia angle (negative because robotics convention often has positive bending toward the body)
        tibia_angle = -(math.pi - math.acos(cos_angle))
        
        # Angle of the leg in the XZ plane
        gamma = math.atan2(local_z, foot_offset_x)
        
        # Angle between leg and femur
        cos_beta = (self.femur_length**2 + leg_length**2 - self.tibia_length**2) / (2 * self.femur_length * leg_length)
        
        # Ensure cos_beta is in the valid range [-1, 1]
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        
        # Femur angle (negative because of the same convention)
        femur_angle = -(gamma + beta)
        
        # Create result array
        joint_angles = [coxa_angle, femur_angle, tibia_angle]
        
        # Handle inverted joints for right-side legs
        if leg_name in self.inverted_joints:
            for i, inverted in enumerate(self.inverted_joints[leg_name]):
                if inverted:
                    joint_angles[i] = -joint_angles[i]
        
        return joint_angles
        
    def leg_target_callback(self, msg):
        """
        Callback for the general leg target subscriber
        
        Args:
            msg (Point): Target position for the foot in the body frame
        """
        # For now, we'll just move the front left leg
        leg_name = 'front_left'
        target_position = [msg.x, msg.y, msg.z]
        
        try:
            # Compute inverse kinematics
            joint_angles = self.inverse_kinematics(leg_name, target_position)
            
            # Publish joint commands
            joint_cmd = Float64MultiArray()
            joint_cmd.data = joint_angles
            self.ik_publishers[leg_name].publish(joint_cmd)
            
            # Log the computed joint angles
            self.get_logger().info(f"IK for {leg_name}: Coxa={math.degrees(joint_angles[0]):.2f}°, " +
                                  f"Femur={math.degrees(joint_angles[1]):.2f}°, " +
                                  f"Tibia={math.degrees(joint_angles[2]):.2f}°")
        except Exception as e:
            self.get_logger().error(f"Error computing inverse kinematics: {e}")
    
    def leg_target_position_callback(self, msg, leg_name):
        """
        Callback for individual leg target position subscribers
        
        Args:
            msg (Point): Target position for the foot
            leg_name (str): Name of the leg
        """
        target_position = [msg.x, msg.y, msg.z]
        
        try:
            # Compute inverse kinematics
            joint_angles = self.inverse_kinematics(leg_name, target_position)
            
            # Publish joint commands
            joint_cmd = Float64MultiArray()
            joint_cmd.data = joint_angles
            self.ik_publishers[leg_name].publish(joint_cmd)
            
            # Log the computed joint angles
            self.get_logger().debug(f"IK for {leg_name}: Coxa={math.degrees(joint_angles[0]):.2f}°, " +
                                  f"Femur={math.degrees(joint_angles[1]):.2f}°, " +
                                  f"Tibia={math.degrees(joint_angles[2]):.2f}°")
        except Exception as e:
            self.get_logger().error(f"Error computing inverse kinematics for {leg_name}: {e}")
    
    def publish_joint_state(self, joint_positions):
        """
        Publish the joint state message for visualization and control
        
        Args:
            joint_positions (dict): Dictionary of joint positions keyed by joint name
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        for leg_name, joint_list in self.joint_mapping.items():
            for i, joint in enumerate(joint_list):
                if joint in joint_positions:
                    joint_state_msg.name.append(joint)
                    joint_state_msg.position.append(joint_positions[joint])
        
        self.joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodKinematics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()