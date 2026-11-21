import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time

class Hexa2GaitController(Node):
    def __init__(self):
        super().__init__('hexa2_gait_controller')
        
        # Explicitly set this as a tripod gait controller
        self.get_logger().info('Initializing Tripod Gait Controller')
        
        # Subscribe to cmd_vel for teleop control
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Joint trajectory publisher for controlling the servos
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10)
            
        # Define the leg joint names (using your URDF joint names j1-j18)
        self.leg_joints = [
            'j1', 'j2', 'j3',   # Front Left leg (coxa, femur, tibia)
            'j4', 'j5', 'j6',   # Middle Left leg
            'j7', 'j8', 'j9',   # Back Left leg
            'j10', 'j11', 'j12', # Back Right leg
            'j13', 'j14', 'j15', # Middle Right leg
            'j16', 'j17', 'j18'  # Front Right leg
        ]
        
        # Default standing pose (neutral position) - optimized for your robot
        # Using values within the joint limits defined in your URDF:
        # - Coxa joints: -0.5 to 0.5 radians
        # - Femur joints: -1.0 to 1.0 radians
        # - Tibia joints: -1.4 to 0.0 radians
        self.neutral_pose = np.array([
            0.0, -0.5, -0.7,  # Front Left (j1, j2, j3)
            0.0, -0.5, -0.7,  # Middle Left (j4, j5, j6)
            0.0, -0.5, -0.7,  # Back Left (j7, j8, j9)
            0.0, -0.5, -0.7,  # Back Right (j10, j11, j12)
            0.0, -0.5, -0.7,  # Middle Right (j13, j14, j15)
            0.0, -0.5, -0.7   # Front Right (j16, j17, j18)
        ])
        
        # Define leg groups for tripod gait
        # This matches your hexapod's structure - legs in the same group move together
        self.tripod_a = [0, 2, 4]  # Front Left, Back Left, Middle Right (indices of legs)
        self.tripod_b = [1, 3, 5]  # Middle Left, Back Right, Front Right
        
        # Movement parameters optimized for diymore 13kg servos
        # These servos are fast (0.13sec/60deg) so we can have a quicker cycle time
        self.stride_length = 0.08  # meters
        self.stride_height = 0.05  # meters
        self.cycle_time = 0.8     # seconds per complete step cycle (faster with these servos)
        
        # Current movement command
        self.current_cmd_vel = Twist()
        
        # Start in standing pose
        self.move_to_pose(self.neutral_pose)
        
        # Create a timer for the gait cycle
        self.timer = self.create_timer(0.05, self.gait_cycle_callback)
        
        # Gait state
        self.gait_phase = 0.0  # 0.0 to 1.0 represents a full gait cycle
        self.is_walking = False
        
        self.get_logger().info('Hexa2 gait controller initialized')
        
    def cmd_vel_callback(self, msg):
        """Process velocity commands from teleop"""
        self.current_cmd_vel = msg
        
        # Check if we should be walking
        linear_velocity = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
        angular_velocity = abs(msg.angular.z)
        
        if linear_velocity > 0.01 or angular_velocity > 0.01:
            self.is_walking = True
        else:
            self.is_walking = False
            # If stopped, return to neutral pose
            self.move_to_pose(self.neutral_pose)
    
    def gait_cycle_callback(self):
        """Main gait generation loop"""
        if not self.is_walking:
            return
            
        # Update gait phase
        self.gait_phase += 0.05 / self.cycle_time
        if self.gait_phase >= 1.0:
            self.gait_phase = 0.0
            
        # Generate leg positions for current phase
        joint_positions = self.generate_tripod_gait(
            self.gait_phase, 
            self.current_cmd_vel.linear.x,
            self.current_cmd_vel.linear.y, 
            self.current_cmd_vel.angular.z
        )
        
        # Send joint commands
        self.publish_joint_trajectory(joint_positions)
        
    def generate_tripod_gait(self, phase, vx, vy, omega):
        """Generate tripod gait joint positions based on current phase and velocity"""
        joint_positions = np.copy(self.neutral_pose)
        
        # Scale stride based on velocity
        speed = np.sqrt(vx**2 + vy**2)
        stride_length = min(self.stride_length, self.stride_length * speed / 0.2)
        
        # Direction of movement
        if speed > 0.01:
            direction = np.arctan2(vy, vx)
        else:
            direction = 0.0
            
        # Adjust for rotation
        turn_factor = np.clip(omega / 0.5, -1.0, 1.0)
        
        # Process each leg
        legs = [0, 1, 2, 3, 4, 5]  # Leg indices
        for leg_idx in legs:
            # Get base indices for this leg's joints
            base_idx = leg_idx * 3
            
            # Determine if this leg is in tripod A or B
            in_tripod_a = leg_idx in self.tripod_a
            
            # Adjust phase for tripod gait (180° out of phase)
            leg_phase = phase if in_tripod_a else (phase + 0.5) % 1.0
            
            # Based on leg's position on body, adjust direction of movement
            leg_angle = 0.0
            if leg_idx == 0:  # FL
                leg_angle = np.pi/4
            elif leg_idx == 1:  # ML
                leg_angle = np.pi/2
            elif leg_idx == 2:  # BL
                leg_angle = 3*np.pi/4
            elif leg_idx == 3:  # BR
                leg_angle = -3*np.pi/4
            elif leg_idx == 4:  # MR
                leg_angle = -np.pi/2
            elif leg_idx == 5:  # FR
                leg_angle = -np.pi/4
                
            # Combine linear and angular motion
            leg_direction = direction
            
            # If mainly turning, adjust leg movement for rotation
            if abs(turn_factor) > 0.5:
                # Pure rotation behavior
                leg_direction = leg_angle + np.pi/2 * np.sign(omega)
                
            # First, compute foot position through gait cycle
            if leg_phase < 0.5:  # Stance phase - foot on ground, moving back
                # Linearly move from front to back during stance phase
                x_offset = stride_length * (0.5 - leg_phase) / 0.5
                z_offset = 0.0  # on ground
            else:  # Swing phase - foot in air, moving forward
                # Linearly move from back to front during swing phase
                x_offset = -stride_length * (leg_phase - 0.5) / 0.5
                # Parabolic up-and-down movement during swing
                swing_phase = (leg_phase - 0.5) / 0.5  # 0 to 1 during swing
                z_offset = self.stride_height * np.sin(swing_phase * np.pi)
                
            # Convert to leg-relative XYZ (simplified, should use inverse kinematics)
            x = x_offset * np.cos(leg_direction)
            y = x_offset * np.sin(leg_direction)
            z = -0.12 + z_offset  # Base height plus offset
            
            # Simplified inverse kinematics for 3DOF leg
            # This is a very simplified version - should be replaced with proper IK
            coxa_length = 0.05
            femur_length = 0.08
            tibia_length = 0.11
            
            # Coxa joint angle (rotate to face direction of motion)
            coxa_angle = np.arctan2(y, x)
            
            # Distance from coxa to foot in the XY plane
            coxa_to_foot_xy = np.sqrt(x**2 + y**2) - coxa_length
            
            # Distance from coxa to foot in 3D space
            coxa_to_foot = np.sqrt(coxa_to_foot_xy**2 + z**2)
            
            # Adjust distances if out of reach
            if coxa_to_foot > (femur_length + tibia_length) * 0.99:
                coxa_to_foot = (femur_length + tibia_length) * 0.99
                
            # Using law of cosines for femur and tibia angles
            # Femur angle
            cos_femur_angle = (femur_length**2 + coxa_to_foot**2 - tibia_length**2) / (2 * femur_length * coxa_to_foot)
            cos_femur_angle = np.clip(cos_femur_angle, -1.0, 1.0)  # Avoid domain errors
            femur_angle = np.arccos(cos_femur_angle)
            
            # Tibia angle
            cos_tibia_angle = (femur_length**2 + tibia_length**2 - coxa_to_foot**2) / (2 * femur_length * tibia_length)
            cos_tibia_angle = np.clip(cos_tibia_angle, -1.0, 1.0)  # Avoid domain errors
            tibia_angle = np.arccos(cos_tibia_angle)
            
            # Adjust femur angle based on the height
            femur_atan = np.arctan2(z, coxa_to_foot_xy)
            femur_angle = femur_atan - femur_angle
            
            # Adjust angles for the robot's configuration
            coxa_adjusted = coxa_angle
            femur_adjusted = -femur_angle
            tibia_adjusted = -(np.pi - tibia_angle)
            
            # Store the computed joint angles
            joint_positions[base_idx] = coxa_adjusted
            joint_positions[base_idx + 1] = femur_adjusted
            joint_positions[base_idx + 2] = tibia_adjusted
            
        return joint_positions
                
    def move_to_pose(self, joint_positions):
        """Move all joints to specified positions"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.leg_joints
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = 1  # 1 second to reach position
        
        trajectory_msg.points.append(point)
        self.joint_trajectory_pub.publish(trajectory_msg)
        
    def publish_joint_trajectory(self, joint_positions):
        """Publish joint positions as trajectory points"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.leg_joints
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50ms
        
        trajectory_msg.points.append(point)
        self.joint_trajectory_pub.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Hexa2GaitController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown of the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()