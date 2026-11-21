import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
from adafruit_servokit import ServoKit

class Hexa2ServoController(Node):
    def __init__(self):
        super().__init__('hexa2_servo_controller')
        
        # Create a subscription to the joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
            
        # Create a subscription to the joint trajectory command
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.joint_trajectory_callback,
            10)
        
        # Initialize the servo controller (PCA9685)
        self.get_logger().info('Initializing servo controller')
        try:
            # Initialize two PCA9685 boards for controlling all 18 servos
            self.kit = ServoKit(channels=16, address=0x40)  # First controller at default address
            self.kit2 = ServoKit(channels=16, address=0x41)  # Second controller at address 0x41
            
            # Configure servo parameters for diymore 13kg servos (500-2500μs pulse width)
            for i in range(18):  # 18 servos total
                # Set servo range for full 180 degree motion
                servo_num = i if i < 16 else i - 16
                kit_instance = self.kit if i < 16 else self.kit2
                
                # Set pulse width range to 500-2500μs as per your servo specs
                kit_instance.servo[servo_num].set_pulse_width_range(500, 2500)
                
                # Center all servos at start (90 degrees)
                kit_instance.servo[servo_num].angle = 90
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo controller: {e}')
        
        # Mapping from joint names to servo channels
        # Adjusted for your robot's URDF joint names (j1-j18)
        self.joint_to_servo_map = {
            'j1': 0,  # Front Left Coxa
            'j2': 1,  # Front Left Femur
            'j3': 2,  # Front Left Tibia
            'j4': 3,  # Middle Left Coxa
            'j5': 4,  # Middle Left Femur
            'j6': 5,  # Middle Left Tibia
            'j7': 6,  # Back Left Coxa
            'j8': 7,  # Back Left Femur
            'j9': 8,  # Back Left Tibia
            'j10': 9,  # Back Right Coxa
            'j11': 10, # Back Right Femur
            'j12': 11, # Back Right Tibia
            'j13': 12, # Middle Right Coxa
            'j14': 13, # Middle Right Femur
            'j15': 14, # Middle Right Tibia
            'j16': 15, # Front Right Coxa
            'j17': 16, # Front Right Femur (on second board)
            'j18': 17, # Front Right Tibia (on second board)
        }
        
        # Define joint angle offsets (calibration)
        # Initial values based on your URDF
        self.joint_offsets = {joint: 0.0 for joint in self.joint_to_servo_map.keys()}
        
        # Define joint angle scaling factors
        # Convert from radians to servo units
        self.joint_scaling = {joint: 180.0/np.pi for joint in self.joint_to_servo_map.keys()}
        
        # Define joint limits from your URDF
        self.joint_limits = {
            # Coxa joints
            'j1': (-0.5, 0.5),
            'j4': (-0.5, 0.5),
            'j7': (-0.5, 0.5),
            'j10': (-0.5, 0.5),
            'j13': (-0.5, 0.5),
            'j16': (-0.5, 0.5),
            
            # Femur joints
            'j2': (-1.0, 1.0),
            'j5': (-1.0, 1.0),
            'j8': (-1.0, 1.0),
            'j11': (-1.0, 1.0),
            'j14': (-1.0, 1.0),
            'j17': (-1.0, 1.0),
            
            # Tibia joints
            'j3': (-1.4, 0.0),
            'j6': (-1.4, 0.0),
            'j9': (-1.4, 0.0),
            'j12': (-1.4, 0.0),
            'j15': (-1.4, 0.0),
            'j18': (-1.4, 0.0),
        }
        
        # Define which joints have inverted direction (negative z-axis in URDF)
        # This affects how we translate ROS joint commands to servo commands
        self.inverted_joints = ['j11', 'j12', 'j14', 'j15', 'j17', 'j18']
        
        self.get_logger().info('Hexa2 servo controller initialized')

    def joint_states_callback(self, msg):
        """Process joint state messages and update servo positions"""
        try:
            for i, name in enumerate(msg.name):
                if name in self.joint_to_servo_map:
                    servo_idx = self.joint_to_servo_map[name]
                    angle_rad = msg.position[i]
                    
                    # Check if the joint is inverted in the URDF
                    if name in self.inverted_joints:
                        angle_rad = -angle_rad
                    
                    # Apply joint limits before conversion
                    if name in self.joint_limits:
                        min_rad, max_rad = self.joint_limits[name]
                        angle_rad = max(min_rad, min(max_rad, angle_rad))
                    
                    # Convert from joint angle space (-limit to +limit) to servo angle space (0-180)
                    # For the specific joint limits in your URDF
                    min_rad, max_rad = self.joint_limits.get(name, (-np.pi/2, np.pi/2))
                    angle_norm = (angle_rad - min_rad) / (max_rad - min_rad)
                    
                    # Apply offset and scaling to convert to servo angle
                    angle_deg = 180.0 * angle_norm + self.joint_offsets[name]
                    
                    # Ensure angle is within valid range
                    angle_deg = max(0, min(180, angle_deg))
                    
                    # Determine which controller to use based on servo index
                    kit_instance = self.kit if servo_idx < 16 else self.kit2
                    actual_idx = servo_idx if servo_idx < 16 else servo_idx - 16
                    
                    # Set servo angle
                    kit_instance.servo[actual_idx].angle = angle_deg
                    
                    self.get_logger().debug(f'Joint {name}: {angle_rad:.2f} rad -> {angle_deg:.2f} deg (Servo {actual_idx})')
                    
        except Exception as e:
            self.get_logger().error(f'Error setting servo angles: {e}')

    def joint_trajectory_callback(self, msg):
        """Process trajectory messages for smoother motion"""
        try:
            # Extract joint names
            joint_names = msg.joint_names
            
            # Process each trajectory point
            for point in msg.points:
                # Set positions for each joint
                for i, name in enumerate(joint_names):
                    if name in self.joint_to_servo_map:
                        servo_idx = self.joint_to_servo_map[name]
                        angle_rad = point.positions[i]
                        
                        # Check if the joint is inverted in the URDF
                        if name in self.inverted_joints:
                            angle_rad = -angle_rad
                        
                        # Apply joint limits before conversion
                        if name in self.joint_limits:
                            min_rad, max_rad = self.joint_limits[name]
                            angle_rad = max(min_rad, min(max_rad, angle_rad))
                        
                        # Convert from joint angle space to servo angle space
                        min_rad, max_rad = self.joint_limits.get(name, (-np.pi/2, np.pi/2))
                        angle_norm = (angle_rad - min_rad) / (max_rad - min_rad)
                        
                        # Apply offset and scaling to convert to servo angle
                        angle_deg = 180.0 * angle_norm + self.joint_offsets[name]
                        
                        # Ensure angle is within valid range
                        angle_deg = max(0, min(180, angle_deg))
                        
                        # Determine which controller to use based on servo index
                        kit_instance = self.kit if servo_idx < 16 else self.kit2
                        actual_idx = servo_idx if servo_idx < 16 else servo_idx - 16
                        
                        # Set servo angle
                        kit_instance.servo[actual_idx].angle = angle_deg
                
                # Sleep to achieve the desired trajectory timing
                # This is a simple implementation; a more sophisticated approach would use the time_from_start
                if len(msg.points) > 1:
                    time.sleep(0.05)  # Simple fixed delay
                    
        except Exception as e:
            self.get_logger().error(f'Error processing trajectory: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Hexa2ServoController()
    
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