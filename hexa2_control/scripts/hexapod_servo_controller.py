#!/usr/bin/env python3
# hexapod_servo_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
from smbus2 import SMBus
import time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class PCA9685:
    """Simple PCA9685 PWM controller using smbus2"""
    # Register addresses
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    
    def __init__(self, bus_num, address):
        self.bus = SMBus(bus_num)
        self.address = address
        
        # Initialize
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.05)  # Wait for oscillator
        
        # Set frequency to 50Hz (for servos)
        self.set_pwm_freq(50)
    
    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)
        
        oldmode = self.bus.read_byte_data(self.address, self.MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.bus.write_byte_data(self.address, self.MODE1, newmode)  # go to sleep
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, oldmode | 0xa1)
    
    def set_pwm(self, channel, on, off):
        """Sets a single PWM channel"""
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)
    
    def set_servo_angle(self, channel, angle):
        """Set servo angle (0-180 degrees)"""
        # Convert angle to pulse length (roughly 500-2500 μs)
        pulse = int(500 + (2000 * angle / 180))
        
        # Convert pulse length to 12-bit value
        value = int(pulse * 4096 / 20000)  # 20000μs = 20ms period at 50Hz
        
        # Set PWM value
        self.set_pwm(channel, 0, value)
    
    def close(self):
        self.bus.close()

class HexapodServoController(Node):
    def __init__(self):
        super().__init__('hexapod_servo_controller')
        
        # Initialize PCA9685 boards
        try:
            # Use bus 0 (which is where i2cdetect found your devices)
            self.kit1 = PCA9685(0, 0x40)  # First controller
            self.kit2 = PCA9685(0, 0x41)  # Second controller
            
            self.get_logger().info('Servo controllers initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo controllers: {e}')
            return
        
        # Define joint to servo mapping based on your calibration script
        self.joint_to_servo_map = {
            'j1': (0, 0),    # (board_idx, channel) - Front Left Coxa
            'j2': (0, 1),    # Front Left Femur
            'j3': (0, 2),    # Front Left Tibia
            'j4': (0, 4),    # Middle Left Coxa
            'j5': (0, 5),    # Middle Left Femur
            'j6': (0, 6),    # Middle Left Tibia
            'j7': (0, 8),    # Back Left Coxa
            'j8': (0, 9),    # Back Left Femur
            'j9': (0,15),    # Back Left Tibia  
            'j10': (1, 0),   # Back Right Coxa
            'j11': (1, 1),  # Back Right Femur
            'j12': (1, 2),  # Back Right Tibia
            'j13': (1, 4),  # Middle Right Coxa
            'j14': (1, 5),  # Middle Right Femur
            'j15': (1, 6),  # Middle Right Tibia
            'j16': (1, 8),  # Front Right Coxa
            'j17': (1, 9),   # Front Right Femur (on second board)
            'j18': (1, 10)    # Front Right Tibia (on second board)
        }
        
        # Inverted joints (based on URDF axis directions)
        # From URDF: j11, j12, j14, j15, j17, j18 use axis xyz="0 0 -1"
        self.inverted_joints = ['j11', 'j12', 'j14', 'j15', 'j17', 'j18']
        
        # Joint limits from URDF
        self.joint_limits = {
            'j1': (-0.5, 0.5),
            'j2': (-1.0, 1.0),
            'j3': (-1.4, 0.0),
            'j4': (-0.5, 0.5),
            'j5': (-1.0, 1.0),
            'j6': (-1.4, 0.0),
            'j7': (-0.5, 0.5),
            'j8': (-1.0, 1.0),
            'j9': (-1.4, 0.0),
            'j10': (-0.5, 0.5),
            'j11': (-1.0, 1.0),
            'j12': (-1.4, 0.0),
            'j13': (-0.5, 0.5),
            'j14': (-1.0, 1.0),
            'j15': (-1.4, 0.0),
            'j16': (-0.5, 0.5),
            'j17': (-1.0, 1.0),
            'j18': (-1.4, 0.0)
        }
        
        # Mapping of leg names to joint names
        self.leg_to_joints = {
            'front_left': ['j1', 'j2', 'j3'],
            'middle_left': ['j4', 'j5', 'j6'],
            'back_left': ['j7', 'j8', 'j9'],
            'back_right': ['j10', 'j11', 'j12'],
            'middle_right': ['j13', 'j14', 'j15'],
            'front_right': ['j16', 'j17', 'j18']
        }
        
        # Load calibration if available
        self.calibration = {}
        try:
            from servo_calibration import calibration
            self.calibration = calibration
            self.get_logger().info('Loaded calibration from file')
        except ImportError:
            self.get_logger().info('No calibration file found, using defaults')
            for joint in self.joint_to_servo_map.keys():
                self.calibration[joint] = 0.0
        
        # Subscribe to joint commands for each leg
        self.joint_subscribers = {}
        for leg, joints in self.leg_to_joints.items():
            sub = self.create_subscription(
                Float64MultiArray,
                f'/hexapod/leg/{leg}/joint_commands',
                lambda msg, leg_name=leg: self.joint_command_callback(msg, leg_name),
                10
            )
            self.joint_subscribers[leg] = sub
        
        # Create a subscriber for target positions from IK
        for leg in self.leg_to_joints.keys():
            self.create_subscription(
                Point,
                f'/hexapod/leg/{leg}/target_position',
                lambda msg, leg_name=leg: self.leg_target_callback(msg, leg_name),
                10
            )
        
        # Create a service client for the inverse kinematics service
        # For now, we'll just calculate IK directly in the callback
        # Store the current joint angles
        self.current_angles = {joint: 90.0 for joint in self.joint_to_servo_map.keys()}
        # Set servos_enabled flag
        self.servos_enabled = True
        
        # Set robot to home position at startup
        self.set_home_position()
        # Set up timer for periodic publishing of servo states
        self.timer = self.create_timer(0.1, self.publish_servo_states)
        
        # Store the current joint angles
        self.current_angles = {joint: 90.0 for joint in self.joint_to_servo_map.keys()}
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to the depth image topic
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )
        self.get_logger().info('Subscribed to /camera/camera/depth/image_rect_raw')
        
        self.get_logger().info('Hexapod Servo Controller initialized')
    
    def rad_to_deg(self, rad):
        """Convert radians to degrees"""
        return rad * 180.0 / math.pi
    
    def deg_to_rad(self, deg):
        """Convert degrees to radians"""
        return deg * math.pi / 180.0
    
    def limit_angle(self, joint, angle_rad):
        """
        Limit the angle to the joint's range as defined in the URDF
        
        Args:
            joint (str): Joint name
            angle_rad (float): Angle in radians
            
        Returns:
            float: Limited angle in radians
        """
        if joint in self.joint_limits:
            min_angle, max_angle = self.joint_limits[joint]
            return max(min_angle, min(max_angle, angle_rad))
        return angle_rad
    
    def joint_command_callback(self, msg, leg_name):
        """
        Callback for joint command subscribers
        
        Args:
            msg (Float64MultiArray): Joint angles in radians
            leg_name (str): Name of the leg
        """
        if len(msg.data) != 3:
            self.get_logger().error(f"Invalid joint command for {leg_name}: expected 3 values, got {len(msg.data)}")
            return
        
        joint_names = self.leg_to_joints[leg_name]
        
        for i, joint in enumerate(joint_names):
            angle_rad = msg.data[i]
            
            # Limit the angle to the joint's range
            angle_rad = self.limit_angle(joint, angle_rad)
            
            # Convert to degrees for the servo
            angle_deg = self.rad_to_deg(angle_rad)
            
            # Invert if necessary
            if joint in self.inverted_joints:
                angle_deg = - angle_deg
            
            # Map to servo range (0-180) and add calibration offset
            servo_angle = 90 + angle_deg + self.calibration[joint]
            
            # Ensure within servo range
            servo_angle = max(0, min(180, servo_angle))
            
            # Store the current angle
            self.current_angles[joint] = servo_angle
            
            # Set the servo position
            self.set_servo_angle(joint, servo_angle)
    
    def inverse_kinematics(self, leg_name, target_position):
        """
        Compute inverse kinematics for a leg.
    
        Args:
            leg_name (str): Name of the leg (e.g., 'front_left')
            target_position (list): Target foot position [x, y, z] in the robot's body frame
        
        Returns:
            list: Joint angles [coxa_angle, femur_angle, tibia_angle] in radians
        """
        # Define robot dimensions (in meters)
        coxa_length = 0.05
        femur_length = 0.10
        tibia_length = 0.12

        # Define leg base positions (relative to the robot's body center)
        leg_positions = {
            'front_left':  [ 0.110,  0.080, 0.0],
            'middle_left': [ 0.000,  0.130, 0.0],
            'back_left':   [-0.110,  0.080, 0.0],
            'back_right':  [-0.110, -0.080, 0.0],
            'middle_right':[ 0.000, -0.130, 0.0],
            'front_right': [ 0.110, -0.080, 0.0],
        }

        # Define leg orientations (in radians)
        leg_orientations = {
            'front_left':   math.pi/4,    # 45 degrees
            'middle_left':  math.pi/2,    # 90 degrees
            'back_left':    3*math.pi/4,  # 135 degrees
            'back_right':   -3*math.pi/4, # -135 degrees
            'middle_right': -math.pi/2,   # -90 degrees
            'front_right':  -math.pi/4,   # -45 degrees
        }

        if leg_name not in leg_positions or leg_name not in leg_orientations:
            self.get_logger().error(f"Unknown leg name '{leg_name}' for IK computation")
            return [0.0, 0.0, 0.0]

        # Get the leg's mounting position and orientation
        base_pos = leg_positions[leg_name]
        orientation = leg_orientations[leg_name]

        # Transform the target position from the body frame to the leg's local frame
        local_x = target_position[0] - base_pos[0]
        local_y = target_position[1] - base_pos[1]
        local_z = target_position[2] - base_pos[2]

        # Compute the distance in the horizontal (XY) plane
        len_xy = math.sqrt(local_x**2 + local_y**2)

        # Compute the coxa angle: the angle in the horizontal plane from the leg's base
        coxa_angle = math.atan2(local_y, local_x) - orientation

        # Adjust the horizontal distance by subtracting the coxa length
        foot_offset_x = len_xy - coxa_length

        # Compute the distance from the femur joint to the target foot position
        leg_length = math.sqrt(foot_offset_x**2 + local_z**2)

        # Check if the target is reachable; if not, clamp leg_length
        if leg_length > (femur_length + tibia_length):
            self.get_logger().warning(f"Target position unreachable for leg {leg_name}, clamping distance")
            leg_length = femur_length + tibia_length - 0.001

        # Use the law of cosines to compute the tibia angle
        cos_angle = (femur_length**2 + tibia_length**2 - leg_length**2) / (2 * femur_length * tibia_length)
        # Clamp to valid range in case of numerical issues
        cos_angle = max(-1.0, min(1.0, cos_angle))
        tibia_angle = -(math.pi - math.acos(cos_angle))

        # Compute the femur angle
        gamma = math.atan2(local_z, foot_offset_x)
        cos_beta = (femur_length**2 + leg_length**2 - tibia_length**2) / (2 * femur_length * leg_length)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        femur_angle = -(gamma + beta)

        return [coxa_angle, femur_angle, tibia_angle]

    
    def leg_target_callback(self, msg, leg_name):
        """
        Callback for leg target position subscribers
        
        Args:
            msg (Point): Target position for the foot
            leg_name (str): Name of the leg
        """
        # Extract target position
        target = [msg.x, msg.y, msg.z]
        
        # Compute inverse kinematics
        joint_angles = self.inverse_kinematics(leg_name, target)
        
        # Create joint command message
        cmd = Float64MultiArray()
        cmd.data = joint_angles
        
        # Call the joint command callback
        self.joint_command_callback(cmd, leg_name)
    
    def set_servo_angle(self, joint, angle):
        """
        Set a servo to a specific angle
        
        Args:
            joint (str): Joint name
            angle (float): Angle in degrees (0-180)
        """
        if not hasattr(self, 'servos_enabled') or not self.servos_enabled:
            # In simulation mode, just log the command
            self.get_logger().info(f"Setting {joint} to {angle} degrees (simulation mode)")
            return
            
        if joint not in self.joint_to_servo_map:
            self.get_logger().error(f"Unknown joint: {joint}")
            return
    
        if joint not in self.joint_to_servo_map:
            self.get_logger().error(f"Unknown joint: {joint}")
            return
        
        board_idx, channel = self.joint_to_servo_map[joint]
        
        try:
            if board_idx == 0:
                self.kit1.set_servo_angle(channel, angle)
            else:
                self.kit2.set_servo_angle(channel, angle)
        except Exception as e:
            self.get_logger().error(f"Error setting servo angle for {joint}: {e}")
    
    def publish_servo_states(self):
        """
        Periodically publish the current servo states
        """
        pass  # Implement if needed for diagnostics

    def set_home_position(self):
        """Put all legs in a neutral standing position"""
        self.get_logger().info("Setting robot to home position")
        
        # Define neutral joint angles in radians for each leg
        home_angles = {
            'front_left': [0.0, -0.5, -0.5],    # [coxa, femur, tibia]
            'middle_left': [0.0, -0.5, -0.5],
            'back_left': [0.0, -0.5, -0.5],
            'back_right': [0.0, -0.5, -0.5],
            'middle_right': [0.0, -0.5, -0.5],
            'front_right': [0.0, -0.5, -0.5]
        }
        
        # Apply these angles to each leg
        for leg_name, angles in home_angles.items():
            cmd = Float64MultiArray()
            cmd.data = angles
            self.joint_command_callback(cmd, leg_name)
        
        self.get_logger().info("Home position set")
    
    def depth_image_callback(self, msg):
        """
        Callback for processing depth images.
        
        Args:
            msg (Image): Depth image message.
        """
        try:
            # Convert ROS Image message to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Process the depth image (e.g., find obstacles)
            min_distance = np.min(depth_image)
            self.get_logger().info(f'Minimum distance detected: {min_distance:.2f} meters')
            
            # Add logic to react to obstacles if necessary
            if min_distance < 0.5:  # Example threshold
                self.get_logger().warn('Obstacle detected! Stopping servos.')
                self.servos_enabled = False
            else:
                self.servos_enabled = True
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HexapodServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Move all servos to a safe position before shutting down
        for joint in node.joint_to_servo_map.keys():
            node.set_servo_angle(joint, 90)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()