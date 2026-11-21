#!/usr/bin/env python3
# Simple hexapod walking implementation

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
import time
import math

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

class HexapodWalk(Node):
    def __init__(self):
        super().__init__('hexapod_walk')
        
        # Initialize PCA9685 boards
        try:
            # Use bus 0 (which is where i2cdetect found your devices)
            self.kit1 = PCA9685(0, 0x40)  # First controller
            self.kit2 = PCA9685(0, 0x41)  # Second controller
            
            self.get_logger().info('Servo controllers initialized')
            self.servos_enabled = True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo controllers: {e}')
            self.servos_enabled = False
        
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
            'j9': (0, 15),    # Back Left Tibia
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
        
        # Define which joints have inverted direction
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
        
        # Load calibration
        self.calibration = {}
        try:
            from servo_calibration import calibration
            self.calibration = calibration
            self.get_logger().info('Loaded calibration from file')
        except ImportError:
            self.get_logger().info('No calibration file found, using defaults')
            for joint in self.joint_to_servo_map.keys():
                self.calibration[joint] = 0.0
        
        # Define tripod gait groups
        self.tripod_groups = [
            ['front_left', 'middle_right', 'back_left'],  # Group 1
            ['front_right', 'middle_left', 'back_right']   # Group 2
        ]
        
        # Store current angles
        self.current_angles = {joint: 90.0 for joint in self.joint_to_servo_map.keys()}
        
        self.get_logger().info('Hexapod Walk initialized')
    
    def rad_to_deg(self, rad):
        """Convert radians to degrees"""
        return rad * 180.0 / math.pi
    
    def deg_to_rad(self, deg):
        """Convert degrees to radians"""
        return deg * math.pi / 180.0
        
    def set_servo_angle(self, joint, angle_deg):
        """
        Set a servo to a specific angle
        
        Args:
            joint (str): Joint name
            angle_deg (float): Angle in degrees (0-180)
        """
        if not self.servos_enabled:
            self.get_logger().info(f"Setting {joint} to {angle_deg} degrees (simulation mode)")
            return
            
        if joint not in self.joint_to_servo_map:
            self.get_logger().error(f"Unknown joint: {joint}")
            return
        
        board_idx, channel = self.joint_to_servo_map[joint]
        
        # Apply calibration offset
        servo_angle = angle_deg + self.calibration.get(joint, 0.0)
        
        # Ensure within servo range
        servo_angle = max(0, min(180, servo_angle))
        
        try:
            if board_idx == 0:
                self.kit1.set_servo_angle(channel, servo_angle)
            else:
                self.kit2.set_servo_angle(channel, servo_angle)
                
            # Update current angle
            self.current_angles[joint] = servo_angle
        except Exception as e:
            self.get_logger().error(f"Error setting servo angle for {joint}: {e}")
    
    def set_joint_angle_rad(self, joint, angle_rad):
        """
        Set a joint to a specific angle in radians, handling inverted joints and limits
        
        Args:
            joint (str): Joint name
            angle_rad (float): Angle in radians
        """
        # Apply joint limits
        if joint in self.joint_limits:
            min_angle, max_angle = self.joint_limits[joint]
            angle_rad = max(min_angle, min(max_angle, angle_rad))
        
        # Convert to degrees
        angle_deg = self.rad_to_deg(angle_rad)
        
        # Apply inversion if needed
        if joint in self.inverted_joints:
            angle_deg = -angle_deg
        
        # Map to servo range (0-180) - neutral at 90
        servo_angle = 90 + angle_deg
        
        # Set the servo
        self.set_servo_angle(joint, servo_angle)
    
    def set_leg_position(self, leg_name, angles):
        """
        Set a leg's joint angles
        
        Args:
            leg_name (str): Name of the leg
            angles (list): List of 3 angles [coxa, femur, tibia] in radians
        """
        if leg_name not in self.leg_to_joints:
            self.get_logger().error(f"Unknown leg: {leg_name}")
            return
        
        joints = self.leg_to_joints[leg_name]
        
        if len(angles) != 3:
            self.get_logger().error(f"Expected 3 angles for leg {leg_name}, got {len(angles)}")
            return
        
        for i, joint in enumerate(joints):
            self.set_joint_angle_rad(joint, angles[i])
    
    def reset_all_servos(self):
        """Set all servos to neutral position (90 degrees)"""
        self.get_logger().info("Resetting all servos to neutral position")
        for joint in self.joint_to_servo_map.keys():
            self.set_servo_angle(joint, 90)
            time.sleep(0.05)  # Small delay between commands
        self.get_logger().info("All servos reset")
    
    def set_home_position(self):
        """Set robot to a stable standing position"""
        self.get_logger().info("Setting robot to home position")
        
        # Define standing pose angles in radians for each leg
        # [coxa, femur, tibia]
        standing_angles = {
            'front_left': [0.0, -0.5, -0.5],
            'middle_left': [0.0, -0.5, -0.5],
            'back_left': [0.0, -0.5, -0.5],
            'back_right': [0.0, -0.5, -0.5],
            'middle_right': [0.0, -0.5, -0.5],
            'front_right': [0.0, -0.5, -0.5]
        }
        
        # Apply these angles to each leg
        for leg_name, angles in standing_angles.items():
            self.set_leg_position(leg_name, angles)
            time.sleep(0.1)  # Small delay between legs
        
        self.get_logger().info("Home position set")
    
    def tripod_gait_step(self, step_num, steps_per_cycle=10):
        """
        Execute one step of a tripod gait
        
        Args:
            step_num (int): Current step number in the cycle
            steps_per_cycle (int): Total steps per complete gait cycle
        """
        # Calculate phase (0.0 to 1.0) within the cycle
        phase = step_num / steps_per_cycle
        
        # Define leg parameters - adjust these based on your robot's capabilities
        step_height = 0.04  # Height of leg lift during step (radians)
        stride_angle = 0.3  # Coxa angle range during stride (radians)
        
        # For each tripod group
        for group_idx, group in enumerate(self.tripod_groups):
            # Group 0 is in phase, Group 1 is 180° out of phase (0.5 offset)
            group_phase = (phase + 0.5 * group_idx) % 1.0
            
            for leg_name in group:
                # In the first half of the phase (0.0-0.5), the leg is in the air (swing phase)
                # In the second half (0.5-1.0), the leg is on the ground (stance phase)
                
                if group_phase < 0.5:  # Swing phase (moving forward in the air)
                    # Normalize to 0-1 range for swing phase
                    swing_phase = group_phase * 2.0
                    
                    # Coxa angles from back (-stride_angle/2) to front (stride_angle/2)
                    coxa_angle = -stride_angle/2 + stride_angle * swing_phase
                    
                    # Femur and tibia angles to lift the leg (following a sine curve)
                    # At swing_phase=0 and swing_phase=1, the leg is down
                    # At swing_phase=0.5, the leg is at maximum height
                    femur_angle = -0.5 - step_height * math.sin(swing_phase * math.pi)
                    tibia_angle = -0.5 - step_height * math.sin(swing_phase * math.pi)
                    
                else:  # Stance phase (on ground, moving backward)
                    # Normalize to 0-1 range for stance phase
                    stance_phase = (group_phase - 0.5) * 2.0
                    
                    # Coxa angles from front (stride_angle/2) to back (-stride_angle/2)
                    coxa_angle = stride_angle/2 - stride_angle * stance_phase
                    
                    # Femur and tibia angles for standing
                    femur_angle = -0.5  # Standing position
                    tibia_angle = -0.5  # Standing position
                
                # Apply the calculated angles
                self.set_leg_position(leg_name, [coxa_angle, femur_angle, tibia_angle])
    
    def walk_forward(self, steps=40, delay=0.05):
        """
        Walk forward using tripod gait
        
        Args:
            steps (int): Number of steps to take
            delay (float): Delay between steps in seconds
        """
        self.get_logger().info(f"Walking forward for {steps} steps")
        
        # First, set home position
        self.set_home_position()
        time.sleep(1.0)  # Wait to stabilize
        
        steps_per_cycle = 10
        for step in range(steps):
            self.tripod_gait_step(step % steps_per_cycle, steps_per_cycle)
            time.sleep(delay)
        
        # Return to home position
        self.set_home_position()
        self.get_logger().info("Walking complete")

def main(args=None):
    rclpy.init(args=args)
    node = HexapodWalk()
    
    try:
        # First reset all servos
        node.reset_all_servos()
        time.sleep(1)
        
        # Set to home position
        node.set_home_position()
        time.sleep(2)
        
        # Walk forward
        node.walk_forward(steps=30, delay=0.2)
        
        # Return to home position
        node.set_home_position()
        
    except KeyboardInterrupt:
        node.get_logger().info("Walk interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error during walk: {e}")
    finally:
        # Make sure to set home position before exiting
        node.set_home_position()
        time.sleep(1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()