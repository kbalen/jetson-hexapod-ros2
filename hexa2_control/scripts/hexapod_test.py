#!/usr/bin/env python3
# Simple hexapod test script

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

class HexapodTest(Node):
    def __init__(self):
        super().__init__('hexapod_test')
        
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
        
        # Inverted joints (based on URDF axis directions)
        # From URDF: j11, j12, j14, j15, j17, j18 use axis xyz="0 0 -1"
        self.inverted_joints = ['j11', 'j12', 'j14', 'j15', 'j17', 'j18']
        
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
        
        # Store current angles
        self.current_angles = {joint: 90.0 for joint in self.joint_to_servo_map.keys()}
        
        self.get_logger().info('Hexapod Test initialized')
        
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
            self.get_logger().info(f"Set {joint} to {servo_angle} degrees")
        except Exception as e:
            self.get_logger().error(f"Error setting servo angle for {joint}: {e}")
    
    def reset_all_servos(self):
        """Set all servos to central position"""
        self.get_logger().info("Resetting all servos to center position")
        for joint in self.joint_to_servo_map.keys():
            self.set_servo_angle(joint, 90)
            time.sleep(0.1)  # Small delay between commands
        self.get_logger().info("All servos reset")
    
    def test_joint_sequence(self):
        """Test each joint individually in sequence"""
        self.get_logger().info("Starting joint test sequence")
        
        # First center all servos
        self.reset_all_servos()
        time.sleep(2)
        
        # Test each joint
        for joint in self.joint_to_servo_map.keys():
            self.get_logger().info(f"Testing {joint}")
            
            # Move to minimum (60 degrees)
            self.set_servo_angle(joint, 60)
            time.sleep(1)
            
            # Move to center (90 degrees)
            self.set_servo_angle(joint, 90)
            time.sleep(1)
            
            # Move to maximum (120 degrees)
            self.set_servo_angle(joint, 120)
            time.sleep(1)
            
            # Return to center
            self.set_servo_angle(joint, 90)
            time.sleep(1)
        
        self.get_logger().info("Joint test sequence complete")

def main(args=None):
    rclpy.init(args=args)
    node = HexapodTest()
    
    try:
        # First reset all servos
        node.reset_all_servos()
        time.sleep(2)
        
        # Then test each joint
        node.test_joint_sequence()
        
        # Finally, reset all servos again
        node.reset_all_servos()
        
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error during test: {e}")
    finally:
        # Make sure to reset all servos before exiting
        node.reset_all_servos()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()