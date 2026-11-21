#!/usr/bin/env python3
# Save this file as ~/ros_ws/src/hexa2_control/scripts/calibrate_servos.py

import rclpy
from rclpy.node import Node
import time
import sys
from smbus2 import SMBus

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

class Servo:
    """Simplified servo class to mimic ServoKit interface"""
    def __init__(self, pca, channel):
        self.pca = pca
        self.channel = channel
        self.min_pulse = 500
        self.max_pulse = 2500
        self._angle = 90
    
    @property
    def angle(self):
        return self._angle
    
    @angle.setter
    def angle(self, value):
        self._angle = value
        self.pca.set_servo_angle(self.channel, value)
    
    def set_pulse_width_range(self, min_pulse, max_pulse):
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse

class ServoKit:
    """Simplified ServoKit replacement using direct SMBus access"""
    def __init__(self, channels, address, bus_num=0):
        self.pca = PCA9685(bus_num, address)
        self.servo = [Servo(self.pca, i) for i in range(channels)]

class ServoCalibrator(Node):
    def __init__(self):
        super().__init__('servo_calibrator')
        
        self.get_logger().info('Initializing Servo Calibrator...')
        
        # Initialize PCA9685 boards
        try:
            # Use bus 0 (which is where i2cdetect found your devices)
            self.kit1 = ServoKit(channels=16, address=0x40, bus_num=0)  # First controller
            self.kit2 = ServoKit(channels=16, address=0x41, bus_num=0)  # Second controller
            
            # Set pulse width range for all servos
            for i in range(16):
                self.kit1.servo[i].set_pulse_width_range(500, 2500)  # diymore 13kg servo range
                if i < 2:  # Only 2 servos on second board
                    self.kit2.servo[i].set_pulse_width_range(500, 2500)
                    
            self.get_logger().info('Servo controllers initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo controllers: {e}')
            sys.exit(1)
            
        # Define the leg joint mapping using your URDF joint names
        self.joint_names = [
            'j1', 'j2', 'j3',    # Front Left
            'j4', 'j5', 'j6',    # Middle Left
            'j7', 'j8', 'j9',    # Back Left
            'j10', 'j11', 'j12', # Back Right
            'j13', 'j14', 'j15', # Middle Right
            'j16', 'j17', 'j18'  # Front Right
        ]
        
        # Map each joint to a servo channel on the appropriate PCA9685 board
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
        
        # Define which joints have inverted direction (negative z-axis in URDF)
        self.inverted_joints = ['j11', 'j12', 'j14', 'j15', 'j17', 'j18']
        
        # Create a dictionary to store calibration values
        self.calibration = {joint: 0 for joint in self.joint_names}
        
        self.get_logger().info('Servo Calibrator ready')
        
    def center_all_servos(self):
        """Center all servos at 90 degrees"""
        self.get_logger().info('Centering all servos...')
        
        try:
            for joint in self.joint_names:
                board_idx, channel = self.joint_to_servo_map[joint]
                kit = self.kit1 if board_idx == 0 else self.kit2
                kit.servo[channel].angle = 90
                
            self.get_logger().info('All servos centered')
        except Exception as e:
            self.get_logger().error(f'Error centering servos: {e}')
    
    def test_servo(self, joint_name):
        """Test a specific servo by moving it through its range"""
        try:
            board_idx, channel = self.joint_to_servo_map[joint_name]
            kit = self.kit1 if board_idx == 0 else self.kit2
            
            self.get_logger().info(f'Testing {joint_name} (Board {board_idx}, Channel {channel})')
            
            # First go to center
            kit.servo[channel].angle = 90
            time.sleep(1)
            
            # Move to min position
            kit.servo[channel].angle = 0
            time.sleep(1)
            
            # Move to max position
            kit.servo[channel].angle = 180
            time.sleep(1)
            
            # Back to center
            kit.servo[channel].angle = 90
            self.get_logger().info(f'Test complete for {joint_name}')
            
        except Exception as e:
            self.get_logger().error(f'Error testing servo {joint_name}: {e}')
    
    def calibrate_servo(self, joint_name):
        """Interactive calibration for a specific servo"""
        try:
            board_idx, channel = self.joint_to_servo_map[joint_name]
            kit = self.kit1 if board_idx == 0 else self.kit2
            
            self.get_logger().info(f'Calibrating {joint_name} (Board {board_idx}, Channel {channel})')
            
            # Start at current offset or 90 if none
            current_angle = 90 + self.calibration[joint_name]
            kit.servo[channel].angle = current_angle
            
            while True:
                cmd = input(f"Current offset for {joint_name}: {self.calibration[joint_name]}\n"
                           f"Enter adjustment (-10 to +10), 'q' to quit, 's' to save: ")
                
                if cmd.lower() == 'q':
                    break
                elif cmd.lower() == 's':
                    self.save_calibration()
                    break
                else:
                    try:
                        adjustment = float(cmd)
                        if abs(adjustment) <= 10:
                            self.calibration[joint_name] += adjustment
                            new_angle = 90 + self.calibration[joint_name]
                            # Ensure within range
                            new_angle = max(0, min(180, new_angle))
                            kit.servo[channel].angle = new_angle
                        else:
                            self.get_logger().warning('Adjustment too large, use smaller value')
                    except ValueError:
                        self.get_logger().warning('Invalid input, try again')
            
        except Exception as e:
            self.get_logger().error(f'Error calibrating servo {joint_name}: {e}')
    
    def save_calibration(self):
        """Save calibration values to a file"""
        try:
            with open('servo_calibration.py', 'w') as f:
                f.write('# Servo calibration offsets\n')
                f.write('calibration = {\n')
                for joint, offset in self.calibration.items():
                    f.write(f"    '{joint}': {offset},\n")
                f.write('}\n')
            self.get_logger().info('Calibration saved to servo_calibration.py')
        except Exception as e:
            self.get_logger().error(f'Error saving calibration: {e}')
    
    def load_calibration(self):
        """Load calibration values from a file if it exists"""
        try:
            # Try to import the calibration file
            from servo_calibration import calibration
            self.calibration = calibration
            self.get_logger().info('Loaded calibration from file')
        except ImportError:
            self.get_logger().info('No calibration file found, using defaults')
        except Exception as e:
            self.get_logger().error(f'Error loading calibration: {e}')
    
    def calibrate_all(self):
        """Calibrate all servos one by one"""
        for joint in self.joint_names:
            self.calibrate_servo(joint)
    
    def interactive_menu(self):
        """Display interactive menu for servo testing and calibration"""
        while True:
            print("\nHexapod Servo Calibration Utility")
            print("----------------------------------")
            print("1. Center all servos")
            print("2. Test individual servo")
            print("3. Calibrate individual servo")
            print("4. Calibrate all servos")
            print("5. Save calibration")
            print("6. Load calibration")
            print("7. Standing pose test")
            print("8. Exit")
            
            choice = input("Enter your choice (1-8): ")
            
            if choice == '1':
                self.center_all_servos()
            elif choice == '2':
                self.print_servo_list()
                joint_idx = int(input("Enter servo number to test: "))
                if 0 <= joint_idx < len(self.joint_names):
                    self.test_servo(self.joint_names[joint_idx])
                else:
                    self.get_logger().warning('Invalid servo number')
            elif choice == '3':
                self.print_servo_list()
                joint_idx = int(input("Enter servo number to calibrate: "))
                if 0 <= joint_idx < len(self.joint_names):
                    self.calibrate_servo(self.joint_names[joint_idx])
                else:
                    self.get_logger().warning('Invalid servo number')
            elif choice == '4':
                self.calibrate_all()
            elif choice == '5':
                self.save_calibration()
            elif choice == '6':
                self.load_calibration()
            elif choice == '7':
                self.test_standing_pose()
            elif choice == '8':
                break
            else:
                self.get_logger().warning('Invalid choice, try again')
    
    def print_servo_list(self):
        """Print list of servos with their indices"""
        print("\nServo List:")
        for i, joint in enumerate(self.joint_names):
            board_idx, channel = self.joint_to_servo_map[joint]
            print(f"{i}: {joint} (Board {board_idx}, Channel {channel})")
    
    def test_standing_pose(self):
        """Test the standing pose with all legs down"""
        self.get_logger().info('Setting standing pose...')
        
        try:
            # Create mapping of joint names to positions
            joint_positions = {}
            
            # Map the descriptive names to joint IDs
            leg_mapping = {
                'FL Coxa': 'j1',
                'FL Femur': 'j2',
                'FL Tibia': 'j3',
                'ML Coxa': 'j4',
                'ML Femur': 'j5',
                'ML Tibia': 'j6',
                'BL Coxa': 'j7',
                'BL Femur': 'j8',
                'BL Tibia': 'j9',
                'BR Coxa': 'j10',
                'BR Femur': 'j11',
                'BR Tibia': 'j12',
                'MR Coxa': 'j13',
                'MR Femur': 'j14',
                'MR Tibia': 'j15',
                'FR Coxa': 'j16',
                'FR Femur': 'j17',
                'FR Tibia': 'j18'
            }
            
            # Default standing pose values
            standing_pose = {
                'FL Coxa': 0.0,   # 90 + offset (centered)
                'FL Femur': -30.0, # 60 + offset (angled down)
                'FL Tibia': -30.0, # 60 + offset (angled down)
                'FR Coxa': 0.0,
                'FR Femur': -30.0,
                'FR Tibia': -30.0,
                'ML Coxa': 0.0,
                'ML Femur': -30.0,
                'ML Tibia': -30.0,
                'MR Coxa': 0.0,
                'MR Femur': -30.0,
                'MR Tibia': -30.0,
                'BL Coxa': 0.0,
                'BL Femur': -30.0,
                'BL Tibia': -30.0,
                'BR Coxa': 0.0,
                'BR Femur': -30.0,
                'BR Tibia': -30.0
            }
            
            # Convert descriptive names to joint IDs
            for desc_name, angle in standing_pose.items():
                joint_id = leg_mapping.get(desc_name)
                if joint_id:
                    joint_positions[joint_id] = angle
            
            # Set each servo to its standing pose position
            for joint, angle in joint_positions.items():
                board_idx, channel = self.joint_to_servo_map[joint]
                kit = self.kit1 if board_idx == 0 else self.kit2
                
                # Apply the calibration offset
                adjusted_angle = 90 + angle + self.calibration[joint]
                # Ensure within range
                adjusted_angle = max(0, min(180, adjusted_angle))
                
                kit.servo[channel].angle = adjusted_angle
                
            self.get_logger().info('Standing pose set')
            input("Press Enter to return to center position...")
            self.center_all_servos()
            
        except Exception as e:
            self.get_logger().error(f'Error setting standing pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    calibrator = ServoCalibrator()
    
    try:
        calibrator.load_calibration()
        calibrator.interactive_menu()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        calibrator.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Clean shutdown
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
