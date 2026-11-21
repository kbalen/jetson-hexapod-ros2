#!/usr/bin/env python3
# hexapod_imu.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
import smbus2
import numpy as np

# MPU6050 Constants
MPU6050_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

class HexapodIMU(Node):
    def __init__(self):
        super().__init__('hexapod_imu')
        
        # Initialize I2C bus
        self.bus_num = 0  # Use bus 0 for Jetson Nano in Docker
        self.bus = smbus2.SMBus(self.bus_num)
        
        # Initialize MPU6050
        try:
            # Wake up the MPU6050
            self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            
            # Set sample rate
            self.bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
            
            # Set low pass filter
            self.bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
            
            # Set gyro range to ±250°/s
            self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
            
            # Set accelerometer range to ±2g
            self.bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
            
            self.get_logger().info('MPU6050 IMU initialized')
            self.imu_initialized = True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {e}')
            self.get_logger().warning('Running without IMU feedback')
            self.imu_initialized = False
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, '/hexapod/imu/data', 10)
        
        # Timer for reading IMU data
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz
        
        # For complementary filter
        self.last_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Calibration values (should be determined experimentally)
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        # Load calibration if available
        self.load_calibration()
    
    def read_raw_data(self, addr):
        """Read raw 16-bit data from MPU6050"""
        try:
            high = self.bus.read_byte_data(MPU6050_ADDR, addr)
            low = self.bus.read_byte_data(MPU6050_ADDR, addr+1)
            
            # Combine high and low bytes
            value = (high << 8) | low
            
            # Convert to signed value
            if value > 32767:
                value -= 65536
                
            return value
        except Exception as e:
            self.get_logger().error(f'Error reading from MPU6050: {e}')
            return 0
    
    def read_accel_gyro_data(self):
        """Read accelerometer and gyroscope data"""
        if not self.imu_initialized:
            return [0, 0, 0], [0, 0, 0]  # Return zeros if IMU not initialized
        
        try:
            # Read accelerometer data
            accel_x = self.read_raw_data(ACCEL_XOUT_H)
            accel_y = self.read_raw_data(ACCEL_XOUT_H + 2)
            accel_z = self.read_raw_data(ACCEL_XOUT_H + 4)
            
            # Read gyroscope data
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
            gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
            
            # Convert to physical units
            # Accelerometer: ±2g range -> ±16384 raw values
            accel_x = (accel_x / 16384.0) - self.accel_offset[0]
            accel_y = (accel_y / 16384.0) - self.accel_offset[1]
            accel_z = (accel_z / 16384.0) - self.accel_offset[2]
            
            # Gyroscope: ±250°/s range -> ±131 raw values
            gyro_x = (gyro_x / 131.0) - self.gyro_offset[0]
            gyro_y = (gyro_y / 131.0) - self.gyro_offset[1]
            gyro_z = (gyro_z / 131.0) - self.gyro_offset[2]
            
            return [accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z]
        
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')
            return [0, 0, 0], [0, 0, 0]
    
    def complementary_filter(self, accel, gyro):
        """Apply complementary filter to combine accelerometer and gyroscope data"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Calculate accelerometer angles
        accel_roll = math.atan2(accel[1], accel[2]) * 180.0 / math.pi
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / math.pi
        
        # Integrate gyroscope data
        self.roll = 0.98 * (self.roll + gyro[0] * dt) + 0.02 * accel_roll
        self.pitch = 0.98 * (self.pitch + gyro[1] * dt) + 0.02 * accel_pitch
        
        # Integrate yaw (no accelerometer correction for yaw)
        self.yaw += gyro[2] * dt
        
        return self.roll, self.pitch, self.yaw
    
    def publish_imu_data(self):
        """Read IMU data and publish to ROS topic"""
        if not self.imu_initialized:
            return
            
        accel, gyro = self.read_accel_gyro_data()
        roll, pitch, yaw = self.complementary_filter(accel, gyro)
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation (as quaternion)
        quat = quaternion_from_euler(
            roll * math.pi / 180.0, 
            pitch * math.pi / 180.0, 
            yaw * math.pi / 180.0, 
            'rxyz'
        )
        
        imu_msg.orientation = Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )
        
        # Set angular velocity
        imu_msg.angular_velocity = Vector3(
            x=float(gyro[0] * math.pi / 180.0),  # Convert to rad/s
            y=float(gyro[1] * math.pi / 180.0),
            z=float(gyro[2] * math.pi / 180.0)
        )
        
        # Set linear acceleration
        imu_msg.linear_acceleration = Vector3(
            x=float(accel[0] * 9.81),  # Convert to m/s²
            y=float(accel[1] * 9.81),
            z=float(accel[2] * 9.81)
        )
        
        # Publish message
        self.imu_pub.publish(imu_msg)
    
    def calibrate_imu(self, samples=500):
        """Calibrate the IMU by reading samples and computing offsets"""
        if not self.imu_initialized:
            self.get_logger().error('Cannot calibrate: IMU not initialized')
            return
            
        self.get_logger().info('Calibrating IMU. Keep the robot still...')
        
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        
        # Collect samples
        for _ in range(samples):
            accel, gyro = self.read_accel_gyro_data()
            
            # Sum raw values
            for i in range(3):
                accel_sum[i] += accel[i] + self.accel_offset[i]
                gyro_sum[i] += gyro[i] + self.gyro_offset[i]
                
            time.sleep(0.002)  # 2ms delay between samples
        
        # Calculate average offsets
        self.accel_offset = [accel_sum[i]/samples for i in range(3)]
        self.gyro_offset = [gyro_sum[i]/samples for i in range(3)]
        
        # Adjust Z acceleration to be 1g (9.81 m/s²)
        self.accel_offset[2] -= 1.0
        
        self.get_logger().info('IMU calibration complete')
        self.save_calibration()
    
    def save_calibration(self):
        """Save IMU calibration values to a file"""
        try:
            with open('imu_calibration.py', 'w') as f:
                f.write('# IMU calibration offsets\n')
                f.write('accel_offset = [\n')
                for val in self.accel_offset:
                    f.write(f'    {val},\n')
                f.write(']\n\n')
                
                f.write('gyro_offset = [\n')
                for val in self.gyro_offset:
                    f.write(f'    {val},\n')
                f.write(']\n')
                
            self.get_logger().info('IMU calibration saved to imu_calibration.py')
        except Exception as e:
            self.get_logger().error(f'Error saving IMU calibration: {e}')
    
    def load_calibration(self):
        """Load IMU calibration values from a file if it exists"""
        try:
            # Try to import the calibration file
            from imu_calibration import accel_offset, gyro_offset
            self.accel_offset = accel_offset
            self.gyro_offset = gyro_offset
            self.get_logger().info('Loaded IMU calibration from file')
        except ImportError:
            self.get_logger().info('No IMU calibration file found, using defaults')
        except Exception as e:
            self.get_logger().error(f'Error loading IMU calibration: {e}')
            
def main(args=None):
    rclpy.init(args=args)
    node = HexapodIMU()
    
    try:
        # Calibrate IMU on startup (optional)
        # Uncomment to enable calibration
        # node.calibrate_imu()
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()