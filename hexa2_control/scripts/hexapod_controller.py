#!/usr/bin/env python3
# hexapod_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String, Bool
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time

class ControlMode(enumerate):
    TELEOP = 0
    AUTONOMOUS = 1
    DEMO = 2

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # Control parameters
        self.control_mode = ControlMode.TELEOP
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.current_gait = "tripod"
        
        # Joystick deadzone
        self.joy_deadzone = 0.1
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gait_pub = self.create_publisher(String, '/hexapod/set_gait', 10)
        self.adaptive_pub = self.create_publisher(Bool, '/hexapod/adaptive_mode', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/hexapod/imu/data',
            self.imu_callback,
            10
        )
        
        # For demonstrations
        self.demo_timer = None
        self.current_demo_step = 0
        
        # For path planning
        self.path_points = []
        self.current_path_point = 0
        
        self.get_logger().info('Hexapod Controller initialized')
        self.get_logger().info('Control mode: TELEOP')
        
        # Set default gait
        self.set_gait("tripod")
        
    def joy_callback(self, msg):
        """
        Process joystick input for teleop control
        
        Args:
            msg (Joy): Joystick message
        """
        if self.control_mode != ControlMode.TELEOP:
            return
        
        # Create Twist message for velocity command
        twist = Twist()
        
        # Map left stick X/Y to robot movement
        # Apply deadzone
        left_x = msg.axes[0] if abs(msg.axes[0]) > self.joy_deadzone else 0.0
        left_y = msg.axes[1] if abs(msg.axes[1]) > self.joy_deadzone else 0.0
        
        # Map right stick X to rotation
        right_x = msg.axes[3] if abs(msg.axes[3]) > self.joy_deadzone else 0.0
        
        # Set linear and angular velocities
        twist.linear.x = left_y * self.max_linear_speed  # Forward/backward
        twist.linear.y = left_x * self.max_linear_speed  # Left/right
        twist.angular.z = right_x * self.max_angular_speed  # Rotation
        
        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Check for gait change buttons
        # Assuming buttons are mapped as follows:
        # Button 0 (A): Tripod gait
        # Button 1 (B): Ripple gait
        # Button 2 (X): Wave gait
        # Button 3 (Y): Toggle adaptive mode
        
        if msg.buttons[0] and self.current_gait != "tripod":
            self.set_gait("tripod")
        elif msg.buttons[1] and self.current_gait != "ripple":
            self.set_gait("ripple")
        elif msg.buttons[2] and self.current_gait != "wave":
            self.set_gait("wave")
        
        # Toggle adaptive mode
        if msg.buttons[3]:
            self.toggle_adaptive_mode()
        
        # Start demo mode (using Select button)
        if msg.buttons[8]:
            self.start_demo()
        
        # Stop all movement (using Start button)
        if msg.buttons[9]:
            self.stop_movement()
    
    def imu_callback(self, msg):
        """
        Process IMU data
        
        Args:
            msg (Imu): IMU message
        """
        # Here we could implement additional logic based on IMU data
        # For now, we don't need to do anything as the gait controller
        # already processes IMU data for balancing
        pass
    
    def set_gait(self, gait_name):
        """
        Set the gait pattern
        
        Args:
            gait_name (str): Name of the gait ('tripod', 'ripple', 'wave')
        """
        gait_msg = String()
        gait_msg.data = gait_name
        self.gait_pub.publish(gait_msg)
        self.current_gait = gait_name
        self.get_logger().info(f'Set gait to {gait_name}')
    
    def toggle_adaptive_mode(self):
        """Toggle adaptive walking mode"""
        # TODO: Implement an actual toggle rather than just enabling
        adaptive_msg = Bool()
        adaptive_msg.data = True
        self.adaptive_pub.publish(adaptive_msg)
        self.get_logger().info('Toggled adaptive walking mode')
    
    def stop_movement(self):
        """Stop all movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Stopped movement')
    
    def start_demo(self):
        """Start a demonstration sequence"""
        if self.demo_timer is not None:
            self.demo_timer.cancel()
        
        self.control_mode = ControlMode.DEMO
        self.current_demo_step = 0
        self.get_logger().info('Starting demo mode')
        
        # Create a timer for the demo sequence
        self.demo_timer = self.create_timer(1.0, self.demo_step)
    
    def demo_step(self):
        """Execute a step in the demo sequence"""
        # Simple demo sequence
        demo_steps = [
            # (twist_linear_x, twist_linear_y, twist_angular_z, gait, duration)
            (0.2, 0.0, 0.0, "tripod", 3),  # Walk forward with tripod gait
            (0.0, 0.0, 0.5, "tripod", 2),  # Turn right
            (0.2, 0.0, 0.0, "ripple", 3),  # Walk forward with ripple gait
            (0.0, 0.0, -0.5, "ripple", 2),  # Turn left
            (0.2, 0.0, 0.0, "wave", 3),    # Walk forward with wave gait
            (0.0, 0.0, 0.0, "tripod", 1),  # Stop
        ]
        
        if self.current_demo_step >= len(demo_steps):
            # End of demo
            self.stop_demo()
            return
        
        # Get current step parameters
        step = demo_steps[self.current_demo_step]
        linear_x, linear_y, angular_z, gait, duration = step
        
        # Set gait if different from current
        if gait != self.current_gait:
            self.set_gait(gait)
        
        # Set velocity
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f'Demo step {self.current_demo_step + 1}/{len(demo_steps)}: ' +
                             f'vx={linear_x}, vy={linear_y}, wz={angular_z}, gait={gait}')
        
        # Update timer for next step
        self.demo_timer.cancel()
        self.demo_timer = self.create_timer(duration, self.demo_step)
        
        # Increment step counter
        self.current_demo_step += 1
    
    def stop_demo(self):
        """Stop the demo sequence"""
        if self.demo_timer is not None:
            self.demo_timer.cancel()
            self.demo_timer = None
        
        self.control_mode = ControlMode.TELEOP
        self.stop_movement()
        self.get_logger().info('Demo complete, returning to teleop mode')
    
    def follow_path(self, points):
        """
        Follow a predefined path
        
        Args:
            points (list): List of (x, y) points to visit
        """
        if len(points) == 0:
            self.get_logger().warning('Empty path provided')
            return
        
        self.control_mode = ControlMode.AUTONOMOUS
        self.path_points = points
        self.current_path_point = 0
        
        self.get_logger().info(f'Starting path following with {len(points)} points')
        
        # Start following the path
        self.navigate_to_next_point()
    
    def navigate_to_next_point(self):
        """Navigate to the next point in the path"""
        if self.current_path_point >= len(self.path_points):
            self.get_logger().info('Path complete')
            self.control_mode = ControlMode.TELEOP
            self.stop_movement()
            return
        
        # Get next point
        next_point = self.path_points[self.current_path_point]
        
        # TODO: Implement actual path navigation
        # For now, we just log the path point
        self.get_logger().info(f'Navigating to point {self.current_path_point + 1}: ({next_point[0]}, {next_point[1]})')
        
        # Move to next point in some demo time
        self.create_timer(3.0, self.path_point_reached)
    
    def path_point_reached(self):
        """Called when a path point is reached"""
        self.current_path_point += 1
        self.navigate_to_next_point()

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop movement before shutting down
        node.stop_movement()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()