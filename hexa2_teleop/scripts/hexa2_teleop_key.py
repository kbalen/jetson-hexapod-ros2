#!/usr/bin/env python3
# Save this file as ~/hexapod_ws/src/hexapod_teleop/scripts/hexapod_teleop_key.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import threading

msg = """
Hexapod Teleop Control
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity (forward/backward)
a/d : increase/decrease linear velocity (left/right)
q/e : increase/decrease angular velocity (rotation)

s : stop
CTRL-C to quit
"""

# Key mapping
moveBindings = {
    'w': (1, 0, 0),    # forward
    'x': (-1, 0, 0),   # backward
    'a': (0, 1, 0),    # left
    'd': (0, -1, 0),   # right
    'q': (0, 0, 1),    # turn left
    'e': (0, 0, -1),   # turn right
    'z': (-0.5, 0.5, 0),  # backward + left
    'c': (-0.5, -0.5, 0)  # backward + right
}

speedBindings = {
    'i': (1.1, 1.1),   # increase linear and angular speed
    'k': (0.9, 0.9)    # decrease linear and angular speed
}

class HexapodTeleopKey(Node):
    def __init__(self):
        super().__init__('hexapod_teleop_key')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize velocities
        self.speed = 0.2  # m/s
        self.turn = 0.5   # rad/s
        
        # Initialize twist message
        self.twist = Twist()
        
        self.get_logger().info('Hexapod teleop initialized\n%s' % msg)
        
        # Start keyboard input thread
        self.key_thread = threading.Thread(target=self.get_key)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        # Create a timer for publishing commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def get_key(self):
        """Get keypress and process input"""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                # Wait for keypress
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # Process key
                    if key in moveBindings.keys():
                        # Update velocities based on key
                        x, y, z = moveBindings[key]
                        self.twist.linear.x = x * self.speed
                        self.twist.linear.y = y * self.speed
                        self.twist.angular.z = z * self.turn
                        
                    elif key in speedBindings.keys():
                        # Update speed settings
                        speed_factor, turn_factor = speedBindings[key]
                        self.speed = self.speed * speed_factor
                        self.turn = self.turn * turn_factor
                        
                        self.get_logger().info('Speed: %s, Turn: %s' % (self.speed, self.turn))
                        
                    elif key == 's':
                        # Stop
                        self.twist.linear.x = 0.0
                        self.twist.linear.y = 0.0
                        self.twist.linear.z = 0.0
                        self.twist.angular.x = 0.0
                        self.twist.angular.y = 0.0
                        self.twist.angular.z = 0.0
                        
                    elif key == '\x03':  # CTRL-C
                        # Exit
                        break
                        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def timer_callback(self):
        """Publish velocity commands periodically"""
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodTeleopKey()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before exiting
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()