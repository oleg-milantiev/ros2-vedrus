#!/usr/bin/env python3
"""
ROS2 Keyboard Control Node for Robot Motor Testing.

This node provides a simple keyboard-based interface for testing robot motor control.
It captures keyboard input and translates it into motor speed commands, allowing
for manual control and testing of the robot's movement system.

Keyboard Controls:
----------------
Speed Control:
- '+': Increase speed (+500)
- '-': Decrease speed (-500)

Movement Directions:
- '5': Stop robot
- '4': Rotate left (left motor reverse, right motor forward)
- '6': Rotate right (left motor forward, right motor reverse)
- '8': Move forward (both motors forward)
- '2': Move backward (both motors reverse)
- '7': Move forward and left (left motor half speed, right motor full speed)
- '9': Move forward and right (left motor full speed, right motor half speed)
- '1': Move backward and left (left motor half speed, right motor full speed)
- '3': Move backward and right (left motor full speed, right motor half speed)

Motor Control:
------------
- Speed Range: Increments/decrements by 500 units
- Direction: Determined by speed value sign (positive = forward, negative = reverse)
- Control Rate: Immediate response to key press

Published Topics:
---------------
1. /vedrus/motor/command (vedrus_interfaces/msg/MotorCommand)
   - header: std_msgs/Header with frame_id indicating left or right motor
   - speed: int16 speed value

Dependencies:
------------
- ROS2
- getch package (pip install getch)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vedrus_interfaces.msg import MotorCommand
import getch

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_left = self.create_publisher(MotorCommand, '/vedrus/motor/command', 10)
        self.publisher_right = self.create_publisher(MotorCommand, '/vedrus/motor/command', 10)
        self.speed = 0
        self.lastKey = None
        self.get_logger().info('Keyboard Controller Node has been started.')
        self.print_keys_hint()

    def print_keys_hint(self):
        keys_hint = """
        Keyboard Controls:
        ----------------
        Speed Control:
        - '+': Increase speed (+500)
        - '-': Decrease speed (-500)

        Movement Directions:
        - '5': Stop robot
        - '4': Rotate left (left motor reverse, right motor forward)
        - '6': Rotate right (left motor forward, right motor reverse)
        - '8': Move forward (both motors forward)
        - '2': Move backward (both motors reverse)
        - '7': Move forward and left (left motor half speed, right motor full speed)
        - '9': Move forward and right (left motor full speed, right motor half speed)
        - '1': Move backward and left (left motor half speed, right motor full speed)
        - '3': Move backward and right (left motor full speed, right motor half speed)
        """
        print(keys_hint)

    def publish_command(self, left_speed, right_speed):
        left_msg = MotorCommand()
        right_msg = MotorCommand()
        left_msg.header = Header()
        right_msg.header = Header()
        now = self.get_clock().now().to_msg()
        left_msg.header.stamp = now
        right_msg.header.stamp = now
        left_msg.header.frame_id = 'left'
        right_msg.header.frame_id = 'right'
        left_msg.speed = left_speed
        right_msg.speed = right_speed
        self.publisher_left.publish(left_msg)
        self.publisher_right.publish(right_msg)
        self.get_logger().info(f'Published {left_speed} / {right_speed}')

    def process_key(self, key):
        if key == '5':
            self.publish_command(0, 0)
        elif key == '4':
            self.publish_command(-self.speed, self.speed)
        elif key == '6':
            self.publish_command(self.speed, -self.speed)
        elif key == '8':
            self.publish_command(self.speed, self.speed)
        elif key == '2':
            self.publish_command(-self.speed, -self.speed)
        elif key == '7':
            self.publish_command(self.speed // 2, self.speed)
        elif key == '9':
            self.publish_command(self.speed, self.speed // 2)
        elif key == '1':
            self.publish_command(-self.speed // 2, -self.speed)
        elif key == '3':
            self.publish_command(-self.speed, -self.speed // 2)

    def run(self):
        while rclpy.ok():
            key = getch.getch()
            if key == '+':
                self.speed = min(self.speed + 500, 10000)
                self.get_logger().info(f'Speed increased to: {self.speed}')
                if self.lastKey:
                    self.process_key(self.lastKey)
            elif key == '-':
                self.speed = max(self.speed - 500, 0)
                self.get_logger().info(f'Speed decreased to: {self.speed}')
                if self.lastKey:
                    self.process_key(self.lastKey)
            else:
                self.lastKey = key
                self.process_key(key)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()