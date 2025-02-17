#!/usr/bin/env python3
"""
ROS2 Keyboard Control Node for Robot Motor Testing.

This node provides a simple keyboard-based interface for testing robot motor control.
It captures keyboard input and translates it into motor speed commands, allowing
for manual control and testing of the robot's movement system.

Keyboard Controls:
----------------
Left Motor:
- 'q': Increase speed (+500)
- 'a': Decrease speed (-500)

Right Motor:
- 'w': Increase speed (+500)
- 's': Decrease speed (-500)

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
- ROS2 Humble or newer
- getch package (pip install getch)
- vedrus_interfaces package

Start:
ros2 run vedrus keyboard

Usage:
-----
1. Start the node: ros2 run vedrus keyboard
2. Use keys (q,a,w,s) to control motors
3. Monitor terminal for speed level feedback
4. Ctrl+C to exit

Note:
----
This is a testing utility and should be used with caution.
Consider implementing additional safety features for production use.
"""

import rclpy
import getch
from rclpy.node import Node
from std_msgs.msg import Header
from vedrus_interfaces.msg import MotorCommand

class VedrusTestNode(Node):

    left_speed = 0
    right_speed = 0

    def __init__(self):
        super().__init__('vedrus_test')

        self.publisher = self.create_publisher(MotorCommand, '/vedrus/motor/command', 10)

        while (1):
            was_left = self.left_speed
            was_right = self.right_speed

            match getch.getch():
                case 'q':
                    self.left_speed += 500
                case 'a':
                    self.left_speed -= 500
                case 'w':
                    self.right_speed += 500
                case 's':
                    self.right_speed -= 500

            if was_left != self.left_speed:
                msg = MotorCommand()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'left'
                msg.speed = self.left_speed
                print(f'Left Motor Speed: {self.left_speed}')
                self.publisher.publish(msg)

            if was_right != self.right_speed:
                msg = MotorCommand()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'right'
                msg.speed = self.right_speed
                print(f'Right Motor Speed: {self.right_speed}')
                self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    vedrus_test_node = VedrusTestNode()
    rclpy.spin(vedrus_test_node)

    vedrus_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()