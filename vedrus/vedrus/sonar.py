"""
ROS2 Node for Reading Four HC-SR04 Sonar Sensors via Sysfs and Publishing Data

This script reads data from four HC-SR04 sonar sensors using sysfs interfaces and publishes
the readings to a ROS2 topic. The GPIO pins and topic name are configurable through
node parameters.

# Installation:
Ensure the sysfs GPIO interface is enabled on your system. This can typically be done
by exporting the GPIO pins through `/sys/class/gpio/`.

# Usage:
1. Save this script as `sonar_node.py`.
2. Export the GPIO pins for trigger and echo via sysfs:
echo <pin_number> > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio<trig_pin>/direction
echo in > /sys/class/gpio/gpio<echo_pin>/direction
3. Create a launch file (example below).
4. Run the launch file with `ros2 launch`.
5. Alternative use ros2 run your_package hcsr04 --ros-args -p pin_numbers:="[127, 125, 124, 144, 122, 120, 123, 121]"

# ROS2 Launch File Example (Python Style):
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='sonar_node',
            name='sonar_node',
            parameters=[
                {'names': ['sensor1', 'sensor2', 'sensor3', 'sensor4']},  # Names for each sensor
                {'pin_numbers': [127, 125, 124, 144, 122, 120, 123, 121]},  # Example GPIO pins
                {'topic_name': 'sonar_data'},
            ]
        ),
    ])

# ROS2 Topic Subscription Example:
ros2 topic echo /sonar_data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import os

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')

        # Declare and get parameters
        self.declare_parameter('pin_numbers', [127, 125, 124, 144, 122, 120, 123, 121])
        self.declare_parameter('topic_name', 'sonar_data')
        self.declare_parameter('names', ['sensor1', 'sensor2', 'sensor3', 'sensor4'])

        self.pin_numbers = self.get_parameter('pin_numbers').get_parameter_value().integer_array_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.names = self.get_parameter('names').get_parameter_value().string_array_value

        self.trig_pins = self.pin_numbers[:len(self.pin_numbers)//2]
        self.echo_pins = self.pin_numbers[len(self.pin_numbers)//2:]

        # Export GPIO pins and configure direction
        for trig, echo in zip(self.trig_pins, self.echo_pins):
            self._export_pin(trig, "out")
            self._export_pin(echo, "in")

        # Create publisher
        self.publisher_ = self.create_publisher(Range, self.topic_name, 10)
        self.timer = self.create_timer(0.5, self.read_sonars)  # 2 Hz timer

    def _export_pin(self, pin, direction):
        gpio_path = f"/sys/class/gpio/gpio{pin}"
        if not os.path.exists(gpio_path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(pin))
        with open(f"{gpio_path}/direction", "w") as f:
            f.write(direction)

    def _write_gpio(self, pin, value):
        with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
            f.write(str(value))

    def _read_gpio(self, pin):
        with open(f"/sys/class/gpio/gpio{pin}/value", "r") as f:
            return int(f.read().strip())

    def read_sonar(self, trig, echo):
        # Trigger the sonar
        self._write_gpio(trig, 1)
        time.sleep(0.00001)
        self._write_gpio(trig, 0)

        # Measure the echo pulse duration
        start_time = time.time()
        timeout = start_time + 0.1

        while self._read_gpio(echo) == 0:
            start_time = time.time()
            if start_time > timeout:
                return -1

        stop_time = time.time()
        while self._read_gpio(echo) == 1:
            stop_time = time.time()
            if stop_time > timeout:
                return -1

        # Calculate distance (in meters)
        time_elapsed = stop_time - start_time
        return (time_elapsed * 34300) / 2  # Speed of sound = 34300 cm/s

    def read_sonars(self):
        for i, (trig, echo) in enumerate(zip(self.trig_pins, self.echo_pins)):
            distance = self.read_sonar(trig, echo)

            if distance > 0:
                # Publish the Range message
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = f'sonar_{i}'
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.header.frame_id = f'sonar_{i}'
                range_msg.min_range = 0.02  # Minimum measurable range in meters
                range_msg.max_range = 4.0  # Maximum measurable range in meters
                range_msg.range = distance / 100.0  # Convert cm to meters

                self.publisher_.publish(range_msg)

    def destroy_node(self):
        super().destroy_node()
        for pin in self.trig_pins + self.echo_pins:
            with open("/sys/class/gpio/unexport", "w") as f:
                f.write(str(pin))


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
