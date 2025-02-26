"""
ROS2 Node for Reading Four HC-SR04 Sonar Sensors via Sysfs and Publishing Data

This script reads data from four HC-SR04 sonar sensors using sysfs interfaces and publishes
the readings to a ROS2 topic. The GPIO pins, topic name, and azimuths are configurable through
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
                {'pins_trig': [127, 125, 124, 144]},
                {'pins_echo': [122, 120, 123, 121]},
                {'topic_name': 'sonar_data'},
                {'azimuths': [0.0, 90.0, 180.0, 270.0]},  # Example azimuths
            ]
        ),
    ])

# ROS2 Topic Subscription Example:
ros2 topic echo /sonar_data
"""

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from vedrus_interfaces.msg import Sonar
import time
import os
import select
from collections import deque
import statistics

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')

        # Declare and get parameters
        self.declare_parameter('pins_trig', [127, 125, 124, 144])
        self.declare_parameter('pins_echo', [122, 120, 123, 121])
        self.declare_parameter('topic_name', 'sonar_data')
        self.declare_parameter('names', ['sensor1', 'sensor2', 'sensor3', 'sensor4'])
        self.declare_parameter('azimuths', [0.0, 90.0, 180.0, 270.0])

        self.trig_pins = self.get_parameter('pins_trig').get_parameter_value().integer_array_value
        self.echo_pins = self.get_parameter('pins_echo').get_parameter_value().integer_array_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.names = self.get_parameter('names').get_parameter_value().string_array_value
        self.azimuths = self.get_parameter('azimuths').get_parameter_value().double_array_value

        # Initialize sliding buffers for each sensor
        self.buffers = {name: deque(maxlen=8) for name in self.names}

        # Export GPIO pins and configure direction for each sensor
        for trig, echo in zip(self.trig_pins, self.echo_pins):
            self._export_pin(trig, "out")
            self._export_pin(echo, "in")
            self._set_edge(echo, "both")

        self.fd_map = {echo: open(f"/sys/class/gpio/gpio{echo}/value", "r") for echo in self.echo_pins}

        # Create publisher
        self.publisher_ = self.create_publisher(Sonar, self.topic_name, 10)
        self.timer = self.create_timer(0.05, self.read_sonars)  # 20 Hz timer

    def _export_pin(self, pin, direction):
        gpio_path = f"/sys/class/gpio/gpio{pin}"
        if not os.path.exists(gpio_path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(pin))
        with open(f"{gpio_path}/direction", "w") as f:
            f.write(direction)

    def _set_edge(self, pin, edge):
        """Set GPIO edge trigger mode (rising, falling, both)"""
        with open(f"/sys/class/gpio/gpio{pin}/edge", "w") as f:
            f.write(edge)

    def _write_gpio(self, pin, value):
        with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
            f.write(str(value))

    def _read_gpio(self, pin):
        with open(f"/sys/class/gpio/gpio{pin}/value", "r") as f:
            return int(f.read().strip())

    def _wait_for_edge(self, pin, timeout=0.01):
        """Wait for rising or falling edge using select.poll()"""
        fd = self.fd_map[pin]
        poll = select.poll()
        poll.register(fd, select.POLLPRI)
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            events = poll.poll(int(timeout * 1000))
            if events:
                fd.seek(0)
                return int(fd.read().strip())
        return -1

    def read_sonar(self, trig, echo):
        # Trigger the sonar
        self._write_gpio(trig, 1)
        time.sleep(0.00001)
        self._write_gpio(trig, 0)

        # Measure the echo pulse duration
        start_time = time.time()
        timeout = start_time + 0.01 # ~1.75м

        if self._wait_for_edge(echo) != 1:
            return -1
        start_time = time.time()

        # Wait for falling edge (echo end)
        if self._wait_for_edge(echo) != 0:
            return -1
        stop_time = time.time()

        # Calculate distance (in meters)
        time_elapsed = stop_time - start_time
        return (time_elapsed * 34300) / 2  # Speed of sound = 34300 cm/s

    def read_sonars(self):
        for i, (trig, echo) in enumerate(zip(self.trig_pins, self.echo_pins)):
            distance = self.read_sonar(trig, echo)

            if distance > 0:
                # Add the new value to the buffer
                self.buffers[self.names[i]].append(distance)

                # Apply median filter
                filtered_distance = statistics.median(self.buffers[self.names[i]])
                #filtered_distance = sum(self.buffers[self.names[i]]) / len(self.buffers[self.names[i]])
                # TODO kalman https://chat.deepseek.com/a/chat/s/6e3ba245-f131-4c8f-bf0d-40c05bad8460
                # https://alexgyver.ru/lessons/filters/

                # Publish the filtered data
                msg = Sonar()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.names[i]
                msg.range = filtered_distance / 100.0  # Convert cm to meters
                msg.azimuth = self.azimuths[i]

                self.publisher_.publish(msg)

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
