'''
BluePill ROS2 Node

This script implements a ROS2 node for controlling and monitoring a motor wheel connected to an STM32 controller
via UART. The node communicates with the motor controller by sending and receiving specific commands.

### Features:
1. Sends motor control commands (start, stop, set speed, configure PID parameters).
2. Publishes motor status to the topic `/vedrus/motor/status`.
3. Subscribes to commands from the topic `/vedrus/motor/command`.

### Parameters:
- `name` (string, default: "left"): Name of the motor (e.g., "left").
- `port` (string, default: "/dev/ttyS3"): UART port connected to the STM32 controller.
- `P` (float, default: -1): Proportional gain for the PID controller (negative value means not set).
- `I` (float, default: -1): Integral gain for the PID controller.
- `D` (float, default: -1): Derivative gain for the PID controller.

### Subscribed Topics:
- `/vedrus/motor/command` (vedrus_interface/MotorCommand):
  - Accepts a command to control the motor speed.
  - Command structure: `std_msgs/Header header, int16 speed`.

### Published Topics:
- `/vedrus/motor/status` (vedrus_interface/MotorStatus):
  - Publishes the motor's real-time status.
  - Status structure: `std_msgs/Header header, int32 position, float32 speed, int16 target, float32 power`.

### Usage:
#### Run the node:
ros2 run your_package_name bluepill --ros-args -p P:=0.006 -p I:=0.002

Example Launch File:
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='bluepill',
            name='bluepill_left',
            parameters=[
                {'name': 'left'},
                {'port': '/dev/ttyS2'},
                {'P': 0.01},
                {'I': 0.02},
                {'D': 0.005}
            ]
        )
    ])
'''

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Header
from vedrus_interfaces.msg import MotorCommand, MotorStatus
from threading import Thread, Lock

class BluePillNode(Node):
    def __init__(self):
        super().__init__('bluepill')

        # Declare and read parameters
        self.declare_parameter('name', 'left')
        self.declare_parameter('port', '/dev/ttyS2')
        self.declare_parameter('P', -1.)
        self.declare_parameter('I', -1.)
        self.declare_parameter('D', -1.)

        self.motor_name = self.get_parameter('name').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.P = self.get_parameter('P').get_parameter_value().double_value
        self.I = self.get_parameter('I').get_parameter_value().double_value
        self.D = self.get_parameter('D').get_parameter_value().double_value

        self.serial_lock = Lock()

        # Open UART connection
        try:
            self.serial = serial.Serial(self.port, baudrate=115200, timeout=1)
            self.get_logger().info(f"Connected to {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {self.port}: {e}")
            raise SystemExit

        # Send initial PID parameters if set
        if self.P != -1.:
            self.send_command(f"P{self.P:.3f}")
        if self.I != -1.:
            self.send_command(f"I{self.I:.3f}")
        if self.D != -1.:
            self.send_command(f"D{self.D:.3f}")

        # Publishers and subscribers
        self.publisher = self.create_publisher(MotorStatus, '/vedrus/motor/status', 10)
        self.subscription = self.create_subscription(
            MotorCommand,
            '/vedrus/motor/command',
            self.command_callback,
            10
        )

        # Thread for receiving motor status
        self.running = True
        self.read_thread = Thread(target=self.read_motor_status)
        self.read_thread.start()

    def send_command(self, command: str):
        try:
            with self.serial_lock:
                self.serial.write(f"{command}\r\n".encode())
                self.get_logger().info(f"Sent command: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def command_callback(self, msg: MotorCommand):
        if msg.header.frame_id == self.motor_name:
            speed_command = f"M{msg.speed}"
            self.send_command(speed_command)

    def read_motor_status(self):
        while self.running:
            try:
                with self.serial_lock:
                    line = self.serial.readline().decode().strip()

                if line.startswith("VED"):
                    status = self.parse_motor_status(line)
                    if status:
                        self.publisher.publish(status)
            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")

    def parse_motor_status(self, line: str) -> MotorStatus:
        try:
            data = {}
            for item in line[5:].split(','):
                key, value = item.split(':')
                data[key] = value

            msg = MotorStatus()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.motor_name
            msg.position = int(data['POS'])
            msg.speed = float(data['SPD'])
            msg.target = float(data['TGT'])
            msg.power = float(data['PWR'])
            return msg

        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Failed to parse motor status: {e}")
            return None

    def destroy_node(self):
        self.running = False
        self.read_thread.join()
        self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BluePillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
