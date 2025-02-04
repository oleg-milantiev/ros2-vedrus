"""
This script defines a ROS 2 node that publishes magnetometer readings from a QMC5883L sensor to a specified topic. The node supports configuring its behavior through ROS 2 parameters, allowing flexibility in deployment scenarios.

Features:
- Configurable I2C bus and magnetic declination via ROS 2 parameters.
- Continuous publishing of magnetometer data at a fixed rate (10 Hz).
- Utilizes a timer for efficient periodic task execution.
- Configurable sensor calibration matrix via ROS 2 parameters.

Requirements:
- py_qmc5883l

To install the necessary Python packages, use the following pip command:
pip install py-qmc5883l

Usage in a launch file to configure node parameters:
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'i2c_bus': 2,          # Default I2C bus for the sensor
        'declination': 7.1465, # Default declination in degrees
        'topic_name': '/imu/bearing'  # Default topic for IMU data
    }
    
    node = Node(
        package='your_package',
        executable='mag_node',
        name='mag_node',
        parameters=[params]
    )
    
    return LaunchDescription([node])
"""

import py_qmc5883l
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MagNode(Node):
	def __init__(self):
		super().__init__('mag_node')
		
		# Declare parameters with default values
		self.declare_parameter('i2c_bus', 2)
		self.declare_parameter('declination', 7.1465)
		self.declare_parameter('topic_name', '/imu/bearing')
		
		# Initialize sensor and publisher with parameters
		i2c_bus = self.get_parameter('i2c_bus').value
		declination = self.get_parameter('declination').value
		topic_name = self.get_parameter('topic_name').value
		self.publisher = self.create_publisher(Float32, topic_name, 10)
		self.sensor = py_qmc5883l.QMC5883L(i2c_bus=i2c_bus, output_data_rate=0b00000000)
		self.sensor.calibration = [[1.0313066562489555, -0.031047791919103184, 863.5276384091891], [-0.031047791919103188, 1.030791068052312, 1322.8836301489387], [0.0, 0.0, 1.0]]
		self.sensor.set_declination(declination)

		# Timer for publishing readings (same as before)
		self.timer = self.create_timer(0.1, self.publish_reading)

	def publish_reading(self):
		msg = Float32()
		msg.data = self.sensor.get_bearing()
		self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = MagNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()