#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from influxdb import InfluxDBClient

class InfluxLogger(Node):

	def __init__(self):
		super().__init__('influx_logger')

		self.declare_parameters(
			namespace='',
			parameters=[
				('host', rclpy.Parameter.Type.STRING),
				('port', rclpy.Parameter.Type.INTEGER),
				('user', rclpy.Parameter.Type.STRING),
				('password', rclpy.Parameter.Type.STRING),
				('database', rclpy.Parameter.Type.STRING),
			]
		)

		self.client = InfluxDBClient(
			self.get_parameter('host').get_parameter_value().string_value,
			self.get_parameter('port').get_parameter_value().integer_value,
			self.get_parameter('user').get_parameter_value().string_value,
			self.get_parameter('password').get_parameter_value().string_value,
			self.get_parameter('database').get_parameter_value().string_value)

		self.create_subscription(
			Image,
			'/topic1',
			self.sub1,
			10)

	def sub1(self, data):

		sensor_data = [
			{
				"measurement": "temperature",
				"tags": {
					"location": "room"
				},
				"fields": {
					"value": 25.5
				}
			}
		]

		self.client.write_points(sensor_data)

def main(args=None):
	rclpy.init(args=args)

	influx_logger = InfluxLogger()
	rclpy.spin(influx_logger)

	influx_logger.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
