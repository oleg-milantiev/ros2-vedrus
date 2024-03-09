#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from influxdb import InfluxDBClient
from vedrus_interfaces.msg import Motor, Sonar

class InfluxLogger(Node):

	def __init__(self):
		super().__init__('influx_logger')

		self.declare_parameters(
			namespace='',
			parameters=[
				('host', 'localhost'),
				('port', 8086),
				('user', 'r2'),
				('password', 'r2'),
				('database', 'r2'),
			]
		)

		self.client = InfluxDBClient(
			self.get_parameter('host').get_parameter_value().string_value,
			self.get_parameter('port').get_parameter_value().integer_value,
			self.get_parameter('user').get_parameter_value().string_value,
			self.get_parameter('password').get_parameter_value().string_value,
			self.get_parameter('database').get_parameter_value().string_value)

		self.create_subscription(Motor, '/vedrus/motor', self.motor_callback, 10)
		#self.create_subscription(Sonar, '/vedrus/sonar', self.sonar_callback, 10)

	def motor_callback(self, data):

		sensor_data = [
			{
				"measurement": "motor",
				"tags": {
					"location": data.header.frame_id
				},
				"fields": {
					"tick": data.tick,
					"current": data.current,
					"power": data.power,
					"forward": data.forward,
				}
			}
		]

		self.client.write_points(sensor_data)

	def sonar_callback(self, data):

		sensor_data = [
			{
				"measurement": "sonar",
				"tags": {
					"location": data.header.frame_id
				},
				"fields": {
					"range": data.range,
					"azimuth": data.azimuth,
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
