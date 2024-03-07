'''
TODO:
- move from timer to serial-async
- move to C++
- ? one big or few minor missages ?
'''
import sys
import rclpy
import serial
from rclpy.node import Node
import time
import threading

from std_msgs.msg import UInt8, UInt16, Float32, String
from sensor_msgs.msg import Temperature, RelativeHumidity
from vedrus_interfaces.msg import Motor, MotorMove, Sonar


class VedrusArduNode(Node):
	side = None
	start = None

	def move_motor(self, msg):
		bytes = [ord('c')]
		bytes.append(ord('b') if msg.breaking else ord('m'))

		if self.side == 'left':
			bytes.append(ord('f') if msg.forward else ord('r'))
		else:
			bytes.append(ord('r') if msg.forward else ord('f'))

		bytes.append(int(msg.power1))
		bytes.append(int(msg.power2))

		self.ser.write(bytes)

	def __init__(self):
		super().__init__('vedrus_ardu')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', rclpy.Parameter.Type.STRING),
			]
		)

		self.ser = serial.Serial(self.get_parameter('device').get_parameter_value().string_value, 115200, rtscts=True)

		self.serial_thread = threading.Thread(target=self.serial_reader)
		self.serial_thread.start()

		self.timer = self.create_timer(0.2, self.timer_publish)

		self.get_logger().info('Node initialized')

	line = None

	def timer_publish(self):
		if self.line is not None:
			div = self.line.split(',')
			self.line = None

			if len(div) == 15 and (div[0] == 'VEDR' or div[0] == 'VEDL') and div[ len(div) - 1 ] == 'END':
				if self.side is None:
					self.side = 'left' if div[0] == 'VEDL' else 'right'
					self.start = div[0]

					self.subscription = self.create_subscription(
						MotorMove,
						'vedrus/'+ self.side +'/move',
						self.move_motor,
						10)

					self.publisher_temperature = self.create_publisher(Temperature, 'vedrus/'+ self.side +'/temperature', 10)
					self.publisher_humidity = self.create_publisher(RelativeHumidity, 'vedrus/'+ self.side +'/humidity', 10)

					self.publisher_sonar = self.create_publisher(Sonar, 'vedrus/sonar', 10)

					self.publisher_motor = self.create_publisher(Motor, 'vedrus/motor', 10)

					self.publisher_raw = self.create_publisher(String, 'vedrus/'+ self.side +'/raw', 10)

			if self.side is not None and len(div) == 15 and div[0] == self.start and div[ len(div) - 1 ] == 'END':

				try:
					#[0'VEDR', 1'196.88', 2'210.56', 3'0', 4'0', 5'2882', 6'-1', 7'0', 8'0', 9'0', 10'205.50', 11'0', 12'0', 13'0', 'END']
					stamp = self.get_clock().now().to_msg()

					msg = Temperature()
					msg.temperature = float(div[7])
					msg.variance = 0.
					self.publisher_temperature.publish(msg)

					msg = RelativeHumidity()
					msg.relative_humidity = float(div[8])
					msg.variance = 0.
					self.publisher_humidity.publish(msg)

					msg = Sonar()
					msg.header.frame_id = self.side +'_side'
					msg.header.stamp = stamp
					msg.range = -1. if div[5] == '-1' else float(div[5]) / 57.
					msg.azimuth = 90. if self.side == 'right' else 270.
					self.publisher_sonar.publish(msg)

					msg = Sonar()
					msg.header.frame_id = self.side +'_rear'
					msg.header.stamp = stamp
					msg.range = -1. if div[6] == '-1' else float(div[6]) / 57.
					msg.azimuth = 180.
					self.publisher_sonar.publish(msg)

					msg = Motor()
					msg.header.frame_id = self.side +'_front'
					msg.header.stamp = stamp
					msg.tick = int(div[3])
					msg.current = float(div[1])
					msg.power = int(div[11])
					msg.forward = div[13] == '1'
					self.publisher_motor.publish(msg)

					msg = Motor()
					msg.header.frame_id = self.side +'_rear'
					msg.header.stamp = stamp
					msg.tick = int(div[4])
					msg.current = float(div[2])
					msg.power = int(div[12])
					msg.forward = div[13] == '1'
					self.publisher_motor.publish(msg)

				except ValueError:
					print('ValueError')
					pass

	def serial_reader(self):
		self.get_logger().info('Serial threading started')

		while rclpy.ok():
			try:
				self.line = self.ser.readline().decode('ascii').strip()

			except:
				pass

def main(args=None):
	rclpy.init(args=args)

	vedrus_ardu_node = VedrusArduNode()
	rclpy.spin(vedrus_ardu_node)

	vedrus_ardu_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
