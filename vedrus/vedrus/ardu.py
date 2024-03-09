'''
TODO:
- move to C++?
'''
from collections import deque
import csv
from influxdb import InfluxDBClient
import rclpy
from rclpy.node import Node
import serial
from simple_pid import PID
import sys
import time
import threading

from std_msgs.msg import UInt8, UInt16, Float32, String
from sensor_msgs.msg import Temperature, RelativeHumidity
from vedrus_interfaces.msg import Motor, MotorMove, MotorPID, Sonar

DEBUG = False
CSV = True

class VedrusArduNode(Node):
	side = None # left | right side sign
	start = None # VEDL | VEDR line start sign
	line = None # string[] line, readed from serial

	pid1 = None # simple_pid.PID instance
	tickIncrements1 = deque([0, 0, 0, 0, 0], maxlen=5) # actual tick increments by 0.2s cycle
	tickLast1 = None
	powerLast1 = 0

	pid2 = None # simple_pid.PID instance
	tickIncrements2 = deque([0, 0, 0, 0, 0], maxlen=5) # actual tick increments by 0.2s cycle
	tickLast2 = None
	powerLast2 = 0

	# получение прямой команды к моторам через /vedrus/side/motor::MotorMove
	# эта команда будет тут же выполнена
	def motor_move(self, msg):
		bytes = [ord('c')]
		bytes.append(ord('b') if msg.breaking else ord('m'))

		if self.side == 'left':
			bytes.append(ord('f') if msg.forward else ord('r'))
		else:
			bytes.append(ord('r') if msg.forward else ord('f'))

		bytes.append(int(msg.power1))
		bytes.append(int(msg.power2))

		self.ser.write(bytes)

	# получение нового ticks per second через /vedrus/side/pid::MotorPID.
	# к нему моторы будут стремиться с помощью ПИД-регулятора
	def motor_pid(self, msg):
		'''
		Поступает сообзщение MotorPID:
		- bool forward
		- int16 speed
		'''
		self.pid1.setpoint = msg.speed
		self.pid2.setpoint = msg.speed
		print(msg)

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

		#https://drives.ru/stati/nastrojka-pid-regulyatora/
		#rear
		self.pid1 = PID(1, 1, 0.025, setpoint=0)
		self.pid1.sample_time = 0.2
		self.pid1.output_limits = (-1, 25) # 255

		#front
		self.pid2 = PID(1, 1, 0.025, setpoint=0)
		self.pid2.sample_time = 0.2
		self.pid2.output_limits = (-1, 25) # 255

		self.timer = self.create_timer(0.2, self.timer_publish)

		self.get_logger().info('Node initialized')

	def timer_publish(self):
		if self.line is None:
			return

		div = self.line.split(',')
		self.line = None

		if len(div) == 15 and (div[0] == 'VEDR' or div[0] == 'VEDL') and div[ len(div) - 1 ] == 'END':
			if self.side is None:
				self.side = 'left' if div[0] == 'VEDL' else 'right'
				self.start = div[0]

				self.create_subscription(
					MotorMove,
					'vedrus/'+ self.side +'/move',
					self.motor_move,
					10)

				self.create_subscription(
					MotorPID,
					'vedrus/'+ self.side +'/pid',
					self.motor_pid,
					10)

				self.publisher_temperature = self.create_publisher(Temperature, 'vedrus/'+ self.side +'/temperature', 10)
				self.publisher_humidity = self.create_publisher(RelativeHumidity, 'vedrus/'+ self.side +'/humidity', 10)

				self.publisher_sonar = self.create_publisher(Sonar, 'vedrus/sonar', 10)

				self.publisher_motor = self.create_publisher(Motor, 'vedrus/motor', 10)

				self.publisher_raw = self.create_publisher(String, 'vedrus/'+ self.side +'/raw', 10)

				if CSV:
					self.csv_motor_front = csv.writer(open('/opt/ros/iron/log/csv/motor/'+ self.side +'/front.csv', mode='w', newline=''))
					self.csv_motor_rear = csv.writer(open('/opt/ros/iron/log/csv/motor/'+ self.side +'/rear.csv', mode='w', newline=''))
					self.csv_sonar_side = csv.writer(open('/opt/ros/iron/log/csv/sonar/'+ self.side +'/side.csv', mode='w', newline=''))
					self.csv_sonar_rear = csv.writer(open('/opt/ros/iron/log/csv/sonar/'+ self.side +'/rear.csv', mode='w', newline=''))
					self.csv_motor_front.writerow([
						'time', 'tick', 'current', 'power', 'forward'
					])
					self.csv_motor_rear.writerow([
						'time', 'tick', 'current', 'power', 'forward'
					])
					self.csv_sonar_side.writerow([
						'time', 'range', 'azimuth'
					])
					self.csv_sonar_rear.writerow([
						'time', 'range', 'azimuth'
					])


		if self.side is not None and len(div) == 15 and div[0] == self.start and div[ len(div) - 1 ] == 'END':

			try:
				#[0'VEDR', 1'196.88', 2'210.56', 3'0', 4'0', 5'2882', 6'-1', 7'0', 8'0', 9'0', 10'205.50', 11'0', 12'0', 13'0', 'END']
				stamp = self.get_clock().now().to_msg()
				stampSecond = stamp.sec + stamp.nanosec / 1e9

				#msg = Temperature()
				#msg.temperature = float(div[7])
				#msg.variance = 0.
				#self.publisher_temperature.publish(msg)

				#msg = RelativeHumidity()
				#msg.relative_humidity = float(div[8])
				#msg.variance = 0.
				#self.publisher_humidity.publish(msg)

				msg = Sonar()
				msg.header.frame_id = self.side +'_side'
				msg.header.stamp = stamp
				msg.range = 300. if div[5] == '-1' else float(div[5]) / 57.
				msg.azimuth = 90. if self.side == 'right' else 270.
				self.publisher_sonar.publish(msg)

				if CSV:
					self.csv_sonar_side.writerow([
						stampSecond, msg.range, msg.azimuth
					])

				msg = Sonar()
				msg.header.frame_id = self.side +'_rear'
				msg.header.stamp = stamp
				msg.range = 300. if div[6] == '-1' else float(div[6]) / 57.
				msg.azimuth = 180.
				self.publisher_sonar.publish(msg)

				if CSV:
					self.csv_sonar_rear.writerow([
						stampSecond, msg.range, msg.azimuth
					])

				msg = Motor()
				msg.header.frame_id = self.side +'_rear'
				msg.header.stamp = stamp
				msg.tick = int(div[3])
				msg.current = float(div[1])
				msg.power = int(div[11])
				msg.forward = div[13] == '1'
				self.publisher_motor.publish(msg)

				if CSV:
					self.csv_motor_rear.writerow([
						stampSecond, msg.tick, msg.current, msg.power, msg.forward
					])

				if self.tickLast1 is None:
					self.tickLast1 = msg.tick

				tickIncrement1 = msg.tick - self.tickLast1
				if tickIncrement1 < 0:
					tickIncrement1 += 65536

				self.tickIncrements1.append(tickIncrement1)
				self.tickLast1 = msg.tick

				if DEBUG:
					print('+'+ str(tickIncrement1) +'. sum='+ str(sum(self.tickIncrements1)) +'. setpoint='+ str(self.pid1.setpoint))

				#powerToSet1 = self.pid1(sum(self.tickIncrements1))
				powerToSet1 = self.pid1(tickIncrement1)

				if self.pid1.setpoint == 0 and powerToSet1 < 4:
					powerToSet1 = 0

				msg = Motor()
				msg.header.frame_id = self.side +'_front'
				msg.header.stamp = stamp
				msg.tick = int(div[4])
				msg.current = float(div[2])
				msg.power = int(div[12])
				msg.forward = div[13] == '1'
				self.publisher_motor.publish(msg)

				if CSV:
					self.csv_motor_front.writerow([
						stampSecond, msg.tick, msg.current, msg.power, msg.forward
					])

				if self.tickLast2 is None:
					self.tickLast2 = msg.tick

				tickIncrement2 = msg.tick - self.tickLast2
				if tickIncrement2 < 0:
					tickIncrement2 += 65536

				self.tickIncrements2.append(tickIncrement2)
				self.tickLast2 = msg.tick

				if DEBUG:
					print('+'+ str(tickIncrement2) +'. sum='+ str(sum(self.tickIncrements2)) +'. setpoint='+ str(self.pid2.setpoint))

				#powerToSet2 = self.pid2(sum(self.tickIncrements2))
				powerToSet2 = self.pid2(tickIncrement2)

				if self.pid2.setpoint == 0 and powerToSet2 < 4:
					powerToSet2 = 0

				# TBD
				# если зависает в pwm > 0 без движения, то контроллер вырубается, пока ему не подать pwm = 0. Хоть 255 шли, будет стоять на месте, пока 0 не подашь. А потом работает после 0
				# типа защита от заблокированного колеса
				#if powerToSet >= 25 and sum(self.tickIncrements) == 0:
					# send power = 0

				if powerToSet1 != self.powerLast1 or powerToSet2 != self.powerLast2:
					if DEBUG:
						print('                             ', powerToSet1, powerToSet2)

					self.powerLast1 = powerToSet1
					self.powerLast2 = powerToSet2

					msg = MotorMove()
					msg.breaking = (powerToSet1 < 0) or (powerToSet2 < 0)
					msg.forward = True
					msg.power1 = round(powerToSet1) if powerToSet1 >= 0 else 0
					msg.power2 = round(powerToSet2) if powerToSet2 >= 0 else 0
					self.motor_move(msg)

			except ValueError as ex:
				print(ex)
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
