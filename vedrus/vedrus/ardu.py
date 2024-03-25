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

MAX_POWER = 50 # 255
DEBUG = False
CSV = True

class VedrusArduNode(Node):
	crash = False

	side = None # left | right side sign
	start = None # VEDL | VEDR line start sign
	line = None # string[] line, readed from serial

	pid1 = None # simple_pid.PID instance
	tickLast1 = None
	powerLast1 = 0

	pid2 = None # simple_pid.PID instance
	tickLast2 = None
	powerLast2 = 0

	pidBreaking = False

	# получение прямой команды к моторам через /vedrus/side/motor::MotorMove
	# эта команда будет тут же выполнена
	def motor_move(self, msg):
		# crash ставится раз и больше не даёт двигать моторами
		if self.crash:
			return

		if msg.crash:
			self.get_logger().info(self.side +': Got crash signal!')

			self.crash = True

			bytes = [ord('c'), ord('b'), ord('f'), 0, 0]
			self.ser.write(bytes)
			return

		if DEBUG:
			self.get_logger().info(self.side +': Got MOVE: '+ ('+' if msg.forward else '-') + str(msg.power1) +':'+ ('+' if msg.forward else '-') + str(msg.power2))

		bytes = [ord('c')]
		bytes.append(ord('b') if msg.breaking else ord('m'))

		if self.side == 'left':
			bytes.append(ord('f') if msg.forward else ord('r'))
		else:
			bytes.append(ord('r') if msg.forward else ord('f'))

		bytes.append(int(msg.power1))
		bytes.append(int(msg.power2))

		self.ser.write(bytes)

	'''
	Получение нового ticks per frame через /vedrus/side/pid::MotorPID.
	К нему моторы будут стремиться с помощью ПИД-регуляторов
	Поступает сообзщение MotorPID:
	- bool forward
	- signed float32 speed: +вперёд и -назад
	'''
	def motor_pid(self, msg):
		self.pid1.setpoint = msg.speed
		self.pid2.setpoint = msg.speed
		if DEBUG:
			self.get_logger().info(self.side +':Got PID: '+ str(msg.speed))

		if msg.breaking:
			if DEBUG:
				self.get_logger().info(self.side +':Got PID breaking')

			self.pidBreaking = True
			self.powerLast1 = 0
			self.powerLast2 = 0

			msg = MotorMove()
			msg.breaking = True
			msg.forward = msg.forward
			msg.power1 = 0
			msg.power2 = 0
			#self.motor_move(msg)
			self.publisher_move.publish(msg)

			self.pidInit()

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

		self.pidInit()

		self.timer = self.create_timer(0.2, self.timer_publish)

		self.get_logger().info('Node initialized')

	def pidInit(self):
		self.pidBreaking = False

		#https://drives.ru/stati/nastrojka-pid-regulyatora/
		#rear
		self.pid1 = PID(2., 0.1, 0.05, setpoint=0)
		self.pid1.sample_time = 0.2
		self.pid1.output_limits = (-MAX_POWER, MAX_POWER)
		self.powerLast1 = 0

		#front
		#self.pid2 = PID(1, 0.1, 0.05, setpoint=0)
		self.pid2 = PID(2., 0.1, 0.05, setpoint=0)
		self.pid2.sample_time = 0.2
		self.pid2.output_limits = (-MAX_POWER, MAX_POWER)
		self.powerLast2 = 0

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
				self.publisher_move = self.create_publisher(MotorMove, 'vedrus/'+ self.side +'/move', 10)

				self.create_subscription(
					MotorPID,
					'vedrus/'+ self.side +'/pid',
					self.motor_pid,
					10)

				self.publisher_temperature = self.create_publisher(Temperature, 'vedrus/'+ self.side +'/temperature', 10)
				self.publisher_humidity = self.create_publisher(RelativeHumidity, 'vedrus/'+ self.side +'/humidity', 10)

				self.publisher_sonar = self.create_publisher(Sonar, 'vedrus/sonar', 10)

				self.publisher_motor = self.create_publisher(Motor, 'vedrus/motor', 10)

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
				msg.azimuth = 195. if self.side == 'left' else 165.
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

				self.tickLast1 = msg.tick

				if self.powerLast1 < 0:
					tickIncrement1 = -tickIncrement1

				if DEBUG:
					print('+'+ str(tickIncrement1) +'. setpoint='+ str(self.pid1.setpoint))

				powerToSet1 = self.pid1(tickIncrement1)

				if self.pid1.setpoint == 0 and abs(powerToSet1) < 4:
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

				self.tickLast2 = msg.tick

				if self.powerLast2 < 0:
					tickIncrement2 = -tickIncrement2

				if DEBUG:
					print('+'+ str(tickIncrement2) +'. setpoint='+ str(self.pid2.setpoint))

				powerToSet2 = self.pid2(tickIncrement2)

				if self.pid2.setpoint == 0 and abs(powerToSet2) < 4:
					powerToSet2 = 0

				# TBD
				# если зависает в pwm > 0 без движения, то контроллер вырубается, пока ему не подать pwm = 0. Хоть 255 шли, будет стоять на месте, пока 0 не подашь. А потом работает после 0
				# типа защита от заблокированного колеса
				#if powerToSet >= 25 and sum(self.tickIncrements) == 0:
					# send power = 0

				if not self.pidBreaking and (powerToSet1 != self.powerLast1 or powerToSet2 != self.powerLast2):
					if DEBUG:
						print('                             ', powerToSet1, powerToSet2)

					self.powerLast1 = powerToSet1
					self.powerLast2 = powerToSet2

					msg = MotorMove()
					msg.breaking = False
					msg.forward = (powerToSet1 > 0) or (powerToSet2 > 0) # TBD раздельный forward для моторов (в т.ч. в firmware). Заодно и раздельный breaking
					msg.power1 = round(abs(powerToSet1))
					msg.power2 = round(abs(powerToSet2))

					# TBD с целью оптимизации эта нода не шлёт сама себе сообщения MotorMove через ROS
					# Однако, это привело к невозможности отображения в пульте мощности моторов
					# Возможно, параллельно стоит слать статус моторов именно пульту
					#self.motor_move(msg)
					self.publisher_move.publish(msg)

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
