import csv
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from simple_pid import PID
from vedrus_interfaces.msg import MotorMove, MotorPID, Safety, KeepAlive, Status, StatusItem
import time

DEBUG = True
CSV = True
RGBD_FOV = 80.

class ModeParent():
	def _statusInit(self):
		self.status = Status()
		self.status.header.stamp = node.get_clock().now().to_msg()
		self.status.header.frame_id = 'controller'

	def _statusAppend(self, name, value):
		statusItem = StatusItem()
		statusItem.name = name
		statusItem.value = value
		self.status.items.append(statusItem)

	def _statusPublish(self):
		node.publisherStatus.publish(self.status)


# Режим "Поворот к персоне" (с движением вперёд или без)
class ModeRotateToPerson(ModeParent):
	ROTATE_SPEED_MAX = 3.

	# Азимут последней замеченной персоны (и время)
	azimuth = None
	persons = []
	lastPersonTime = None

	# simple_pid.PID instance
	pid = None

	# время последнего safety::alarm
	lastAlarmTime = None

	# последовательно пойманные safety::alarm
	alarmCount = 0

	# стартовал ли режим? (движется ли сейчас?)
	started = False

	# Данные глубин / азимутов за последнюю секунду (две?)
	depths = []

	# TBD move to parent
	def __init__(self):
		if CSV:
			self.csv = csv.writer(open('/opt/ros/iron/log/csv/controller/mode/rotate_to_person.csv', mode='w', newline=''))

			self.csv.writerow([
				'time', 'rotateAzimuth', 'rotateSpeed', 'move', 'left', 'right'
			])

	# поймал safety.warning
	def warning(self, msg):
		second = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

		# TBD две персоны с разными азимутами (ex.: > 10°)
		# TBD N персон с близкими азимутами (ex.: 10°)
		if msg.header.frame_id == 'yolo:person':
			self.azimuth = msg.azimuth
			self.lastPersonTime = time.time()

			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: got person: {:.4f}°'.format(self.azimuth))

			'''
			Надо крепко подумать о том, что считать персоны можно только без движения бота. Или делать это по одному timeframe
			
			self.persons.append({
				'time': second,
				'azimuth': msg.azimuth,
			})

			# Удаляю не актуальные в окне 1с
			self.persons = [item for item in self.persons if item['time'] >= (second - 1)]

			Есть набор азимутов. 
			Пример: [15]
				-> он один, его и берём
			Пример: [15,17]
				-> база = последний (msg.azimuth)
				-> разница < gap, берём последний
			if len(self.persons) == 1:
				self.azimuth = msg.azimuth
				self.lastPersonTime = time.time()

				if DEBUG:
					node.get_logger().info('ModeRotateToPerson: got person: {:.4f}°'.format(self.azimuth))

			one = True
			gap = 10 # 15 градусов щель
			for item in self.persons:
				if distance(item['azimuth'], msg.azimuth) > gap
			'''

		# Собираю данные RGBD для дальнейшего поиска расстояния до персоны
		if msg.header.frame_id == 'rgbd':
			self.depths.append({
				'time': second,
				'azimuth': msg.azimuth - 360. if msg.azimuth > 180. else msg.azimuth, # -180 ... +180
				'range': msg.range,
			})

			# Удаляю не актуальные в окне 1с
			self.depths = [item for item in self.depths if item['time'] >= (second - 1)]

	# поймал safety.alarm
	def alarm(self, msg):
		self.lastAlarmTime = time.time()
		self.alarmCount += 1

		if DEBUG:
			node.get_logger().info('ModeRotateToPerson: got alarm count='+ str(self.alarmCount))

	# По собранному в self.warning данныым self.depths ищу, были ли предупреждения (объект ближе метра) примерно по текущему азимуту
	def __isPersonNearAtAzimuth(self):
		if self.azimuth is None or len(self.depths) == 0:
			return False

		azimuth180 = self.azimuth - 360. if self.azimuth > 180. else self.azimuth # -180 ... +180
		gap = 5. # +-5° допуск

		azimuthDepths = [item for item in self.depths if (azimuth180 - gap) < item['azimuth'] < (azimuth180 + gap)]

		return len(azimuthDepths) > 0

	def cycle(self):
		self._statusInit()
		self._statusAppend('mode', 'RotateToPerson')
		self._statusAppend('forward', 'Y' if self.forward else 'N')

		# вход в режим. Начало поворотов
		if not self.started:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: start to rotate/move to person')

			self.started = True
			self.pid = PID(3.0, 0.1, 0.05, setpoint=0.0)
			self.pid.sample_time = 0.2
			self.pid.output_limits = (-1.0, 1.0)

		# случайная одиночная тревога за 0.5с. Игнорю
		if self.alarmCount == 1 and time.time() - self.lastAlarmTime >= 0.5:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: drop single alarm')

			self.alarmCount = 0

		# много тревоги. Остановка!
		if self.alarmCount > 1:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: got many alarms! Will stop')

			node.left(0.0, breaking=True)
			node.right(0.0, breaking=True)

			route = ModeSafetyStop()
			route.out = ModeRotateToPerson()
			route.out.forward = self.forward

			self._statusAppend('action', 'Stop by many alarms')
			self._statusPublish()

			return route

		# Персона есть? А то сидим и ждём
		if self.azimuth is None:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: no person.')

			self._statusAppend('action', 'No person')
			self._statusPublish()

			return self

		# Персоны не было 3 секунды. ПОТЕРЯЛ!!! :)
		if (time.time() - self.lastPersonTime) > 3:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: No new person 3 sec.')

			# Стоп машина! :)
			node.left(0.)  # без тормоза
			node.right(0.) # плавно скорость в ноль

			self.azimuth = None

			self._statusAppend('action', 'Lost person')
			self._statusPublish()

			return self

		# -1 .. +1
		rotateAzimuth = (self.azimuth - 360. if self.azimuth > 180. else self.azimuth) / 180.
		#rotateSpeed = self.pid(rotateAzimuth)
		rotateSpeed = rotateAzimuth

		#self._statusAppend('rotateAzimuth', '{:.2f}'.format(rotateAzimuth))
		self._statusAppend('rotateSpeed', '{:.2f}'.format(rotateSpeed))

		# если задан флаг "двигаться вперёд"
		# если азимут в FOV передней RGBD камеры
		# если расстояние до персоны > 1м
		if self.forward and not (RGBD_FOV/2. < self.azimuth < (360.-RGBD_FOV/2.)) and not self.__isPersonNearAtAzimuth():
			move = True

			left = self.ROTATE_SPEED_MAX - (rotateSpeed * 2 * self.ROTATE_SPEED_MAX)
			right = self.ROTATE_SPEED_MAX + (rotateSpeed * 2* self.ROTATE_SPEED_MAX)
		else:
			move = False

			left = self.ROTATE_SPEED_MAX * rotateSpeed * 2 
			right = -self.ROTATE_SPEED_MAX * rotateSpeed * 2

		self._statusAppend('move', 'Y' if move else 'N')

		node.left(min(left, self.ROTATE_SPEED_MAX))
		node.right(min(right, self.ROTATE_SPEED_MAX))

		if CSV:
			stamp = node.get_clock().now().to_msg()
			stampSecond = stamp.sec + stamp.nanosec / 1e9

			self.csv.writerow([
				stampSecond, rotateAzimuth, rotateSpeed, move, left, right
			])

		if DEBUG:
			print('{:12.2f} {:12.2f} {:12.2f}      {:b} {:12.4f} {:12.4f}'.format(stampSecond, rotateAzimuth, rotateSpeed, move, left, right))

		self._statusPublish()

		# ... или продолжить жить в этом режиме
		return self

# Режим "Медленное движение вперёд"
class ModeForwardSlow():
	# время последнего safety::alarm
	lastAlarmTime = None

	# последовательно пойманные safety::alarm
	alarmCount = 0

	# стартовал ли режим? (движется ли сейчас?)
	started = False

	# поймал safety.warning
	def warning(self, msg):
		pass

	# поймал safety.alarm
	def alarm(self, msg):
		self.lastAlarmTime = time.time()
		self.alarmCount += 1

		if DEBUG:
			node.get_logger().info('ModeForwardSlow: got alarm count='+ str(self.alarmCount))
		return

	def cycle(self):
		# вход в режим. Начало движения
		if not self.started:
			if DEBUG:
				node.get_logger().info('ModeForwardSlow: start to move')

			self.started = True
			node.left(5.)
			node.right(5.)

		# случайная одиночная тревога за 0.5с. Игнорю
		if self.alarmCount == 1 and time.time() - self.lastAlarmTime >= 0.5:
			if DEBUG:
				node.get_logger().info('ModeForwardSlow: drop single alarm')

			self.alarmCount = 0

		# много тревоги. Остановка!
		if self.alarmCount > 1:
			if DEBUG:
				node.get_logger().info('ModeForwardSlow: got many alarms! Will stop')

			node.left(0., breaking=True)
			node.right(0., breaking=True)

			route = ModeSafetyStop()
			route.out = ModeForwardSlow()

			return route

		#if DEBUG:
		#	node.get_logger().info('ModeForwardSlow: continue moving...')

		# ... или продолжить жить в этом режиме
		return self

# Режим "Останов перед препятствием"
class ModeSafetyStop(ModeParent):
	# время последнего safety::alarm
	lastAlarmTime = None

	# выход из режима в другой режим
	out = None

	def __init__(self):
		self.lastAlarmTime = time.time()

	# поймал safety.warning
	def warning(self, msg):
		pass

	# поймал safety.alarm
	def alarm(self, msg):
		if DEBUG:
			node.get_logger().info('ModeSafetyStop: got alarm')

		self.lastAlarmTime = time.time()
		return

	def cycle(self):
		self._statusInit()
		self._statusAppend('mode', 'SafetyStop')

		# выход из стопа в self.out после 2 секунд без safety::alarm
		if time.time() - self.lastAlarmTime > 2:
			if DEBUG:
				node.get_logger().info('ModeSafetyStop: alarm 2 seconds ago. Route to out')

			self._statusAppend('action', 'Route out')
			self._statusPublish()

			return self.out

		#if DEBUG:
		#	node.get_logger().info('ModeSafetyStop: continue stay...')

		self._statusAppend('action', 'Waiting')
		self._statusPublish()

		# ... или продолжить жить в этом режиме
		return self


class VedrusControlerNode(Node):
	crash = False

	# текущий режим
	mode = None

	# Управление моторами
	publisherLeft = None
	publisherRight = None

	# время последнего keepalive Safety
	keepalive = None


	def left(self, speed, breaking=False):
		#if DEBUG:
		#	self.get_logger().info('VedrusControlerNode::left: set speed to '+ str(speed))

		msg = MotorPID()

		msg.speed = speed
		msg.breaking = breaking

		self.publisherLeft.publish(msg)
		return

	def right(self, speed, breaking=False):
		#if DEBUG:
		#	self.get_logger().info('VedrusControlerNode::right: set speed to '+ str(speed))

		msg = MotorPID()

		msg.speed = speed
		msg.breaking = breaking

		self.publisherRight.publish(msg)
		return

	def __init__(self):
		super().__init__('vedrus_control_node')

		self.publisherStatus = self.create_publisher(Status, 'vedrus/status', 10)
		self.publisherLeft = self.create_publisher(MotorPID, 'vedrus/left/pid', 10)
		self.publisherRight = self.create_publisher(MotorPID, 'vedrus/right/pid', 10)

		# keepalive
		self.publisher_keepalive = self.create_publisher(KeepAlive, '/vedrus/keepalive/controller', 10)
		self.create_timer(0.33333333, self.timer_keepalive)
		self.create_subscription(
			KeepAlive,
			'/vedrus/keepalive/safety',
			self.safety_keepalive_callback,
			10)

		# в таймере выполняю задачу
		#self.create_timer(0.2, self.task_safety_forward)
		#self.create_timer(0.2, self.task_stay_and_rotate_to_person)
		self.create_timer(0.2, self.task_find_person_rotate_and_follow)

		self.create_subscription(
			Safety,
			'/vedrus/safety',
			self.safety_callback,
			10)

		if DEBUG:
			self.get_logger().info('VedrusControlerNode is started')


	# safety
	def safety_callback(self, msg):
		if msg.warning:
			if self.mode is not None:
				self.mode.warning(msg)
		if msg.alarm:
			if self.mode is not None:
				self.mode.alarm(msg)


	# task
	def task_safety_forward(self):
		# Жду запуска Safety
		if self.keepalive is None:
			if DEBUG:
				self.get_logger().info('VedrusControlerNode: waiting for Safety')
			return

		# начальный режим и его параметры (здесь: роутинг)
		if self.mode is None:
			self.mode = ModeSafetyStop()
			self.mode.out = ModeForwardSlow()

		# выполню цикл режима. И тот решит какой режим следующий
		self.mode = self.mode.cycle()

	def task_stay_and_rotate_to_person(self):
		# Жду запуска Safety
		if self.keepalive is None:
			if DEBUG:
				self.get_logger().info('VedrusControlerNode: waiting for Safety')
			return

		# начальный режим и его параметры (здесь: роутинг)
		if self.mode is None:
			self.mode = ModeSafetyStop()
			self.mode.out = ModeRotateToPerson()
			self.mode.out.forward = False

		# выполню цикл режима. И тот решит какой режим следующий
		self.mode = self.mode.cycle()

	def task_find_person_rotate_and_follow(self):
		# Жду запуска Safety
		if self.keepalive is None:
			if DEBUG:
				self.get_logger().info('VedrusControlerNode: waiting for Safety')
			return

		# начальный режим и его параметры (здесь: роутинг)
		if self.mode is None:
			self.mode = ModeSafetyStop()
			self.mode.out = ModeRotateToPerson()
			self.mode.out.forward = True

		# выполню цикл режима. И тот решит какой режим следующий
		self.mode = self.mode.cycle()


	# keepalive
	def safety_keepalive_callback(self, data):
		self.keepalive = time.time()

	def timer_keepalive(self):
		if self.crash:
			return

		# отправлю свой keepalive
		msg = KeepAlive()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'controller'
		self.publisher_keepalive.publish(msg)

		# Жду запуска Safety
		if self.keepalive is None:
			return

		# проверю safety keepalive
		if time.time() - self.keepalive >= 2:
			self.crash = True

			self.get_logger().info('Got safety keepalive timeout!')

			msg = MotorMove()
			msg.crash = True

			publisher = self.create_publisher(MotorMove, '/vedrus/left/move', 10)
			publisher.publish(msg)

			publisher = self.create_publisher(MotorMove, '/vedrus/right/move', 10)
			publisher.publish(msg)


def main(args=None):
	global node

	rclpy.init(args=args)
	node = VedrusControlerNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
