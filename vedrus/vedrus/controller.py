import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from simple_pid import PID
from vedrus_interfaces.msg import MotorMove, MotorPID, Safety, KeepAlive
import time

DEBUG = True

# Режим "Поворот к персоне" (с движением вперёд или без)
class ModeRotateToPerson():
	ROTATE_SPEED_MAX = 5.

	# Азимут последней замеченной персоны
	azimuth = 0.

	# simple_pid.PID instance
	pid = None

	# время последнего safety::alarm
	lastAlarmTime = None

	# последовательно пойманные safety::alarm
	alarmCount = 0

	# стартовал ли режим? (движется ли сейчас?)
	started = False

	def __init__(self, forward=False):
		self.forward = forward

	# TBD DRY какой-то base классссс
	# поймал safety.warning
	def warning(self, node, msg):
		# TBD две персоны с разными азимутами
		if msg.header.frame_id == 'yolo:person':
			self.azimuth = msg.azimuth

			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: got warning with person. New azimuth='+ str(self.azimuth))

	# поймал safety.alarm
	def alarm(self, node, msg):
		self.lastAlarmTime = time.time()
		self.alarmCount += 1

		if DEBUG:
			node.get_logger().info('ModeRotateToPerson: got alarm count='+ str(self.alarmCount))
		return

	def cycle(self, node):
		# вход в режим. Начало поворотов
		if not self.started:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: start to move')

			self.started = True
			self.pid = PID(5.0, 0.1, 0.05, setpoint=0.0)
			#self.pid = PID(5, 2, 0.025, setpoint=0.0)
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
			route.out = ModeRotateToPerson(forward=False)

			return route

		# -1 .. +1
		rotateAzimuth = (self.azimuth - 360. if self.azimuth > 180. else self.azimuth) / 180.
		if DEBUG:
			node.get_logger().info('ModeRotateToPerson: rotateAzimuth='+ str(rotateAzimuth))

		rotateSpeed = self.pid(rotateAzimuth)

		if DEBUG:
			node.get_logger().info('ModeRotateToPerson: rotateSpeed='+ str(rotateSpeed))

		node.left(-self.ROTATE_SPEED_MAX * rotateSpeed)
		node.right(self.ROTATE_SPEED_MAX * rotateSpeed)

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
	def warning(self, node, msg):
		pass

	# поймал safety.alarm
	def alarm(self, node, msg):
		self.lastAlarmTime = time.time()
		self.alarmCount += 1

		if DEBUG:
			node.get_logger().info('ModeForwardSlow: got alarm count='+ str(self.alarmCount))
		return

	def cycle(self, node):
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
class ModeSafetyStop():
	# время последнего safety::alarm
	lastAlarmTime = None

	# выход из режима в другой режим
	out = None

	def __init__(self):
		self.lastAlarmTime = time.time()

	# поймал safety.warning
	def warning(self, node, msg):
		pass

	# поймал safety.alarm
	def alarm(self, node, msg):
		if DEBUG:
			node.get_logger().info('ModeSafetyStop: got alarm')

		self.lastAlarmTime = time.time()
		return

	def cycle(self, node):
		# выход из стопа в self.out после 2 секунд без safety::alarm
		if time.time() - self.lastAlarmTime > 2:
			if DEBUG:
				node.get_logger().info('ModeSafetyStop: alarm 2 seconds ago. Route to out')
			return self.out

		#if DEBUG:
		#	node.get_logger().info('ModeSafetyStop: continue stay...')

		# ... или продолжить жить в этом режиме
		return self


class VedrusControlerNode(Node):
	crash = False

	# текущий режим
	mode = None

	publisherLeft = None
	publisherRight = None

	# время последнего keepalive Safety
	keepalive = None


	def left(self, speed, breaking=False):
		if DEBUG:
			self.get_logger().info('VedrusControlerNode::left: set speed to '+ str(speed))

		msg = MotorPID()

		msg.speed = speed
		msg.breaking = breaking

		self.publisherLeft.publish(msg)
		return

	def right(self, speed, breaking=False):
		if DEBUG:
			self.get_logger().info('VedrusControlerNode::right: set speed to '+ str(speed))

		msg = MotorPID()

		msg.speed = speed
		msg.breaking = breaking

		self.publisherRight.publish(msg)
		return

	def __init__(self):
		super().__init__('vedrus_control_node')

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
		self.keepalive = time.time() + 10 # 10 sec keepalive start gap

		# в таймере выполняю задачу
		#self.create_timer(0.2, self.task_safety_forward)
		self.create_timer(0.2, self.task_stay_and_rotate_to_person)

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
				self.mode.warning(self, msg)
		if msg.alarm:
			if self.mode is not None:
				self.mode.alarm(self, msg)


	# task
	def task_safety_forward(self):
		# Жду запуска Safety
		if self.keepalive is None:
			return

		# начальный режим и его параметры (здесь: роутинг)
		if self.mode is None:
			self.mode = ModeSafetyStop()
			self.mode.out = ModeForwardSlow()

		# выполню цикл режима. И тот решит какой режим следующий
		self.mode = self.mode.cycle(self)

	def task_stay_and_rotate_to_person(self):
		# Жду запуска Safety
		if self.keepalive is None:
			return

		# начальный режим и его параметры (здесь: роутинг)
		if self.mode is None:
			self.mode = ModeSafetyStop()
			self.mode.out = ModeRotateToPerson(forward=False)

		# выполню цикл режима. И тот решит какой режим следующий
		self.mode = self.mode.cycle(self)


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

		# проверю safety keepalive
		if time.time() - self.keepalive >= 2:
			self.crash = True

			self.get_logger().info('Got safety keepalive timeout!')

			msg = MotorMove()
			msg.crash = True
			# TBD надо ли все поля перечислить?

			publisher = self.create_publisher(MotorMove, '/vedrus/left/move', 10)
			publisher.publish(msg)

			publisher = self.create_publisher(MotorMove, '/vedrus/right/move', 10)
			publisher.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = VedrusControlerNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
