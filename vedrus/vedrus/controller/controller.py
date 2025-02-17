import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float32  # type: ignore
from sensor_msgs.msg import Imu  # type: ignore
from vedrus_interfaces.msg import MotorCommand, Safety, KeepAlive, Status, StatusItem
import time

from .modes.parent import ModeParent
from .modes.rotate_to_person import ModeRotateToPerson
from .modes.forward_slow import ModeForwardSlow
from .modes.safety_stop import ModeSafetyStop

DEBUG = True
CSV = False
RGBD_FOV = 92.2

class VedrusControlerNode(Node):
	crash = False

	# текущий режим
	mode = None

	# Управление моторами
	publisherMotor = None

	# время последнего keepalive Safety
	keepalive = None

	def left(self, speed):
		#if DEBUG:
		#	self.get_logger().info('VedrusControlerNode::left: set speed to '+ str(speed))

		msg = MotorCommand()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'left'
		msg.speed = speed
		self.publisherMotor.publish(msg)

		return

	def right(self, speed):
		#if DEBUG:
		#	self.get_logger().info('VedrusControlerNode::right: set speed to '+ str(speed))

		msg = MotorCommand()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'right'
		msg.speed = speed
		self.publisherMotor.publish(msg)

		return

	def __init__(self):
		super().__init__('vedrus_control_node')

		self.publisherStatus = self.create_publisher(Status, '/vedrus/status', 10)
		self.publisherMotor = self.create_publisher(MotorCommand, '/vedrus/motor/command', 10)

		# magnetic bearing
		self.create_subscription(Float32, '/imu/bearing', self.bearing_callback, 10)

		# keepalive
		self.publisher_keepalive = self.create_publisher(KeepAlive, '/vedrus/keepalive/controller', 10)
		self.create_timer(0.33333333, self.timer_keepalive)
		self.create_subscription(KeepAlive, '/vedrus/keepalive/safety', self.safety_keepalive_callback, 10)

		# в таймере выполняю задачу
		self.create_timer(0.2, self.task_safety_forward)
		#self.create_timer(0.2, self.task_stay_and_rotate_to_person)
		#self.create_timer(0.2, self.task_find_person_rotate_and_follow)

		self.create_subscription(Safety, '/vedrus/safety', self.safety_callback, 10)

		if DEBUG:
			self.get_logger().info('VedrusControlerNode is started')


	bearing = None
	def bearing_callback(self, msg):
		self.bearing = msg.data


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
			self.mode = ModeSafetyStop(self)
			self.mode.out = ModeForwardSlow(self)

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
			self.mode = ModeSafetyStop(self)
			self.mode.out = ModeRotateToPerson(self)
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
			self.mode = ModeSafetyStop(self)
			self.mode.out = ModeRotateToPerson(self)
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

			msg = MotorCommand()
			msg.header.stamp = self.get_clock().now().to_msg()
			msg.header.frame_id = 'left'
			msg.speed = 0
			msg.crash = True

			self.publisherMotor.publish(msg)

			msg.header.frame_id = 'right'
			self.publisherMotor.publish(msg)
			
def main(args=None):
	global node

	rclpy.init(args=args)
	node = VedrusControlerNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
