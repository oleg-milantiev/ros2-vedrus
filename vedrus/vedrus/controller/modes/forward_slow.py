import time

from .safety_stop import ModeSafetyStop
from .parent import ModeParent

DEBUG = True

# Режим "Медленное движение вперёд"
class ModeForwardSlow(ModeParent):
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
		# RGBD камера на солнце шумит (доделать safety фильтрацию last / preLast)
		# пока что игнорирую алармы от RGBD
		#if msg.header.frame_id == 'rgbd':
		#	return

		self.lastAlarmTime = time.time()
		self.alarmCount += 1

		if DEBUG:
			self.node.get_logger().info('ModeForwardSlow: got alarm count='+ str(self.alarmCount))
		return

	def cycle(self):
		# вход в режим. Начало движения
		if not self.started:
			if DEBUG:
				self.node.get_logger().info('ModeForwardSlow: start to move')

			self.started = True
			self.node.left(500)
			self.node.right(500)

		# случайная одиночная тревога за 0.5с. Игнорю
		if self.alarmCount == 1 and time.time() - self.lastAlarmTime >= 0.5:
			if DEBUG:
				self.node.get_logger().info('ModeForwardSlow: drop single alarm')

			self.alarmCount = 0

		# много тревоги. Остановка!
		if self.alarmCount > 1:
			if DEBUG:
				self.node.get_logger().info('ModeForwardSlow: got many alarms! Will stop')

			self.node.left(0)
			self.node.right(0)

			route = ModeSafetyStop(self.node)
			route.out = ModeForwardSlow(self.node)

			return route

		#if DEBUG:
		#	node.get_logger().info('ModeForwardSlow: continue moving...')

		# ... или продолжить жить в этом режиме
		return self