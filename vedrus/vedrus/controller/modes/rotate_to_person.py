import csv
import time
from .safety_stop import ModeSafetyStop
from .parent import ModeParent

DEBUG = True
CSV = False
RGBD_FOV = 92.2

# Режим "Поворот к персоне" (с движением вперёд или без)
class ModeRotateToPerson(ModeParent):
	ROTATE_SPEED_MAX = 5.

	# Азимут последней замеченной персоны (и время)
	azimuth = None
	persons = []
	lastPersonTime = None

	# время последнего safety::alarm
	lastAlarmTime = None

	# последовательно пойманные safety::alarm
	alarmCount = 0

	# стартовал ли режим? (движется ли сейчас?)
	started = False

	# Данные глубин / азимутов за последнюю секунду (две?)
	depths = []

	# TBD move to parent
#	def __init__(self, node):
#		if CSV:
#			self.csv = csv.writer(open('/opt/ros/iron/log/csv/controller/mode/rotate_to_person.csv', mode='w', newline=''))
#
#			self.csv.writerow([
#				'time', 'rotateAzimuth', 'rotateSpeed', 'move', 'left', 'right'
#			])
#		parent.__init__(node)

	# поймал safety.warning
	def warning(self, msg):
		second = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

		# TBD две персоны с разными азимутами (ex.: > 10°)
		# TBD N персон с близкими азимутами (ex.: 10°)
		if msg.header.frame_id == 'yolo:person' and self.node.bearing is not None:
			'''
			# игнорирую +-5°, т.к. персона суть не точная
			if abs(msg.azimuth) < 5:
				return
			'''

			# в сообщении азимут по base_link
			# переведу в магнитный
			azimuth = msg.azimuth + self.node.bearing
			if azimuth > 360:
				azimuth -= 360

			self.azimuth = azimuth
			self.lastPersonTime = time.time()

			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: got person: {:.4f}°'.format(self.azimuth))

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
			azimuth = msg.azimuth + self.node.bearing
			azimuth = azimuth % 360

			self.depths.append({
				'time': second,
				'azimuth': azimuth,
				'range': msg.range,
			})

			# Удаляю не актуальные в окне 0.5с
			self.depths = [item for item in self.depths if item['time'] >= (second - 0.5)]

	# поймал safety.alarm
	def alarm(self, msg):
		# RGBD камера на солнце шумит (доделать safety фильтрацию last / preLast)
		# пока что игнорирую алармы от RGBD
		#if msg.header.frame_id == 'rgbd':
		#	return

		self.lastAlarmTime = time.time()
		self.alarmCount += 1

		if DEBUG:
			self.node.get_logger().info('ModeRotateToPerson: got alarm count='+ str(self.alarmCount))

	# По собранному в self.warning данныым self.depths ищу, были ли предупреждения (объект ближе метра) примерно по текущему азимуту
	def __isPersonNearAtAzimuth(self):
		if self.azimuth is None or len(self.depths) == 0:
			return False

		gap = 5. # +-5° допуск

		azimuthDepths = [item for item in self.depths if self._anglesMatch(self.azimuth, item['azimuth'], gap)]
		print(azimuthDepths)
		return len(azimuthDepths) > 0

	def cycle(self):
		'''
		TBD
		- controller: есть моща поворота, а bearing не меняется?
		- ardu: малая моща на моторе без движения уводит его в защиту
		'''
		self._statusInit()
		self._statusAppend('mode', 'RotateToPerson')
		self._statusAppend('forward', 'Y' if self.forward else 'N')
		self._statusAppend('bearing', '{:.2f}'.format(self.node.bearing))

		# вход в режим. Начало поворотов
		if not self.started:
			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: start to rotate/move to person. Forward=' +('Y' if self.forward else 'N'))

			self.started = True

		# случайная одиночная тревога за 0.5с. Игнорю
		if self.alarmCount == 1 and time.time() - self.lastAlarmTime >= 0.5:
			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: drop single alarm')

			self.alarmCount = 0

		# много тревоги. Остановка!
		if self.alarmCount > 1:
			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: got many alarms! Will stop')

			self.node.left(0)
			self.node.right(0)

			route = ModeSafetyStop(self.node)
			route.out = ModeRotateToPerson(self.node)
			route.out.forward = self.forward

			self._statusAppend('action', 'Stop by many alarms')
			self._statusPublish()

			return route

		# Ещё нет данных компАса
		if self.node.bearing is None:
			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: No bearing data.')

			self._statusAppend('action', 'No bearing')
			self._statusPublish()

			return self

		# Персона есть? А то сидим и ждём
		if self.azimuth is None:
			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: no person.')

			self._statusAppend('action', 'No person')
			self._statusPublish()

			return self

		# Персоны не было 3 секунды. ПОТЕРЯЛ!!! :)
		if (time.time() - self.lastPersonTime) > 3:
			if DEBUG:
				self.node.get_logger().info('ModeRotateToPerson: No new person 3 sec.')

			# Стоп машина! :)
			self.node.left(0)
			self.node.right(0)

			self.azimuth = None

			self._statusAppend('action', 'Lost person')
			self._statusPublish()

			return self

		'''
		# хочу повернуть и не могу уже три секунды
		if (time.time() - self.lastBearingChanged) > 3:
			if DEBUG:
				node.get_logger().info('ModeRotateToPerson: No rotate 3 sec.')

			# Стоп машина! :)
			node.left(0.)  # без тормоза
			node.right(0.) # плавно скорость в ноль

			self.azimuth = None

			self._statusAppend('action', 'No rotate')
			self._statusPublish()

			return self
		'''

		# -180 .. +180
		rotateAzimuth = self.azimuth - self.node.bearing
		if rotateAzimuth <= -180:
			rotateAzimuth += 360
		elif rotateAzimuth > 180:
			rotateAzimuth -= 360

		# -1 .. +1
		rotateSpeed = rotateAzimuth / 180

		self._statusAppend('rotateAzimuth', '{:.2f}'.format(rotateAzimuth))
		self._statusAppend('rotateSpeed', '{:.2f}'.format(rotateSpeed))

		isInRGBDCamera = self._anglesMatch(self.azimuth, self.node.bearing, RGBD_FOV)
		if DEBUG:
			self.node.get_logger().info('ModeRotateToPerson: is in RGBD camera: '+('Y' if isInRGBDCamera else 'N') )

		isPersonNear = self.__isPersonNearAtAzimuth()
		if DEBUG:
			self.node.get_logger().info('ModeRotateToPerson: is person near: '+('Y' if isPersonNear else 'N') )

		# если задан флаг "двигаться вперёд"
		# если азимут в FOV передней RGBD камеры
		# если расстояние до персоны > 1м
		if self.forward and isInRGBDCamera and not isPersonNear:
			move = True

			right = self.ROTATE_SPEED_MAX - (rotateSpeed * 2. * self.ROTATE_SPEED_MAX)
			left = self.ROTATE_SPEED_MAX + (rotateSpeed * 2. * self.ROTATE_SPEED_MAX)
		else:
			move = False

			if -5. < rotateAzimuth < 5.:
				if DEBUG:
					self.node.get_logger().info('ModeRotateToPerson: Done rotate to person')

				# Стоп машина! :)
				self.node.left(0)
				self.node.right(0)

				self.azimuth = None

				self._statusAppend('action', 'Done rotate')
				self._statusPublish()

				return self

			else:

				left = self.ROTATE_SPEED_MAX * rotateSpeed * 6. 
				right = -self.ROTATE_SPEED_MAX * rotateSpeed * 6.

		self._statusAppend('move', 'Y' if move else 'N')
		if DEBUG:
			self.node.get_logger().info('ModeRotateToPerson: move: '+('Y' if move else 'N'))
			self.node.get_logger().info('ModeRotateToPerson: left: '+str(left))
			self.node.get_logger().info('ModeRotateToPerson: right: '+str(right))

		self.node.left(max(min(left, self.ROTATE_SPEED_MAX), -self.ROTATE_SPEED_MAX))
		self.node.right(max(min(right, self.ROTATE_SPEED_MAX), -self.ROTATE_SPEED_MAX))

		if CSV:
			stamp = self.node.get_clock().now().to_msg()
			stampSecond = stamp.sec + stamp.nanosec / 1e9

			self.csv.writerow([
				stampSecond, rotateAzimuth, rotateSpeed, move, left, right
			])

#		if DEBUG:
#			print('{:12.2f} {:12.2f} {:12.2f}      {:b} {:12.4f} {:12.4f}'.format(stampSecond, rotateAzimuth, rotateSpeed, move, left, right))

		self._statusPublish()

		# ... или продолжить жить в этом режиме
		return self
