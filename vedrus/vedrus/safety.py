'''
TBD
- ardu 1 / 2 keepalive

1280x720 6,15,30
848X480 6,15,30,60,90 <- native for d435
640x480 6,15,30,60,90
640x360 6,15,30,60,90
480x270 6,15,30,60,90
424x240 6,15,30,60,90
'''
import rclpy
from rclpy.node import Node
from vedrus_interfaces.msg import Safety, Sonar, MotorMove, KeepAlive
from yolov8_interfaces.msg import InferenceResult, Yolov8Inference

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import time
bridge = CvBridge()

import numpy as np
from scipy.ndimage import center_of_mass, label

MAX_X = 212
MAX_Y = 120

#TBD читать из инфо камеры
CAMERA_AZIMUTH = {
	'front': 0.,
	'back': 180.,
	}
CAMERA_FOV = {
	'front': 91.2,
	'back': 80.,
	}

DEPTH_PUBLISH_TOPIC = '/img_out'

#TBD
#YOLO_RANGE_ALARM = {
#	'max': 
#	}

class SafetyNode(Node):
	crash = False

	def __init__(self):
		super().__init__('safety_node')

		self.create_subscription(
			Image,
			'/depth/image_rect_raw',
			self.depth_camera_callback,
			10)
		self.create_subscription(
			Sonar,
			'/vedrus/sonar',
			self.sonar_callback,
			10)
		self.create_subscription(
			Yolov8Inference,
			'/yolov8/inference',
			self.yolo_callback,
			10)

		self.publisher_safety = self.create_publisher(Safety, '/vedrus/safety', 10)

		self.publisher_keepalive = self.create_publisher(KeepAlive, '/vedrus/keepalive/safety', 10)
		self.create_timer(0.33333333, self.timer_keepalive)
		self.create_subscription(
			KeepAlive,
			'/vedrus/keepalive/controller',
			self.controller_keepalive_callback,
			10)
		self.keepalive = time.time() + 10 # 10 sec keepalive start gap

		if 'DEPTH_PUBLISH_TOPIC' in globals():
			self.publisher_depth = self.create_publisher(Image, DEPTH_PUBLISH_TOPIC, 10)

		self.get_logger().info('Start Safety')

	def controller_keepalive_callback(self, data):
		self.keepalive = time.time()

	def timer_keepalive(self):
		if self.crash:
			return

		# отправлю свой keepalive
		msg = KeepAlive()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'safety'
		self.publisher_keepalive.publish(msg)

		# проверю контроллера keepalive
		if time.time() - self.keepalive >= 2:
			self.crash = True

			self.get_logger().info('Got controller keepalive timeout!')

			msg = MotorMove()
			msg.crash = True
			# TBD надо ли все поля перечислить?

			publisher = self.create_publisher(MotorMove, '/vedrus/left/move', 10)
			publisher.publish(msg)

			publisher = self.create_publisher(MotorMove, '/vedrus/right/move', 10)
			publisher.publish(msg)


	def yolo_callback(self, data):
		for inference in data.yolov8_inference:
			if inference.class_name in ['person', 'dog', 'cat', 'car']:
				msg = Safety()
				msg.header.frame_id = 'yolo:'+ inference.class_name
				msg.header.stamp = self.get_clock().now().to_msg()
				msg.alarm = False
				msg.warning = True
				msg.range = 100. # TBD через размер бокса, в зависимости от класса
				msg.azimuth = self.__az360((((inference.right - inference.left) / 2. + inference.left - 320.) / 320.) * CAMERA_FOV[data.header.frame_id] / 2. + CAMERA_AZIMUTH[data.header.frame_id])
				self.publisher_safety.publish(msg)

	def sonar_callback(self, data):
		#return #DEBUG
		if 0. < data.range < 100.:
			msg = Safety()
			msg.header.frame_id = 'sonar'
			msg.header.stamp = self.get_clock().now().to_msg()
			msg.alarm = 0. < data.range < 50.
			msg.warning = 50. < data.range < 100.
			msg.range = data.range
			msg.azimuth = data.azimuth
			self.publisher_safety.publish(msg)

	# Пара массивов прошлых и препрошлых глубин raw
	depths50Last = []
	depths50Prelast = []
	depths100Last = []
	depths100Prelast = []

	def depth_camera_callback(self, data):
		def rebin(a, shape):
			sh = shape[0],a.shape[0]//shape[0],shape[1],a.shape[1]//shape[1]
			return a.reshape(sh).mean(-1).mean(1)

		arr = np.array(bridge.imgmsg_to_cv2(data, 'passthrough'))
		arr = rebin(arr, (MAX_Y, MAX_X)) / 32
		arr = np.where(arr > 255, 255, arr)

		# затереть нолики = рамку. И её градиент
		#arr[0:30, 0:30] = 255
		#arr[0:MAX_Y, 0:5] = 255
		#arr[0:30, 0:MAX_X] = 255
		#arr[MAX_Y - 1, 0:MAX_X] = 255

		def centers(mask):
			# уменьшу разрешение маски по min для фильтрации мелкого мусора
			mask = mask.reshape(MAX_Y // 2, 2, MAX_X // 2, 2).min(axis=(3, 1))
			mask = np.kron(mask, np.ones((2, 2)))
			lbl = label(mask, np.ones((3,3)))

			if lbl[1] == 0:
				return []

			# уберу лейблы с малым весом (колвом точек)
			index = []
			for i in range(1, lbl[1]+1):
				if np.where(lbl[0] == i, 1, 0).sum() > 10:
					index.append(i)

			if len(index) == 1:
				return [center_of_mass(mask)]
			else:
				return center_of_mass(mask, lbl[0], index)

		cc = centers(np.where(((arr > 3) & (arr < 16)), 1, 0))

		#self.depths50PreLast = self.depth50Last[:]
		#self.depths50Last = cc[:]

		stamp = self.get_clock().now().to_msg()

		for c in cc:
			msg = Safety()
			msg.header.frame_id = 'rgbd'
			msg.header.stamp = stamp
			msg.alarm = True
			msg.warning = False
			msg.range = 50. # TBD калибрануть Z
			msg.azimuth = self.__az360((c[1] * 2. / float(MAX_X) - 1.) * CAMERA_FOV['front'] / 2)
			self.publisher_safety.publish(msg)
			arr[int(c[0]),int(c[1])] = 250

		cc = centers(np.where((18 < arr) & (arr < 34), 1, 0))

		for c in cc:
			msg = Safety()
			msg.header.frame_id = 'rgbd'
			msg.header.stamp = stamp
			msg.alarm = False
			msg.warning = True
			msg.range = 100. # TBD калибрануть Z
			msg.azimuth = self.__az360((c[1] * 2. / float(MAX_X) - 1.) * CAMERA_FOV['front'] / 2)
			self.publisher_safety.publish(msg)
			arr[int(c[0]),int(c[1])] = 150

		if 'DEPTH_PUBLISH_TOPIC' in globals():
			self.publisher_depth.publish(bridge.cv2_to_imgmsg(arr.astype('uint8'), encoding='mono8'))

	def __az360(self, azimuth):
		return azimuth + 360 if azimuth < 0 else azimuth;

def main(args=None):
	rclpy.init(args=args)
	node = SafetyNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
