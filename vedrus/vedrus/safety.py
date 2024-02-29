import rclpy
from rclpy.node import Node
from vedrus_interfaces.msg import Safety

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
bridge = CvBridge()

import numpy as np
from scipy.ndimage import center_of_mass, label


class SafetyNode(Node):
	def __init__(self):
		super().__init__('safety_node')

		self.create_subscription(
			Image,
			'/depth/image_rect_raw',
			self.camera_callback,
			10)
		self.publisher = self.create_publisher(Image, '/img_out', 10)

	def rebin(self, a, shape):
		sh = shape[0],a.shape[0]//shape[0],shape[1],a.shape[1]//shape[1]
		return a.reshape(sh).mean(-1).mean(1)

	def camera_callback(self, data):
		arr = np.array(bridge.imgmsg_to_cv2(data, 'passthrough'))
		arr = self.rebin(arr, (120, 160)) / 32
		arr = np.where(arr > 255, 255, arr)
		arr[0:120, 0:7] = 255

		mask = np.where(arr < 20, 1, 0)
		lbl = label(mask)
		c = center_of_mass(mask, lbl[0], range(1, lbl[1]))
		print(c)
		for i in c:
			arr[int(i[0]),int(i[1])] = 255

		self.publisher.publish(bridge.cv2_to_imgmsg(arr.astype('uint8'), encoding='mono8'))


def main(args=None):
	rclpy.init(args=args)
	node = SafetyNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
