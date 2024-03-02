#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

from rknnlite.api import RKNNLite

from yolov8_interfaces.msg import InferenceResult, Yolov8Inference
import cv2

bridge = CvBridge()

'''
TODO:
- проверка входных параметров, включая размер изображения
- IMG_PUBLISH_TOPIC в необязательный параметр
'''

#IMG_PUBLISH_TOPIC = '/yolov8/img_out'

IMG_SAVE_PATH = '/opt/ros/iron/log/'


class YOLOv8_solver(Node):

	def __init__(self):
		super().__init__('yolov8_rknn_solver')

		self.declare_parameters(
			namespace='',
			parameters=[
				('classes', rclpy.Parameter.Type.STRING_ARRAY),
				('model', rclpy.Parameter.Type.STRING),
				('camera_ids', rclpy.Parameter.Type.STRING_ARRAY),
				('camera_rates', rclpy.Parameter.Type.INTEGER_ARRAY),
				('save_image_rates', []),
				('camera_raw_topics', rclpy.Parameter.Type.STRING_ARRAY),
				('inference_topic', rclpy.Parameter.Type.STRING),
				('obj_threshold', 0.35),
				('nms_threshold', 0.65),
			]
		)

		self.classes = self.get_parameter('classes').get_parameter_value().string_array_value
		self.cameras = self.get_parameter('camera_ids').get_parameter_value().string_array_value
		self.obj_threshold = self.get_parameter('obj_threshold').get_parameter_value().double_value
		self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value

		self.yolov8_inference = Yolov8Inference()

		topics = self.get_parameter('camera_raw_topics').get_parameter_value().string_array_value

		for i in range(len(topics)):
			self.create_subscription(
				Image,
				topics[i],
				lambda msg, i=i: self.camera_callback(msg, i),
				10)

		self.yolov8_pub = self.create_publisher(Yolov8Inference, self.get_parameter('inference_topic').get_parameter_value().string_value, 1)

		self.ratesInit = self.get_parameter('camera_rates').get_parameter_value().integer_array_value
		self.rates = self.ratesInit[:]

		self.saveRatesInit = self.get_parameter('save_image_rates').get_parameter_value().integer_array_value
		self.saveRates = self.saveRatesInit[:]

		self.rknn = RKNNLite()
		self.rknn.load_rknn(self.get_parameter('model').get_parameter_value().string_value)
		self.rknn.init_runtime()

		# TBD вторую модель в необязательный параметр
		#self.rknn2 = RKNNLite()
		#self.rknn2.load_rknn('/opt/ros/iron/family.rknn')
		#self.rknn2.init_runtime()

		if 'IMG_PUBLISH_TOPIC' in globals():
			self.publisher_img = self.create_publisher(Image, IMG_PUBLISH_TOPIC, 10)

	def camera_callback(self, data, camera):
		self.rates[camera] -= 1

		if self.rates[camera] != 0:
			return

		self.rates[camera] = self.ratesInit[camera]

		img = bridge.imgmsg_to_cv2(data, 'bgr8')

		#if img.shape[0] > 640 or img.shape[0] > 640:
			# TBD уменьшение до 640х640 с рамкой
			#img = cv2.resize(img, (640, 640))
		#else
		if img.shape[0] < 640 or img.shape[0] < 640:
			# добавляю чёрную рамку до 640x640
			frame = np.zeros((640, 640, 3), dtype=np.uint8)
			x_offset = (640 - img.shape[1]) // 2
			y_offset = (640 - img.shape[0]) // 2
			frame[y_offset:y_offset+img.shape[0], x_offset:x_offset+img.shape[1]] = img
			img = frame

		if len(self.saveRates) > 0:
			self.saveRates[camera] -= 1

			if self.saveRates[camera] == 0:
				cv2.imwrite(IMG_SAVE_PATH + self.cameras[camera] +'-'+ datetime.fromtimestamp(self.get_clock().now().to_msg().sec).strftime('%Y%m%d-%H%M%S') +'.jpg', img)

				self.saveRates[camera] = self.saveRatesInit[camera]

		results = self.rknn.inference(inputs=[img])

		IMG_SIZE = (640, 640)

		def filter_boxes(boxes, box_confidences, box_class_probs):
			"""Filter boxes with object threshold.
			"""
			box_confidences = box_confidences.reshape(-1)
			candidate, class_num = box_class_probs.shape

			class_max_score = np.max(box_class_probs, axis=-1)
			classes = np.argmax(box_class_probs, axis=-1)

			_class_pos = np.where(class_max_score* box_confidences >= self.obj_threshold)
			scores = (class_max_score* box_confidences)[_class_pos]

			boxes = boxes[_class_pos]
			classes = classes[_class_pos]

			return boxes, classes, scores

		def nms_boxes(boxes, scores):
			"""Suppress non-maximal boxes.
			# Returns
				keep: ndarray, index of effective boxes.
			"""
			x = boxes[:, 0]
			y = boxes[:, 1]
			w = boxes[:, 2] - boxes[:, 0]
			h = boxes[:, 3] - boxes[:, 1]

			areas = w * h
			order = scores.argsort()[::-1]

			keep = []
			while order.size > 0:
				i = order[0]
				keep.append(i)

				xx1 = np.maximum(x[i], x[order[1:]])
				yy1 = np.maximum(y[i], y[order[1:]])
				xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
				yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

				w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
				h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
				inter = w1 * h1

				ovr = inter / (areas[i] + areas[order[1:]] - inter)
				inds = np.where(ovr <= self.nms_threshold)[0]
				order = order[inds + 1]
			keep = np.array(keep)
			return keep

		def dfl(position):
			x = np.array(position)
			n, c, h, w = x.shape
			p_num = 4
			mc = c // p_num
			y = x.reshape(n, p_num, mc, h, w)

			max_values = np.max(y, axis=2, keepdims=True)
			exp_values = np.exp(y - max_values)
			y = exp_values / np.sum(exp_values, axis=2, keepdims=True)

			acc_matrix = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
			y = np.sum(y * acc_matrix, axis=2)

			return y

		def box_process(position):
			grid_h, grid_w = position.shape[2:4]
			col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
			col = col.reshape(1, 1, grid_h, grid_w)
			row = row.reshape(1, 1, grid_h, grid_w)
			grid = np.concatenate((col, row), axis=1)
			stride = np.array([IMG_SIZE[1]//grid_h, IMG_SIZE[0]//grid_w]).reshape(1,2,1,1)

			position = dfl(position)
			box_xy  = grid +0.5 -position[:,0:2,:,:]
			box_xy2 = grid +0.5 +position[:,2:4,:,:]
			xyxy = np.concatenate((box_xy*stride, box_xy2*stride), axis=1)

			return xyxy

		def post_process(input_data):
			boxes, scores, classes_conf = [], [], []
			defualt_branch=3
			pair_per_branch = len(input_data)//defualt_branch
			# Python 忽略 score_sum 输出
			for i in range(defualt_branch):
				boxes.append(box_process(input_data[pair_per_branch*i]))
				classes_conf.append(input_data[pair_per_branch*i+1])
				scores.append(np.ones_like(input_data[pair_per_branch*i+1][:,:1,:,:], dtype=np.float32))

			def sp_flatten(_in):
				ch = _in.shape[1]
				_in = _in.transpose(0,2,3,1)
				return _in.reshape(-1, ch)

			boxes = [sp_flatten(_v) for _v in boxes]
			classes_conf = [sp_flatten(_v) for _v in classes_conf]
			scores = [sp_flatten(_v) for _v in scores]

			boxes = np.concatenate(boxes)
			classes_conf = np.concatenate(classes_conf)
			scores = np.concatenate(scores)

			# filter according to threshold
			boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

			# nms
			nboxes, nclasses, nscores = [], [], []
			for c in set(classes):
				inds = np.where(classes == c)
				b = boxes[inds]
				c = classes[inds]
				s = scores[inds]
				keep = nms_boxes(b, s)

				if len(keep) != 0:
					nboxes.append(b[keep])
					nclasses.append(c[keep])
					nscores.append(s[keep])

			if not nclasses and not nscores:
				return None, None, None

			boxes = np.concatenate(nboxes)
			classes = np.concatenate(nclasses)
			scores = np.concatenate(nscores)

			return boxes, classes, scores

		boxes, classes, scores = post_process(results)

		'''
		print(' ')
		print(boxes)
		print(classes)
		print(scores)
		'''

		self.yolov8_inference.header.frame_id = self.cameras[camera]
		self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

		if boxes is not None:
			for i in range(len(boxes)):
				self.inference_result = InferenceResult()
				self.inference_result.score = float(scores[i])
				self.inference_result.class_name = self.classes[classes[i]]
				self.inference_result.left = int(boxes[i][0])
				self.inference_result.top = int(boxes[i][1])
				self.inference_result.right = int(boxes[i][2])
				self.inference_result.bottom = int(boxes[i][3])
				self.yolov8_inference.yolov8_inference.append(self.inference_result)

				if 'IMG_PUBLISH_TOPIC' in globals():
					img = cv2.rectangle(img, (self.inference_result.top, self.inference_result.left), (self.inference_result.bottom, self.inference_result.right), (0, 0, 255), 2)

			self.yolov8_pub.publish(self.yolov8_inference)
			self.yolov8_inference.yolov8_inference.clear()

			if 'IMG_PUBLISH_TOPIC' in globals():
				self.publisher_img.publish(bridge.cv2_to_imgmsg(img, encoding='bgr8'))

def main(args=None):
	rclpy.init(args=args)

	solver_node = YOLOv8_solver()
	rclpy.spin(solver_node)

	solver_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
