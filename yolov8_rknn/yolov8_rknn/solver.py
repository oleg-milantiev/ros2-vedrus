#!/usr/bin/env python3

import time
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from cv_bridge import CvBridge # type: ignore
from sensor_msgs.msg import Image # type: ignore
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

IMG_PUBLISH_TOPIC = '/yolov8/img_out'


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
                ('obj_threshold', 0.55),
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

    def filter_boxes(self, boxes, box_confidences, box_class_probs):
        """Vectorized version of filter boxes"""
        # Combine scores with class probabilities
        combined_scores = box_confidences.ravel() * np.max(box_class_probs, axis=-1)

        # Find positions where scores exceed threshold
        valid_mask = combined_scores >= self.obj_threshold

        # Return filtered results
        return (boxes[valid_mask],
                np.argmax(box_class_probs, axis=-1)[valid_mask],
                combined_scores[valid_mask])

    def nms_boxes(self, boxes, scores):
        """Replace custom NMS with OpenCV's NMSBoxes."""
        if len(boxes) == 0:
            return np.array([])
        boxes_list = boxes.tolist()
        scores_list = scores.tolist()
        indices = cv2.dnn.NMSBoxes(boxes_list, scores_list, 0, self.nms_threshold)
        return indices.flatten() if len(indices) > 0 else np.array([])

    def dfl(self, position):
        """Optimized DFL using vectorized operations"""
        x = position.reshape(-1, 4, 16, *position.shape[-2:])  # (n, 4, 16, h, w)

        # Stable softmax calculation
        max_values = np.max(x, axis=2, keepdims=True)
        exp = np.exp(x - max_values)
        softmax = exp / np.sum(exp, axis=2, keepdims=True)

        # Vectorized weighted sum using broadcasting
        return np.sum(softmax * np.arange(16, dtype=np.float32).reshape(1, 1, 16, 1, 1), axis=2)

    def box_process(self, position):
        IMG_SIZE = (640, 640)
        grid_h, grid_w = position.shape[2:4]

        # Cache grid and stride calculations
        cache_key = (grid_h, grid_w)
        if not hasattr(self, 'grid_cache'):
            self.grid_cache = {}
            self.stride_cache = {}

        if cache_key not in self.grid_cache:
            col, row = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
            grid = np.stack([col, row], axis=0).reshape(1, 2, grid_h, grid_w)
            stride = np.array([IMG_SIZE[1]/grid_w, IMG_SIZE[0]/grid_h]).reshape(1,2,1,1)

            self.grid_cache[cache_key] = grid
            self.stride_cache[cache_key] = stride

        grid = self.grid_cache[cache_key]
        stride = self.stride_cache[cache_key]

        position = self.dfl(position)
        box_xy = (grid + 0.5 - position[:, 0:2]) * stride
        box_xy2 = (grid + 0.5 + position[:, 2:4]) * stride

        return np.concatenate((box_xy, box_xy2), axis=1)

    def post_process(self, input_data):
        boxes, scores, classes_conf = [], [], []
        default_branch = 3
        pair_per_branch = len(input_data) // default_branch

        for i in range(default_branch):
            branch_idx = i * pair_per_branch

            box_output = self.box_process(input_data[branch_idx])
            boxes.append(box_output.transpose(0, 2, 3, 1).reshape(-1, 4))

            # Process class confidences with optimized reshape
            class_output = input_data[branch_idx+1]
            classes_conf.append(class_output.transpose(0, 2, 3, 1).reshape(-1, len(self.classes)))

            # Calculate scores
            scores.append(np.ones(class_output.shape[0] * class_output.shape[2] * class_output.shape[3], 
                            dtype=np.float32))

        # Vectorized concatenation
        boxes = np.concatenate(boxes)
        classes_conf = np.concatenate(classes_conf)
        scores = np.concatenate(scores)

        # Vectorized filtering
        max_class_probs = np.max(classes_conf, axis=1)
        combined_scores = scores * max_class_probs
        valid_mask = combined_scores >= self.obj_threshold

        boxes = boxes[valid_mask]
        classes = np.argmax(classes_conf[valid_mask], axis=1)
        scores = combined_scores[valid_mask]

        # NMS logic with per-class processing
        nboxes, nclasses, nscores = [], [], []
        for c in np.unique(classes):
            mask = classes == c
            b = boxes[mask]
            s = scores[mask]

            keep = self.nms_boxes(b, s)

            if len(keep) > 0:
                nboxes.append(b[keep])
                nclasses.append(np.full(keep.shape[0], c))
                nscores.append(s[keep])

        if not nboxes:
            return None, None, None

        return (np.concatenate(nboxes), 
                np.concatenate(nclasses), 
                np.concatenate(nscores))

    def camera_callback(self, data, camera):
        start_time = time.time()
        self.rates[camera] -= 1

        if self.rates[camera] != 0:
            return

        self.rates[camera] = self.ratesInit[camera]

        img = bridge.imgmsg_to_cv2(data, 'bgr8')

        h, w = img.shape[:2]
        if h < 640 or w < 640:
            top = (640 - h) // 2
            bottom = 640 - h - top
            left = (640 - w) // 2
            right = 640 - w - left
            img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0, 0, 0))

        results = self.rknn.inference(inputs=[img])

        boxes, classes, scores = self.post_process(results)
        
        # test set
        # boxes = [[1, 2, 3, 4]]
        # classes = [1]
        # scores = [0.9]

        if boxes is not None:
            self.yolov8_inference.header.frame_id = self.cameras[camera]
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

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
                    img = cv2.rectangle(img, (self.inference_result.left, self.inference_result.top), (self.inference_result.right, self.inference_result.bottom), (0, 0, 255), 2)

            self.yolov8_inference.time = time.time() - start_time

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
