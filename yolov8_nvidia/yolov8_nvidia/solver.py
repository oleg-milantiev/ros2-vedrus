#!/usr/bin/env python3

'''
TODO:
- слишком огромный файл. Выделить работу с YOLO в отдельный файл
- rate и IMG_PUBLISH_TOPIC, и IMG_SAVE_PATH в параметры или конфиг
- включение / выключение фич (типа save, publish, exposure и distort) через параметры
  мне не нравится интеграция с моим роботом. Модуль становится не полезным для других
- вынести работу с камерами в отдельный модуль
'''

import time
import rclpy # type: ignore
from rcl_interfaces.srv import SetParameters # type: ignore
from rcl_interfaces.msg import Parameter, ParameterValue # type: ignore
from rcl_interfaces.msg import ParameterType # type: ignore
from rclpy.node import Node # type: ignore
from cv_bridge import CvBridge # type: ignore
from sensor_msgs.msg import Image # type: ignore
import numpy as np # type: ignore
from yolov8_interfaces.msg import InferenceResult, Yolov8Inference
import os
import glob
import cv2 # type: ignore
from datetime import datetime
from ultralytics import YOLO # type: ignore

bridge = CvBridge()

#IMG_SAVE_PATH = '/mnt/emmc/IMG/'
IMG_PUBLISH_TOPIC = '/yolov8/img_out'

class YOLOv8_solver(Node):

    def __init__(self):
        self.exposures = {}
        self.exposureClients = {}
        self.exposureRates = {}
        self.exposureRatesInit = {}
        self.saveNumber = {}
        self.saveRates = {}
        self.saveRatesInit = {}

        super().__init__('yolov8_nvidia_solver')

        self.declare_parameters(
            namespace='',
            parameters=[
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

        self.cameras = self.get_parameter('camera_ids').get_parameter_value().string_array_value
        self.obj_threshold = self.get_parameter('obj_threshold').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value

        self.yolov8_inference = Yolov8Inference()

        topics = self.get_parameter('camera_raw_topics').get_parameter_value().string_array_value

        for i in range(len(topics)):
            self.create_subscription(
                Image,
                topics[i],
                lambda msg, i=i: self.camera_callback(msg, i, topics[i]),
                10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, self.get_parameter('inference_topic').get_parameter_value().string_value, 1)

        self.ratesInit = self.get_parameter('camera_rates').get_parameter_value().integer_array_value
        self.rates = self.ratesInit[:]

        self.model = YOLO(self.get_parameter('model').get_parameter_value().string_value)

        self.init_fisheye_correction()

        if 'IMG_SAVE_PATH' in globals():
            self.init_save_numbers()

        if 'IMG_PUBLISH_TOPIC' in globals():
            self.publisher_img = self.create_publisher(Image, IMG_PUBLISH_TOPIC, 10)

    def init_fisheye_correction(self):
        # 4.jpg
        k1=-0.5402212953096065
        k2=0.10362133512295424
        k3=0.09528874965041999
        k4=0.11721272885952241
        k5=0.9381049520596112
        k6=0.7916997424822741

        h, w = 600, 800
        K = np.array([[-w * k5, 0, w/2],
                    [0, -h * k5, h/2],
                    [0, 0, 1]], dtype=np.float32)
        D = np.array([k1, k2, k3, k4], dtype=np.float32)
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=k6)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)

    def _correct_fisheye_distortion(self, image):
        undistorted_image = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return cv2.copyMakeBorder(undistorted_image[:, 80:720],
                                  20, 20, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0]) 

    def init_save_numbers(self):
        for camera_name in os.listdir(IMG_SAVE_PATH):
            camera_path = os.path.join(IMG_SAVE_PATH, camera_name)
            if os.path.isdir(camera_path):
                image_files = glob.glob(os.path.join(camera_path, '*png'))

                if image_files:
                    # Extract the number from the first image name
                    image_files.sort(reverse=True)
                    latest_image = image_files[0]
                    latest_number = int(os.path.splitext(os.path.basename(latest_image))[0].split('-')[-1])
                else:
                    latest_number = 0

                self.saveNumber[camera_name] = latest_number + 1

    def save_image(self, image, camera):
        if camera not in self.saveNumber:
            self.saveNumber[camera] = 1

        if camera not in self.saveRates:
            self.saveRates[camera] = 1
        self.saveRates[camera] -= 1
        if self.saveRates[camera] != 0:
            return
        if camera not in self.saveRatesInit:
            self.saveRatesInit[camera] = 5 # Save every 5th image 
        self.saveRates[camera] = self.saveRatesInit[camera]

        date_str = datetime.now().strftime('%Y-%m-%d')
        image_number = f'{self.saveNumber[camera]:06d}'
        filename = f'{date_str}-{camera}-{image_number}.png'
        save_path = os.path.join(IMG_SAVE_PATH, camera)
        os.makedirs(save_path, exist_ok=True)
        cv2.imwrite(os.path.join(save_path, filename), image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        self.saveNumber[camera] += 1

    def correctExposure(self, img, name):
        if name == 'front':
            return
        
        if name not in self.exposureRates:
            self.exposureRates[name] = 1
        self.exposureRates[name] -= 1
        if self.exposureRates[name] != 0:
            return
        if name not in self.exposureRatesInit:
            self.exposureRatesInit[name] = 7
        self.exposureRates[name] = self.exposureRatesInit[name]

        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        avg_gray = np.mean(gray_image)
        
        old_exposure = self.exposures.get(name, 100)
        needed_exposure = np.clip(old_exposure * (90 / avg_gray), 1, 10000)

        self.get_logger().info(f'{name}: Average gray value: {avg_gray}, Needed exposure: {needed_exposure}')

        if abs(needed_exposure - old_exposure) / old_exposure > 0.05:
            # Adjust the exposure of the camera
            self.setExposure(name, int(needed_exposure))
            self.exposures[name] = needed_exposure

    def setExposure(self, name, exposure):
        if name not in self.exposureClients:
            self.exposureClients[name] = self.create_client(
                SetParameters,
                f"/vedrus/camera/{name}/{name}/set_parameters"
            )

            if not self.exposureClients[name].wait_for_service(timeout_sec=3.0):
                self.get_logger().error("Parameter service not available!")
                return

        request = SetParameters.Request()
        param = Parameter()
        param.name = "exposure"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = exposure
        request.parameters.append(param)

        future = self.exposureClients[name].call_async(request)

        #rclpy.spin_until_future_complete(self, future)
        #if future.result():
        #    self.get_logger().info(f"Parameter updated: {future.result().results}")
        #else:
        #    self.get_logger().error("Failed to update parameter!")

    def camera_callback(self, data, camera, topic):
        start_time = time.time()
        self.rates[camera] -= 1

        if self.rates[camera] != 0:
            return

        self.rates[camera] = self.ratesInit[camera]

        img = bridge.imgmsg_to_cv2(data, 'bgr8')

        # /vedrus/camera/front/color/image_raw -> front
        name = topic.split('/')[3]

        # Process input image
        if name == 'front':
            h, w = img.shape[:2]
            scale = min(640 / h, 640 / w)
            nh, nw = int(h * scale), int(w * scale)
            img_resized = cv2.resize(img, (nw, nh))
            top = (640 - nh) // 2
            bottom = 640 - nh - top
            left = (640 - nw) // 2
            right = 640 - nw - left
            img = cv2.copyMakeBorder(img_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        else:
            img = self._correct_fisheye_distortion(img)

        self.correctExposure(img, name)

        if 'IMG_SAVE_PATH' in globals():
            self.save_image(img, name)

        # Inference with Ultralytics
        results = self.model(img)

        names = results[0].names
        boxes = results[0].boxes.xyxy
        classes = results[0].boxes.cls
        scores = results[0].boxes.conf

        # test set
        #boxes = [[1, 2, 3, 4]]
        #classes = [1]
        #scores = [0.9]

        if len(boxes) > 0:
            self.yolov8_inference.header.frame_id = self.cameras[camera]
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            for i in range(len(boxes)):
                self.inference_result = InferenceResult()
                self.inference_result.score = float(scores[i])
                self.inference_result.class_name = names[int(classes[i].cpu())]
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
