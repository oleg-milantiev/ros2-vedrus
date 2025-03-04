#!/usr/bin/env python3

import time
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from cv_bridge import CvBridge # type: ignore
from sensor_msgs.msg import Image # type: ignore
import numpy as np
import cv2

bridge = CvBridge()

# debug
IMG_PUBLISH_TOPIC = '/circler/img_out'

class Circle_imager(Node):

    def __init__(self):
        super().__init__('circle_imager')

        '''
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_raw_topics', rclpy.Parameter.Type.STRING_ARRAY),
            ]
        )

        topics = self.get_parameter('camera_raw_topics').get_parameter_value().string_array_value
        '''
        topics = ['/vedrus/camera/rear/image_raw']

        self.init_fisheye_correction()

        for i in range(len(topics)):
            self.create_subscription(
                Image,
                topics[i],
                lambda msg, i=i: self.camera_callback(msg, i),
                10)

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
        K = np.array([[w * k5, 0, w/2],
                    [0, h * k5, h/2],
                    [0, 0, 1]], dtype=np.float32)
        D = np.array([k1, k2, k3, k4], dtype=np.float32)
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=k6)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)

    def camera_callback(self, data, camera):
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        
        if data.header.frame_id != 'front':
            img = self._correct_fisheye_distortion(img)

        if 'IMG_PUBLISH_TOPIC' in globals():
            self.publisher_img.publish(bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    def _correct_fisheye_distortion(self, image):
        undistorted_image = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return cv2.copyMakeBorder(undistorted_image[60:540, 80:720],
                                  20, 20, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0]) 

def main(args=None):
    rclpy.init(args=args)

    node = Circle_imager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
