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
        
        for i in range(len(topics)):
            self.create_subscription(
                Image,
                topics[i],
                lambda msg, i=i: self.camera_callback(msg, i),
                10)

        if 'IMG_PUBLISH_TOPIC' in globals():
            self.publisher_img = self.create_publisher(Image, IMG_PUBLISH_TOPIC, 10)

    def camera_callback(self, data, camera):
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        
        if data.header.frame_id != 'front':
            img = self._correct_fisheye_distortion(img)

        if 'IMG_PUBLISH_TOPIC' in globals():
            self.publisher_img.publish(bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    def _correct_fisheye_distortion(self, img):
        DIM = (img.shape[1], img.shape[0])
        K = np.array([[500.0, 0.0, DIM[0] / 2],  
                    [0.0, 500.0, DIM[1] / 2],  
                    [0.0, 0.0, 1.0]])
        D = np.array([[-0.15], [-0.05], [0.0], [0.0]])  

        # Увеличиваем balance, чтобы тянуть за края и расширить FOV
        balance = 0.5  # Можно пробовать 1.3, если нужно ещё больше FOV
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        return undistorted_img


def main(args=None):
    rclpy.init(args=args)

    node = Circle_imager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
