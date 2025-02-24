'''
TBD
- ardu 1 / 2 keepalive

1280x720 6,15,30
848X480 6,15,30,60,90 <- native for d435
640x480 6,15,30,60,90
640x360 6,15,30,60,90
480x270 6,15,30,60,90
424x240 6,15,30,60,90

# ros2 topic list
/imu/bearing
/parameter_events
/rosout
/tf_static
/vedrus/adc_values
/vedrus/bme280_data/humidity
/vedrus/bme280_data/pressure
/vedrus/bme280_data/temperature
/vedrus/camera/front/color/camera_info
/vedrus/camera/front/color/image_raw
/vedrus/camera/front/color/image_raw/compressed
/vedrus/camera/front/color/image_raw/compressedDepth
/vedrus/camera/front/color/image_raw/theora
/vedrus/camera/front/color/metadata
/vedrus/camera/front/depth/camera_info
/vedrus/camera/front/depth/image_rect_raw
/vedrus/camera/front/depth/image_rect_raw/compressed
/vedrus/camera/front/depth/image_rect_raw/compressedDepth
/vedrus/camera/front/depth/image_rect_raw/theora
/vedrus/camera/front/depth/metadata
/vedrus/camera/front/extrinsics/depth_to_color
/vedrus/camera/left/camera_info
/vedrus/camera/left/image_raw
/vedrus/camera/left/image_raw/compressed
/vedrus/camera/left/image_raw/compressedDepth
/vedrus/camera/left/image_raw/theora
/vedrus/camera/rear/camera_info
/vedrus/camera/rear/image_raw
/vedrus/camera/rear/image_raw/compressed
/vedrus/camera/rear/image_raw/compressedDepth
/vedrus/camera/rear/image_raw/theora
/vedrus/camera/right/camera_info
/vedrus/camera/right/image_raw
/vedrus/camera/right/image_raw/compressed
/vedrus/camera/right/image_raw/compressedDepth
/vedrus/camera/right/image_raw/theora
/vedrus/motor/command
/vedrus/motor/status
/vedrus/sonar
/yolov8/img_out
/yolov8/inference
'''
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from vedrus_interfaces.msg import Safety, Sonar, MotorCommand, KeepAlive
from yolov8_interfaces.msg import InferenceResult, Yolov8Inference
from sensor_msgs.msg import Image # type: ignore
from cv_bridge import CvBridge # type: ignore
import cv2
import numpy as np
from scipy.ndimage import center_of_mass, label
import time

CAMERA_AZIMUTH = {'front': 0., 'left': 250., 'right': 110., 'rear': 180.}
CAMERA_FOV = {'front': 91.2, 'left': 130., 'right': 130., 'rear': 130.}
DEPTH_PUBLISH_TOPIC = '/img_out'

bridge = CvBridge()

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.crash = False
        self.depth_count = 0
        
        self.create_subscription(Image, '/vedrus/camera/front/depth/image_rect_raw', 
                               self.depth_camera_callback, 1) 
        self.create_subscription(Sonar, '/vedrus/sonar', self.sonar_callback, 1)
        self.create_subscription(Yolov8Inference, '/yolov8/inference', self.yolo_callback, 1)

        self.publisher_safety = self.create_publisher(Safety, '/vedrus/safety', 1)
        self.publisher_keepalive = self.create_publisher(KeepAlive, '/vedrus/keepalive/safety', 1)
        self.create_timer(0.33333333, self.timer_keepalive)
        self.create_subscription(KeepAlive, '/vedrus/keepalive/controller', 
                               self.controller_keepalive_callback, 1)
        
        self.keepalive = time.time() + 10  # 10 sec keepalive start gap
        
        if 'DEPTH_PUBLISH_TOPIC' in globals():
            self.publisher_depth = self.create_publisher(Image, DEPTH_PUBLISH_TOPIC, 1)

        self.get_logger().info('Start Safety')

    def controller_keepalive_callback(self, data):
        self.keepalive = time.time()

    def timer_keepalive(self):
        if self.crash:
            return

        msg = KeepAlive()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'safety'
        self.publisher_keepalive.publish(msg)

        if time.time() - self.keepalive >= 2:
            self.crash = True
            self.get_logger().info('Got controller keepalive timeout!')
            
            msg = MotorCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.speed = 0
            msg.crash = True
            
            publisher = self.create_publisher(MotorCommand, '/vedrus/motor/command', 1)
            for side in ['left', 'right']:
                msg.header.frame_id = side
                publisher.publish(msg)

    def yolo_callback(self, data):
        for inference in data.yolov8_inference:
            if inference.class_name in ['person', 'dog', 'cat', 'car']:
                msg = Safety()
                msg.header.frame_id = f'yolo:{inference.class_name}'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.alarm = False
                msg.warning = True
                msg.range = 100.  # TBD: калибровка через размер бокса
                center_x = (inference.right + inference.left) / 2.0 - 320.0
                azimuth = (center_x / 320.0) * CAMERA_FOV[data.header.frame_id] / 2.0 + CAMERA_AZIMUTH[data.header.frame_id]
                msg.azimuth = self.__az360(azimuth)
                self.publisher_safety.publish(msg)

    def sonar_callback(self, data):
        if 0. < data.range < 100.:
            msg = Safety()
            msg.header.frame_id = 'sonar'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.alarm = data.range < 50.
            msg.warning = 50. <= data.range < 100.
            msg.range = data.range
            msg.azimuth = data.azimuth
            self.publisher_safety.publish(msg)

    def depth_camera_callback(self, data):
        # Downrate from 30 to 5 fps
        self.depth_count += 1
        if self.depth_count % 6 != 0:
            return  

        start_time = time.time()
        
        # Convert ROS Image 848x480x30 into OpenCV image 212x120x5
        #depth_image = np.clip((bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')[::4, ::4] >> 2), 0, 255).astype(np.uint8)
        depth_image = np.clip(cv2.resize(
            bridge.imgmsg_to_cv2(data, desired_encoding='passthrough') >> 2,
            (0, 0),
            fx=0.25, fy=0.25,
            interpolation=cv2.INTER_AREA
        ), 0, 255).astype(np.uint8)

        # erase zeros = frame, and its gradient
        depth_image[0:120, 0:7] = 255 # left line
        depth_image[0:20, 0:8] = 255 # up-left corner
        depth_image[0:1, 0:212] = 255 # up line
        depth_image[119, 0:212] = 255 # down line
        depth_image[115:119, 195:212] = 255 # down-right corner

        min_depth = 0      # 0/4 мм
        max_depth = 500/4  # 500 мм
        
        mask = cv2.inRange(depth_image, min_depth, max_depth)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        stamp = self.get_clock().now().to_msg()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 10:
                # Calculate moments to find the center of mass
                M = cv2.moments(contour)
                if M['m00'] != 0:  
                    cx = int(M['m10'] / M['m00'])
                    #cy = int(M['m01'] / M['m00'])

                    msg = Safety()
                    msg.header.frame_id = 'rgbd'
                    msg.header.stamp = stamp
                    msg.alarm = True
                    msg.warning = False
                    msg.range = 50.
                    msg.azimuth = self.__az360((cx * 2.0 / 212 - 1.0) * CAMERA_FOV['front'] / 2.0)
                    self.publisher_safety.publish(msg)

        if 'DEPTH_PUBLISH_TOPIC' in globals():
            self.publisher_depth.publish(bridge.cv2_to_imgmsg(mask, encoding='mono8'))

    def __az360(self, azimuth):
        return azimuth + 360.0 if azimuth < 0 else azimuth

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()