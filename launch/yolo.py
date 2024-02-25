from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov8_rknn',
            executable='solver',
            output='log',
            emulate_tty=True,
            parameters=[
                {'camera_raw_topic': '/image_raw'},
                {'inference_topic': '/yolov8/inference'},
                {'rate': 1},
                {'model': '/opt/ros/iron/family.rknn'}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'framerate': 5.0}
            ]
        )
    ])
