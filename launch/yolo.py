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
                {'classes': ("alex", "bars", "dad", "fish", "ivan", "marta", "max", "mom", "oleg", "poly", "turtle", "yury")},
                {'model': '/opt/ros/iron/family.rknn'},
                {'camera_ids': ('back', 'front')},
                {'camera_rates': (5, 6)},
                {'camera_raw_topics': ('/image_raw', '/color/image_raw')},
                {'inference_topic': '/yolov8/inference'},
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='log',
            emulate_tty=True,
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'framerate': 5.0}
            ]
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            output='log',
            emulate_tty=True,
            parameters=[
                {'enable_accel': True},
                {'enable_gyro': True},
                {'enable_depth': True},
                {'enable_color': True},
                {'enable_infra1': False},
                {'enable_infra2': False},
                {'depth_module.profile': '640x480x6'},
                {'rgb_camera.profile': '640x480x6'},
                {'pointcloud.enable': True},
            ]
        )
    ])
