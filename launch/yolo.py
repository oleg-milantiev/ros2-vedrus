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
                {'hole_filling_filter.enable': True},
                {'depth_module.profile': '640x480x6'},
                {'rgb_camera.profile': '640x480x6'},
#                {'pointcloud.enable': True},
            ]
        ),
#        Node(
#            package='influx',
#            executable='logger',
#            output='screen',
#            emulate_tty=True,
#            parameters=[
#                {'host': 'localhost'},
#                {'port': 8086},
#                {'user': 'r2'},
#                {'password': 'r2'},
#                {'database': 'r2'},
#            ]
#        ),
        Node(
            package='vedrus',
            executable='ardu',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'device': '/dev/ttyUSB0'},
            ]
        ),
        Node(
            package='vedrus',
            executable='ardu',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'device': '/dev/ttyUSB1'},
            ]
        ),
    ])
