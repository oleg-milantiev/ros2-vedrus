"""
# ROS2 Launch File for Autonomous Robot System (short version)

---

## Included nodes

- **Realsense Camera**: Launches the `realsense2_camera_node` with specific parameters for depth, color, and other settings.
- **Controller**: Manages the robot's overall behavior.
- **Bluepill**: Handles motor control with PID parameters.
- **Safety**: Ensures safe operation.

realsense profiles, got by running rs-enumerate-devices:
    Depth       1280x720       Z16         @ 30/15/6 Hz
    Depth        848x480       Z16         @ 90/60/30/15/6 Hz
    Depth        848x100       Z16         @ 300/100 Hz
    Depth        640x480       Z16         @ 90/60/30/15/6 Hz
    Depth        640x360       Z16         @ 90/60/30/15/6 Hz
    Depth        480x270       Z16         @ 90/60/30/15/6 Hz
    Depth        424x240       Z16         @ 90/60/30/15/6 Hz
    Depth        256x144       Z16         @ 300/90 Hz

"""

from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            output='log',
            emulate_tty=True,
            name='front',
            namespace='vedrus/camera',
            parameters=[
                {'enable_depth': True},
                {'enable_color': True},
                {'enable_infra1': False},
                {'enable_infra2': False},
                {'enable_accel': False},
                {'enable_gyro': False},
                {'hole_filling_filter.enable': True},
#                {'depth_module.profile': '424x240x6'}, # I need it! But realsense node just ignoring it
                {'depth_module.profile': '848x480x30'}, # Will downscale and downrate it in the safety node
                {'rgb_camera.profile': '640x480x30'},   # Will downrate in yolo node
                {'verbose_logging': True},
                {'enable_auto_exposure': True},
            ]
        ),

        Node(
            package='vedrus',
            executable='bluepill',
            name='bluepill_left',
            parameters=[
                {'name': 'left'},
                {'port': '/dev/ttyS2'},
                {'P': 0.001},
                {'I': 0.008},
                {'D': 0.0}
            ]
        ),
        Node(
            package='vedrus',
            executable='bluepill',
            name='bluepill_right',
            parameters=[
                {'name': 'right'},
                {'port': '/dev/ttyS3'},
                {'P': 0.001},
                {'I': 0.008},
                {'D': 0.0},
                {'reverse': True}
            ]
        ),

        Node(
            package='vedrus',
            executable='controller',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='vedrus',
            executable='safety',
            output='screen',
            emulate_tty=True,
        ),

    ])
