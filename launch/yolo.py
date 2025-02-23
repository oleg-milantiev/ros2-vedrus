"""
# ROS2 Launch File for Autonomous Robot System (short version)

---

## Features

3. **Node Initialization**:
   - **Realsense Camera**: Launches the `realsense2_camera_node` with specific parameters for depth, color, and other settings.
   - **Controller**: Manages the robot's overall behavior.
   - **Bluepill**: Handles motor control with PID parameters.
   - **Safety**: Ensures safe operation.

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
                {'depth_module.profile': '848x480x15'},
                {'rgb_camera.profile': '640x480x15'},
                {'enable_auto_exposure': True},
            ]
        ),
        Node(
            package='yolov8_rknn',
            executable='solver',
            output='log',
            emulate_tty=True,
            parameters=[
#				{'model': '/opt/ros/iron/family.rknn'},
#				{'classes': ("alex", "bars", "dad", "fish", "ivan", "marta", "max", "mom", "oleg", "poly", "turtle", "yury")},
                {'model': '/opt/ros/iron/yolov8n-1.5.2.rknn'},
#                {'model': '/opt/ros/iron/yolo11n-rk3566.rknn'},
                {'classes': ("person","bicycle","car","motorbike","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush")},
                {'camera_ids': ('front', 'rear', 'left', 'right')}, # как звать камеры. Этот id уйдёт в header.frame_id
                {'camera_rates': (5, 1, 1, 1)}, # обрабатывать каждый пятый кадр с первой и каждый кадр с остальных (5 раз в секунду с каждой)
                {'camera_raw_topics': ('/vedrus/camera/front/color/image_raw', '/vedrus/camera/rear/image_raw', '/vedrus/camera/left/image_raw', '/vedrus/camera/right/image_raw')}, # откуда читать картинки
                {'inference_topic': '/yolov8/inference'}, # куда кидать солвы
            ]
        ),

    ])
