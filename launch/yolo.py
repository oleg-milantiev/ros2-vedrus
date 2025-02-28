# cameras and YOLO debug launch

from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore
import subprocess

SEEK_TEMPLATES = [
    "USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.3):", # rear
    "USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.4.1):", # right
    "USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.4.2):" # left
]

def get_video_devices():
    """Get the output of v4l2-ctl --list-devices and parse it."""
    output = subprocess.check_output(['v4l2-ctl', '--list-devices']).decode('utf-8')
    lines = output.splitlines()
    camera_devices = {}

    current_camera = None
    for line in lines:
        line = line.strip()
        if line in SEEK_TEMPLATES:
            current_camera = line
            camera_devices[current_camera] = []
        elif current_camera and line.startswith('/dev/video'):
            camera_devices[current_camera].append(line)

    return camera_devices

def check_yuyv_support(video_device):
    """Check if a video device supports YUYV format."""
    try:
        output = subprocess.check_output(['v4l2-ctl', '-d', video_device, '--list-formats-ext']).decode('utf-8')
        return "'YUYV' (YUYV 4:2:2)" in output
    except subprocess.CalledProcessError:
        return False

def find_yuyv_cameras():
    """Find cameras that support YUYV format."""
    camera_devices = get_video_devices()
    yuyv_devices = {}

    for camera, devices in camera_devices.items():
        for video_device in devices:
            if check_yuyv_support(video_device):
                yuyv_devices[camera] = video_device
                break

    return yuyv_devices


def generate_launch_description():
    yuyv_cameras = find_yuyv_cameras()

    if len(yuyv_cameras) < 3:
        print(f"Found USB 2.0 Cameras: {len(yuyv_cameras)}");
        return

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
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='rear',
            namespace='vedrus/camera/rear',
            output='log',
            emulate_tty=True,
            parameters=[
                {'video_device': yuyv_cameras['USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.3):']},
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 5.0},
                {'exposure_auto': 3},
                {'exposure_auto_priority': 1},
                {'backlight_compensation': 2},
                {'gamma': 0},
                {'gain': 10},
                {'brightness': 0},
                {'contrast': 20},
                {'publish_compressed': False}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='log',
            name='right',
            namespace='vedrus/camera/right',
            emulate_tty=True,
            parameters=[
                {'video_device': yuyv_cameras['USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.4.1):']},
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 5.0},
                {'exposure_auto': 3},
                {'exposure_auto_priority': 1},
                {'backlight_compensation': 2},
                {'gamma': 0},
                {'gain': 10},
                {'brightness': 0},
                {'contrast': 20},
                {'publish_compressed': False}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='log',
            name='left',
            namespace='vedrus/camera/left',
            emulate_tty=True,
            parameters=[
                {'video_device': yuyv_cameras['USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.4.2):']},
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 5.0},
                {'exposure_auto': 3},
                {'exposure_auto_priority': 1},
                {'backlight_compensation': 2},
                {'gamma': 0},
                {'gain': 10},
                {'brightness': 0},
                {'contrast': 20},
                {'publish_compressed': False}
            ]
        ),
        

    ])

'''
        Node(
            package='vedrus',
            executable='circle',
            output='screen',
            emulate_tty=True,
            parameters=[
#                {'camera_raw_topics': ('/vedrus/camera/front/color/image_raw', '/vedrus/camera/rear/image_raw', '/vedrus/camera/left/image_raw', '/vedrus/camera/right/image_raw')},
                {'camera_raw_topics': ['/vedrus/camera/left/image_raw']},
            ]
        ),

/dev/video6 - rear
/dev/video10 - left
/dev/video14 - right
v4l2-ctl -d /dev/video10 -c exposure_auto=3 -c exposure_auto_priority=1 -c backlight_compensation=2 -c brightness=0 -c gamma=0 -c gain=10 -c contrast=20

[usb_cam_node_exe-4] This driver supports the following formats:
[usb_cam_node_exe-4]    rgb8
[usb_cam_node_exe-4]    yuyv
[usb_cam_node_exe-4]    yuyv2rgb
[usb_cam_node_exe-4]    uyvy
[usb_cam_node_exe-4]    uyvy2rgb
[usb_cam_node_exe-4]    mono8
[usb_cam_node_exe-4]    mono16
[usb_cam_node_exe-4]    y102mono8
[usb_cam_node_exe-4]    raw_mjpeg
[usb_cam_node_exe-4]    mjpeg2rgb
[usb_cam_node_exe-4]    m4202rgb

        Node(
            package='yolov8_rknn',
            executable='solver',
            output='log',
            emulate_tty=True,
            parameters=[
#				{'model': '/opt/ros/iron/family.rknn'},
#				{'classes': ("alex", "bars", "dad", "fish", "ivan", "marta", "max", "mom", "oleg", "poly", "turtle", "yury")},
#                {'model': '/opt/ros/iron/yolov8n-1.5.2.rknn'},
                {'model': '/opt/ros/iron/yolo11n-rk3566.rknn'},
                {'classes': ("person","bicycle","car","motorbike","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush")},
                {'camera_ids': ('front', 'rear', 'left', 'right')}, # как звать камеры. Этот id уйдёт в header.frame_id
                {'camera_rates': (5, 1, 1, 1)}, # обрабатывать каждый пятый кадр с первой и каждый кадр с остальных (5 раз в секунду с каждой)
                {'camera_raw_topics': ('/vedrus/camera/front/color/image_raw', '/vedrus/camera/rear/image_raw', '/vedrus/camera/left/image_raw', '/vedrus/camera/right/image_raw')}, # откуда читать картинки
                {'inference_topic': '/yolov8/inference'}, # куда кидать солвы
            ]
        ),
'''