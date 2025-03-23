# cameras and YOLO debug launch

from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore
import subprocess
import hashlib

CAMERA_LEFT = "USB Camera: USB Camera (usb-0000:00:14.0-1):"
CAMERA_RIGHT = "USB Camera: USB Camera (usb-0000:00:14.0-7):"
CAMERA_REAR = "USB Camera: USB Camera (usb-0000:00:14.0-9):"

SEEK_TEMPLATES = [CAMERA_LEFT, CAMERA_RIGHT, CAMERA_REAR]

def get_video_devices():
    """Get the output of v4l2-ctl --list-devices and parse it."""
    output = subprocess.check_output(['v4l2-ctl', '--list-devices']).decode('utf-8')
    lines = output.splitlines()
    camera_devices = {}

    current_camera = None
    for line in lines:
        line = line.strip()
        if line in SEEK_TEMPLATES:
            current_camera = hashlib.md5(line.encode()).hexdigest()
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
            name='front',
            namespace='vedrus/camera',
            parameters=[
                {'enable_infra1': False},
                {'enable_infra2': False},
#                {'depth_module.depth_profile': '848x480x6'},
                {'depth_module.depth_profile': '424x240x6'},
                {'rgb_camera.color_profile': '848x480x6'},
                {'hole_filling_filter.enable': True},
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
                {'video_device': yuyv_cameras[ hashlib.md5(CAMERA_REAR.encode()).hexdigest() ]},
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 5.0},
                {'exposure_auto': 0},
                {'autoexposure': False},
                {'exposure_auto_priority': 0},
                {'backlight_compensation': 2},
                {'gamma': 100},
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
                {'video_device': yuyv_cameras[ hashlib.md5(CAMERA_RIGHT.encode()).hexdigest() ]},
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 5.0},
                {'exposure_auto': 0},
                {'autoexposure': False},
                {'exposure_auto_priority': 0},
                {'backlight_compensation': 2},
                {'gamma': 100},
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
                {'video_device': yuyv_cameras[ hashlib.md5(CAMERA_LEFT.encode()).hexdigest() ]},
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 5.0},
                {'exposure_auto': 0},
                {'autoexposure': False},
                {'exposure_auto_priority': 0},
                {'backlight_compensation': 2},
                {'gamma': 100},
                {'gain': 10},
                {'brightness': 0},
                {'contrast': 20},
                {'publish_compressed': False}
            ]
        ),

        Node(
            package='vedrus',
            executable='bluepill',
            name='bluepill_left',
            parameters=[
                {'name': 'left'},
                {'port': '/dev/ttyS2'},
                {'max_pwm': 100},
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
                {'max_pwm': 100},
                {'P': 0.001},
                {'I': 0.008},
                {'D': 0.0},
                {'reverse': True}
            ]
        ),

        Node(
            package='yolov8_nvidia',
            executable='solver',
            output='log',
            emulate_tty=True,
            parameters=[
                {'model': 'yolo11n.pt'},
                {'camera_ids': ('front', 'rear', 'left', 'right')}, # как звать камеры. Этот id уйдёт в header.frame_id
                {'camera_rates': (1, 1, 1, 1)}, # обрабатывать каждый кадр с первой и каждый кадр с остальных (5 раз в секунду с каждой)
                {'camera_raw_topics': ('/vedrus/camera/front/color/image_raw', '/vedrus/camera/rear/image_raw', '/vedrus/camera/left/image_raw', '/vedrus/camera/right/image_raw')}, # откуда читать картинки
                {'inference_topic': '/yolov8/inference'}, # куда кидать солвы
            ]
        ),

    ])

'''
find expo again
v4l2-ctl -d /dev/video6 -c exposure_auto=3 -c exposure_auto_priority=1 -c backlight_compensation=2 -c brightness=0 -c gamma=0 -c gain=10 -c contrast=20

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

'''