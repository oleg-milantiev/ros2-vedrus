"""
This is a ROS2 launch file designed to initialize and configure three USB cameras along with a Realsense camera for an autonomous robot system. 
The script begins by detecting all available video devices using the command 'v4l2-ctl --list-devices', filtering them based on specific templates, 
and checking whether each device supports the YUYV format. It then proceeds to configure and launch ROS2 nodes that publish images from these detected cameras. 
Additionally, the script includes other ROS2 nodes for controlling various aspects of the robot such as its behavior, sensors like IMU (Inertial Measurement Unit), pressure sensors, etc.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
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
				{'depth_module.profile': '848x480x15'},
				{'rgb_camera.profile': '640x480x15'},
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
				{'image_width': 640},
				{'framerate': 5.0},
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
				{'image_width': 640},
				{'framerate': 5.0},
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
				{'image_width': 640},
				{'framerate': 5.0},
				{'publish_compressed': False}
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
				{'classes': ("person","bicycle","car","motorbike","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush")},
				{'camera_ids': ('front', 'rear', 'left', 'right')}, # как звать камеры. Этот id уйдёт в header.frame_id
				{'camera_rates': (5, 1, 1, 1)}, # обрабатывать каждый пятый кадр с первой и каждый кадр с остальных (5 раз в секунду с каждой)
#				{'save_image_rates': (5, 5)}, # сохранять изображение раз в N обработок (в моём примере раз в пять секунд)
				{'camera_raw_topics': ('/vedrus/camera/front/color/image_raw', '/vedrus/camera/rear/image_raw', '/vedrus/camera/left/image_raw', '/vedrus/camera/right/image_raw')}, # откуда читать картинки
				{'inference_topic': '/yolov8/inference'}, # куда кидать солвы
			]
		),

	])






'''
		Node(
			package='vedrus',
			executable='controller',
			output='screen',
			emulate_tty=True,
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
			executable='safety',
			output='screen',
			emulate_tty=True,
		),

		Node(
			package='vedrus',
			executable='ads1115',
			name='ads1115',
			parameters=[
				{'i2c_bus': 2},
				{'i2c_address': 0x48},
				{'adc_channels': ['channel_0', 'channel_1', 'channel_2', 'channel_3']},
				{'adc_gains': ['1', '2', '1', '1']},
				{'adc_dividers': [1., 1., 1., 1.]},
				{'topic_name': '/adc_values'},
			]
		),
		Node(
			package='vedrus',
			executable='bme',
			name='bme',
			output='screen',
			parameters=[
				{'i2c_bus': 2},
				{'i2c_address': 0x76},
				{'topic_name': 'bme280_data'}
			]
		),
		Node(
			package='vedrus',
			executable='sonar',
			name='soanr',
			output='screen',
			parameters=[
				{'pin_numbers': [127, 125, 124, 144, 122, 120, 123, 121]},
				{'topic_name': 'sonar_data'},
			]
		),

		Node(
			package='vedrus',
			executable='qmc5883l',
			output='screen',
			emulate_tty=True,
		),
		Node(
			package='vedrus',
			executable='server',
			output='screen',
			emulate_tty=True,
		),
'''
