from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
	output = subprocess.check_output(['v4l2-ctl', '--list-devices']).decode('utf-8')
	found = False
	usbCamDevice = ''

	for line in output.split("\n"):
		if found:
			usbCamDevice = line.strip()
			break
		else:
			if line.startswith('USB Camera: USB Camera (usb-xhci-hcd.4.auto-1.4.2):'):
				found = True

	if not found:
		print('USB 2.0 Camera is not found');
		return

	return LaunchDescription([
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
				{'camera_ids': ('back', 'front')}, # как звать камеры. Этот id уйдёт в header.frame_id
				{'camera_rates': (5, 5)}, # обрабатывать каждый пятый кадр с первой и каждый шестой кадр со второй камеры (то есть раз в секунду для fps 5 и 6 соответственно)
#				{'save_image_rates': (5, 5)}, # сохранять изображение раз в N обработок (в моём примере раз в пять секунд)
				{'camera_raw_topics': ('/image_raw', '/color/image_raw')}, # откуда читать картинки
				{'inference_topic': '/yolov8/inference'}, # куда кидать солвы
			]
		),
		Node(
			package='usb_cam',
			executable='usb_cam_node_exe',
			output='log',
			emulate_tty=True,
			parameters=[
				{'video_device': usbCamDevice},
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
			package='vedrus',
			executable='controller',
			output='screen',
			emulate_tty=True,
		),

#		Node(
#			package='vedrus',
#			executable='ardu',
#			output='screen',
#			emulate_tty=True,
#			parameters=[
#				{'device': '/dev/ttyUSB2'},
#				{'p': 4.5},
#				{'i': 0.15},
#				{'d': 0.15},
#			]
#		),
#		Node(
#			package='vedrus',
#			executable='ardu',
#			output='screen',
#			emulate_tty=True,
#			parameters=[
#				{'device': '/dev/ttyUSB1'},
#				{'p': 4.5},
#				{'i': 0.15},
#				{'d': 0.15},
#			]
#		),
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
	])
