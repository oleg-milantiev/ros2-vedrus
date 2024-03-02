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
			if line.startswith('USB 2.0 Camera: USB Camera'):
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
				{'camera_rates': (5, 6)}, # обрабатывать каждый пятый кадр с первой и каждый шестой кадр со второй камеры (то есть раз в секунду для fps 5 и 6 соответственно)
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
		Node(
			package='vedrus',
			executable='safety',
			output='screen',
			emulate_tty=True,
		),
	])
