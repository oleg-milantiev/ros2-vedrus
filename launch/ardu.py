from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
	return LaunchDescription([
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
