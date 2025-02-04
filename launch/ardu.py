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
				{'p': 1.5},
				{'i': 0.0},
				{'d': 0.0},
			]
		),
		Node(
			package='vedrus',
			executable='ardu',
			output='screen',
			emulate_tty=True,
			parameters=[
				{'device': '/dev/ttyUSB1'},
				{'p': 1.5},
				{'i': 0.0},
				{'d': 0.0},
			]
		),
		
		#Node(
		#	package='mpu6050driver',
		#	executable='mpu6050driver',
		#	name='mpu6050driver_node',
		#	output='screen',
		#	emulate_tty=True,
		#	parameters=[
		#		{'calibrate': True},
		#		# Gyroscope range: 0 -> +-250°/s, 1 -> +-500°/s, 2 -> +-1000°/s, 3 -> +-2000°/s
		#		{'gyro_range': 0},
				# Acceleration range: 0 -> +-2g, 1 -> +-4g, 2 -> +-8g, 3 -> +-16g
		#		{'accel_range': 0},
				# Digital low pass filter bandwidth [0-6]: 0 -> 260Hz, 1 -> 184Hz, 2 -> 94Hz, 3 -> 44
		#		{'dlpf_bandwidth': 3},
		#		{'frequency': 50},
		#	]
		#),

		#Node(
		#	package='hmc5883ldriver',
		#	executable='hmc5883ldriver',
		#	name='hmc5883ldriver_node',
		#	output='screen',
		#	emulate_tty=True,
		#	parameters=[
		#	]
		#),

		#Node(
		#	package='tf2_ros',
		#	executable='static_transform_publisher',
		#	name='bl_imu',
		#	arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
		#),
		
		#Node(
		#	package='imu_filter_madgwick',
		#	executable='imu_filter_madgwick_node',
		#	#package='imu_complementary_filter',
		#	#executable='complementary_filter_node',
		#	output='screen',
		#	emulate_tty=True,
		#	parameters=[
		#		{'use_mag': True},
		#		{'remove_gravity_vector': False},
		#	],
		#	remappings=[
		#		('imu/data_raw', '/imu')
		#	]
		#),

		Node(
			package='vedrus',
			executable='server',
			output='screen',
			emulate_tty=True,
		),

	])



'''
		Node(
			package='robot_localization',
			executable='ukf_node',
			name='ukf_node',
			parameters=[
				{'frequency': 50.},
				{'sensor_timeout': 1.},
				{'odom_frame': 'odom'},
				{'base_link_frame': 'base_link'},
				{'world_frame': 'odom'},
				{'imu0': '/imu'},
				{'imu0_config': [False, False, False, True, True, True, False, False, False, True, True, True, True, True, True]},
				{'imu0_differential': True},
				{'imu0_relative': False},
				{'imu0_remove_gravitational_acceleration': True},
				{'two_d_mode': True},
				{'print_diagnostics': True},

				#imu0_nodelay: false
				#imu0_differential: false
				#
				#imu0_queue_size: 7
				#imu0_pose_rejection_threshold: 0.8                 # Note the difference in parameter names
				#imu0_twist_rejection_threshold: 0.8                #
				#imu0_linear_acceleration_rejection_threshold: 0.8  #

				#imu0_remove_gravitational_acceleration: true
			]
		),




'''
