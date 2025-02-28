from setuptools import find_packages, setup

package_name = 'vedrus'

setup(
	name=package_name,
	version='0.0.1',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='Oleg Milantiev',
	maintainer_email='oleg@milantiev.com',
	description='Vedrusohod bot package',
	license='Apache-2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'ardu = vedrus.ardu:main',
			'bluepill = vedrus.bluepill:main',
#			'gyro = vedrus.gyro:main',
#			'test = vedrus.test:main',
			'circle = vedrus.circle:main',
			'controller = vedrus.controller.controller:main',
			'keyboard = vedrus.keyboard:main',
			'screen = vedrus.screen:main',
			'safety = vedrus.safety:main',
			'server = vedrus.server:main',
			'qmc5883l = vedrus.qmc5883l:main',
			'ads1115 = vedrus.ads1115:main',
			'bme = vedrus.bme:main',
			'sonar = vedrus.sonar:main',
		],
	},
)
