from setuptools import find_packages, setup

package_name = 'yolov8_rknn'

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
    description='YOLOv8 RKNN Image Solver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solver = yolov8_rknn.solver:main'
        ],
    },
)
