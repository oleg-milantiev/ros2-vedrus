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
    description='Vedrus bot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ardu = vedrus.ardu:main',
            'gyro = vedrus.gyro:main',
            'controller = vedrus.controller:main',
            'test = vedrus.test:main'
        ],
    },
)
