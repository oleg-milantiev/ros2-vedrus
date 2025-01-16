'''
This ROS2 node reads temperature, humidity, and pressure data from a BME280 sensor
connected via I2C and publishes the readings to ROS2 topics. The I2C bus number, 
sensor address, and topic name are configurable via ROS2 parameters.

Dependencies:
- ROS2 Foxy or newer
- Python 3.6 or newer
- Richard Hull BME280 library
  Install using: `pip install bme280`

ROS2 Parameters:
- i2c_bus (int): I2C bus number to use (default: 2)
- i2c_address (int): I2C address of the BME280 sensor (default: 0x76)
- topic_name (str): Base name of the ROS2 topic to publish data (default: 'bme280_data')

Published Topics:
- <topic_name>/temperature (sensor_msgs/Temperature): Temperature data in Celsius
- <topic_name>/humidity (sensor_msgs/RelativeHumidity): Relative humidity as a fraction
- <topic_name>/pressure (sensor_msgs/FluidPressure): Atmospheric pressure in Pascals

Example Usage:
1. Create a ROS2 package and place this file inside the package's `scripts` directory.
2. Install the required library using: `pip install adafruit-circuitpython-bme280`.
3. Build and source your ROS2 workspace.
4. Run the node with default parameters:
ros2 run <your_package_name> bme280_publisher

5. To launch with custom parameters, create a ROS2 launch file (Python-style) as follows:

Example Launch File (bme280_launch.py):
-------------------------------------------------
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 return LaunchDescription([
     Node(
         package='<your_package_name>',
         executable='bme280_publisher',
         name='bme280_publisher',
         output='screen',
         parameters=[
             {'i2c_bus': 2},
             {'i2c_address': 0x76},
             {'topic_name': 'custom_bme280_data'}
         ]
     ),
 ])
-------------------------------------------------
6. Run the launch file:
ros2 launch <your_package_name> bme280_launch.py
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
import smbus2
import bme280

class BME280Publisher(Node):
    def __init__(self):
        super().__init__('bme280_publisher')

        # Declare parameters
        self.declare_parameter('i2c_bus', 2)
        self.declare_parameter('i2c_address', 0x76)
        self.declare_parameter('topic_name', 'bme280_data')

        # Retrieve parameters
        i2c_bus_number = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Initialize I2C and BME280 sensor
        try:
            self.bus = smbus2.SMBus(i2c_bus_number)
            self.calibration_params = bme280.load_calibration_params(self.bus, self.i2c_address)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BME280 sensor: {e}")
            self.destroy_node()
            return

        self.publisher_temperature = self.create_publisher(Temperature, f"{topic_name}/temperature", 10)
        self.publisher_humidity = self.create_publisher(RelativeHumidity, f"{topic_name}/humidity", 10)
        self.publisher_pressure = self.create_publisher(FluidPressure, f"{topic_name}/pressure", 10)

        self.timer = self.create_timer(1.0, self.publish_sensor_data)  # Publish at 1 Hz

        self.get_logger().info(f"BME280 publisher node initialized. Publishing to topic '{topic_name}'")

    def publish_sensor_data(self):
        try:
            data = bme280.sample(self.bus, self.i2c_address, self.calibration_params)

            # Publish temperature
            temp_msg = Temperature()
            temp_msg.temperature = data.temperature
            temp_msg.variance = 0.0  # Set variance as 0 (default)
            self.publisher_temperature.publish(temp_msg)

            # Publish humidity
            humidity_msg = RelativeHumidity()
            humidity_msg.relative_humidity = data.humidity / 100.0  # Convert percentage to fraction
            humidity_msg.variance = 0.0  # Set variance as 0 (default)
            self.publisher_humidity.publish(humidity_msg)

            # Publish pressure
            pressure_msg = FluidPressure()
            pressure_msg.fluid_pressure = data.pressure
            pressure_msg.variance = 0.0  # Set variance as 0 (default)
            self.publisher_pressure.publish(pressure_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to read or publish sensor data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BME280Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
