'''
TODO not yet tested

ADS1115 ROS2 Node

This ROS2 node reads analog-to-digital conversion values from an ADS1115 ADC sensor over I2C and publishes the values to a ROS2 topic.

Parameters:
- `i2c_bus` (int): The I2C bus number (e.g., 1).
- `i2c_address` (int): The I2C address of the ADS1115 sensor (e.g., 0x48).
- `adc_channels` (list of str): Names of the ADC channels to read (e.g., ['channel_0', 'channel_1']).
- `adc_gains` (list of str): Gain configuration for each channel (e.g., ['1', '2/3']).
- `topic_name` (str): The name of the ROS2 topic to publish ADC values (e.g., '/adc_values').

ROS2 Launch Example:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='ads1115_node',
            name='ads1115_node',
            parameters=[
                {'i2c_bus': 2},
                {'i2c_address': 72},
                {'adc_channels': ['channel_0', 'channel_1', 'channel_2', 'channel_3']},
                {'adc_gains': ['1', '2/3', '1', '1']},
                {'topic_name': '/adc_values'},
            ]
        )
    ])
```

Install Required Modules:
```bash
pip install adafruit-circuitpython-ads1x15
pip install rclpy
```

ADC Gain Configuration Options:
- '2/3' = +/-6.144V
- '1' = +/-4.096V
- '2' = +/-2.048V
- '4' = +/-1.024V
- '8' = +/-0.512V
- '16' = +/-0.256V

Usage Example:
```bash
ros2 run your_package_name ads1115_node
```
'''

import rclpy
from rclpy.node import Node
import Adafruit_ADS1x15

class ADS1115Node(Node):
    def __init__(self):
        super().__init__('ads1115_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 2)
        self.declare_parameter('i2c_address', 0x48)
        self.declare_parameter('adc_channels', ['channel_0', 'channel_1'])
        self.declare_parameter('adc_gains', ['1', '2/3', '1', '1'])
        self.declare_parameter('topic_name', '/adc_values')

        # Get parameters
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        adc_channels = self.get_parameter('adc_channels').get_parameter_value().string_array_value
        adc_gains = self.get_parameter('adc_gains').get_parameter_value().string_array_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Initialize I2C and ADS1115
        self.adc = Adafruit_ADS1x15.ADS1115(address=i2c_address, busnum=i2c_bus)

        # Map gains to ADS1115 gain configuration
        # Choose a gain of 1 for reading voltages from 0 to 4.09V.
        # Or pick a different gain to change the range of voltages that are read:
        #  - 2/3 = +/-6.144V
        #  -   1 = +/-4.096V
        #  -   2 = +/-2.048V
        #  -   4 = +/-1.024V
        #  -   8 = +/-0.512V
        #  -  16 = +/-0.256V
        # See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
        self.allowed_gains = ['2/3', '1', '2', '4', '8', '16']

        #TBD
        '''
        # Configure ADS1115 channels and gains
        self.analog_inputs = []
        for i, channel_name in enumerate(adc_channels):
            if i >= len(adc_gains):
                self.get_logger().error(f"Gain not provided for channel {channel_name}, using default gain '1'.")
                gain = '1'
            else:
                gain = adc_gains[i]

            if gain not in self.allowed_gains:
                self.get_logger().error(f"Invalid gain '{gain}' for channel {channel_name}, using default gain '1'.")
                gain = '1'

            self.ads.gain = self.gain_map[gain]
            self.analog_inputs.append(AnalogIn(self.ads, getattr(ADS1115, f'P{i}')))
        '''

        # Create a topic publisher
        self.publisher = self.create_publisher(dict, topic_name, 10)

        # Create a timer to read and publish ADC values
        self.timer = self.create_timer(1.0, self.publish_adc_values)

    def publish_adc_values(self):
        GAIN = 1

        values = [0]*4
        for i in range(4):
            values[i] = self.adc.read_adc(i, gain=GAIN)

        print(values)
        sys.exit()

        adc_values = {}
        for i, analog_input in enumerate(self.analog_inputs):
            adc_values[f'channel_{i}'] = analog_input.voltage

        self.publisher.publish(adc_values)


def main(args=None):
    rclpy.init(args=args)
    node = ADS1115Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
