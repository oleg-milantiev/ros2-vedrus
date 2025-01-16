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
from std_msgs.msg import Header
from vedrus_interfaces.msg import Voltage
import Adafruit_ADS1x15

class ADS1115Node(Node):
    def __init__(self):
        super().__init__('ads1115_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 2)
        self.declare_parameter('i2c_address', 0x48)
        self.declare_parameter('adc_channels', ['channel_0', 'channel_1'])
        self.declare_parameter('adc_gains', ['1', '1'])
        self.declare_parameter('adc_dividers', [1.0, 1.0])
        self.declare_parameter('topic_name', '/adc_values')

        # Get parameters
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        adc_channels = self.get_parameter('adc_channels').get_parameter_value().string_array_value
        adc_gains = self.get_parameter('adc_gains').get_parameter_value().string_array_value
        adc_dividers = self.get_parameter('adc_dividers').get_parameter_value().double_array_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        if len(adc_channels) != len(adc_gains) or len(adc_channels) != len(adc_dividers):
            self.get_logger().error("Length of adc_channels, adc_gains, and adc_dividers must match.")
            raise ValueError("Parameter length mismatch.")

        self.adc_channels = adc_channels
        self.adc_gains = adc_gains
        self.adc_dividers = adc_dividers

        # Gain mapping
        self.gain_map = {
            '2/3': 6.144,
            '1': 4.096,
            '2': 2.048,
            '4': 1.024,
            '8': 0.512,
            '16': 0.256
        }

        self.adc = Adafruit_ADS1x15.ADS1115(address=i2c_address, busnum=i2c_bus)

        # Create a publisher
        self.publisher = self.create_publisher(Voltage, topic_name, 10)

        # Create a timer to read and publish ADC values
        self.timer = self.create_timer(1.0, self.publish_adc_values)

    def publish_adc_values(self):
        for i, channel_name in enumerate(self.adc_channels):
            gain = self.adc_gains[i]
            divider = self.adc_dividers[i]

            if gain not in self.gain_map:
                self.get_logger().error(f"Invalid gain '{gain}' for channel {channel_name}. Skipping.")
                continue

            # Read raw ADC value and normalize to voltage
            raw_value = self.adc.read_adc(i, gain=int(gain))
            voltage_range = self.gain_map[gain]
            normalized_voltage = (raw_value / 32768.0) * voltage_range
            scaled_voltage = normalized_voltage * divider

            # Create and publish Voltage message
            msg = Voltage()
            msg.header = Header()
            msg.header.frame_id = channel_name
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = scaled_voltage

            self.publisher.publish(msg)

            self.get_logger().info(f"Published: channel={channel_name}, voltage={scaled_voltage:.3f} V")


def main(args=None):
    rclpy.init(args=args)
    node = ADS1115Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
