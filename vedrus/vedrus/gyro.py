import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class AccelNode(Node):
    def __init__(self):
        super().__init__('accel_node')
        self.publisher = self.create_publisher(Imu, '/vedrus/input/remote', 10)

        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        self.accel_data = Imu()

        while rclpy.ok():
            div = self.serial_port.readline().decode().strip().split(',')
            if len(div) == 4 and div[0] == 'GYRO' and div[3] == 'END':
                self.accel_data.linear_acceleration.x = float(div[1])
                self.accel_data.linear_acceleration.y = float(div[2])
                self.publisher.publish(self.accel_data)

def main(args=None):
    rclpy.init(args=args)
    node = AccelNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()