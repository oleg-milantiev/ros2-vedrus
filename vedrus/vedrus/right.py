import rclpy
from rclpy.node import Node

from vedrus_interfaces.msg import MotorMove


class VedrusLeftNode(Node):
    power = 0

    def __init__(self):
        super().__init__('vedrus_right_node')

        self.timer = self.create_timer(1, self.publish_motor)
        self.timer  # prevent unused variable warning

        self.publisher_ = self.create_publisher(MotorMove, 'vedrus/left/move', 10)

    def publish_motor(self):
        msg = MotorMove()
        msg.breaking = False
        msg.forward = True
        msg.power = self.power * 1 # WTF!!!!??? :)))

        self.power += 1
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    vedrus_left_node = VedrusLeftNode()
#    vedrus_left_node.read_serial()
    rclpy.spin(vedrus_left_node)

    vedrus_left_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
