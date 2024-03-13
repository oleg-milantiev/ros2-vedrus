import rclpy
import getch
from rclpy.node import Node

from vedrus_interfaces.msg import MotorMove


class VedrusTestNode(Node):

    left = 0
    right = 0

    def __init__(self):
        super().__init__('vedrus_test')

        self.publisher_left = self.create_publisher(MotorMove, 'vedrus/left/move', 10)
        self.publisher_right = self.create_publisher(MotorMove, 'vedrus/right/move', 10)

        while (1):
            wasLeft = self.left
            wasRight = self.right

            match getch.getch():
                case 'q':
                    self.left += 5
                case 'a':
                    self.left -= 5
                case 'w':
                    self.right += 5
                case 's':
                    self.right -= 5

            if wasLeft != self.left:
                msg = MotorMove()
                msg.breaking = False
                msg.forward = self.left >= 0
                msg.power1 = abs(self.left) * 1 # WTF!!!!??? :)))
                msg.power2 = abs(self.left) * 1 # WTF!!!!??? :)))

                print('Left')
                print(msg)
                self.publisher_left.publish(msg)

            if wasRight != self.right:
                msg = MotorMove()
                msg.breaking = False
                msg.forward = self.right >= 0
                msg.power1 = abs(self.right) * 1 # WTF!!!!??? :)))
                msg.power2 = abs(self.right) * 1 # WTF!!!!??? :)))

                print('Right')
                print(msg)
                self.publisher_right.publish(msg)


    def publish_motor(self):
        msg = MotorMove()
        msg.breaking = False
        msg.forward = True
        msg.power = self.powers[self.power] * 1 # WTF!!!!??? :)))

        self.publisher_left.publish(msg)
        self.publisher_right.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    vedrus_test_node = VedrusTestNode()
    rclpy.spin(vedrus_test_node)

    vedrus_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
