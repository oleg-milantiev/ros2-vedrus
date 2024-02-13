import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from vedrus_interfaces.msg import MotorMove
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(Imu, '/vedrus/input/remote', self.callback, 10)
        self.left_motor_pub = self.create_publisher(MotorMove, '/vedrus/left/move', 10)
        self.right_motor_pub = self.create_publisher(MotorMove, '/vedrus/right/move', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Таймер на 1 секунду
        self.last_message_time = time.time()

    def callback(self, msg):
        forward_speed = min(max(msg.linear_acceleration.y, -10), 10)
        rotate_speed = min(max(msg.linear_acceleration.x, -10), 10)

        forward_speed = min(max(forward_speed * abs(forward_speed) * 0.5, -255), 255)
        rotate_speed = min(max(rotate_speed * abs(rotate_speed) * 0.5, -255), 255)

        left_power = forward_speed + rotate_speed
        right_power = forward_speed - rotate_speed

        left_move = MotorMove()
        left_move.breaking = False
        left_move.forward = True if left_power >= 0 else False
        left_move.power = abs(int(left_power)) * 3

        right_move = MotorMove()
        right_move.breaking = False
        right_move.forward = True if right_power >= 0 else False
        right_move.power = abs(int(right_power)) * 3

        self.left_motor_pub.publish(left_move)
        self.right_motor_pub.publish(right_move)

        self.last_message_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        if current_time - self.last_message_time >= 1:
            emegency_stop = MotorMove()
            emegency_stop.forward = True
            emegency_stop.power = 0
            self.left_motor_pub.publish(emegency_stop)
            self.right_motor_pub.publish(emegency_stop)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
