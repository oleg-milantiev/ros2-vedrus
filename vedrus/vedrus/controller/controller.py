import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float32  # type: ignore
from sensor_msgs.msg import Imu  # type: ignore
from vedrus_interfaces.msg import MotorCommand, MotorStatus, Safety, KeepAlive, Status, StatusItem
import time

from .modes.parent import ModeParent
from .modes.rotate_to_person import ModeRotateToPerson
from .modes.forward_slow import ModeForwardSlow
from .modes.safety_stop import ModeSafetyStop
from .modes.move_fixed import ModeMoveFixed

DEBUG = True
CSV = False
RGBD_FOV = 92.2

class VedrusControlerNode(Node):
    crash = False

    # текущий режим
    mode = None

    # Управление моторами
    publisherMotor = None

    # время последнего keepalive Safety
    keepaliveSafety = None

    # время последнего keepalive для моторов и bearing
    keepaliveMotorLeft = None
    keepaliveMotorRight = None
    keepaliveBearing = None

    leftSpeed = None
    def left(self, speed):
        #if DEBUG:
        #	self.get_logger().info('VedrusControlerNode::left: set speed to '+ str(speed))

        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'left'
        msg.speed = speed
        self.publisherMotor.publish(msg)

        self.leftSpeed = speed

        return

    rightSpeed = None
    def right(self, speed):
        #if DEBUG:
        #	self.get_logger().info('VedrusControlerNode::right: set speed to '+ str(speed))

        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'right'
        msg.speed = speed
        self.publisherMotor.publish(msg)

        self.rightSpeed = speed

        return

    def __init__(self):
        super().__init__('vedrus_control_node')

        self.publisherStatus = self.create_publisher(Status, '/vedrus/status', 10)
        self.publisherMotor = self.create_publisher(MotorCommand, '/vedrus/motor/command', 10)

        # magnetic bearing
        self.create_subscription(Float32, '/imu/bearing', self.bearing_callback, 10)

        # motors data
        self.create_subscription(MotorStatus, '/vedrus/motor/status', self.motors_callback, 10)

        # keepalive
        self.publisher_keepalive = self.create_publisher(KeepAlive, '/vedrus/keepalive/controller', 10)
        self.create_timer(0.33333333, self.timer_keepalive)
        self.create_subscription(KeepAlive, '/vedrus/keepalive/safety', self.safety_keepalive_callback, 10)

        # в таймере выполняю задачу
        self.create_timer(0.2, self.task_safety_short_forward_reverse)
        #self.create_timer(0.2, self.task_safety_forward)
        #self.create_timer(0.2, self.task_stay_and_rotate_to_person)
        #self.create_timer(0.2, self.task_find_person_rotate_and_follow)

        self.create_subscription(Safety, '/vedrus/safety', self.safety_callback, 10)

        if DEBUG:
            self.get_logger().info('VedrusControlerNode is started')


    bearing = None
    def bearing_callback(self, msg):
        self.bearing = msg.data
        self.keepaliveBearing = time.time()


    motorLeft = None
    motorRight = None
    def motors_callback(self, msg):
        if msg.header.frame_id == 'left':
            self.motorLeft = msg
            self.keepaliveMotorLeft = time.time()
        else:
            self.motorRight = msg
            self.keepaliveMotorRight = time.time()


    # safety
    def safety_callback(self, msg):
        if msg.warning:
            if self.mode is not None:
                self.mode.warning(msg)
        if msg.alarm:
            if self.mode is not None:
                self.mode.alarm(msg)


    def __canWork(self):
        current_time = time.time()
        return (self.keepaliveSafety is not None and current_time - self.keepaliveSafety < 2 and
                self.keepaliveMotorLeft is not None and current_time - self.keepaliveMotorLeft < 2 and
                self.keepaliveMotorRight is not None and current_time - self.keepaliveMotorRight < 2 and
                self.keepaliveBearing is not None and current_time - self.keepaliveBearing < 2)

    # tasks
    def task_safety_short_forward_reverse(self):
        if not self.__canWork():
            if DEBUG:
                self.get_logger().info('VedrusControlerNode: task_safety_short_forward_reverse: waiting for all keepalives')
            return

        # initial mode and its parameters (here: routing)
        if self.mode is None:
            if DEBUG:
                self.get_logger().info('VedrusControlerNode: task_safety_short_forward_reverse: Go to ModeSafetyStop. Will route to ModeMoveFixed +20k')
            self.mode = ModeSafetyStop(self)
            self.mode.out = ModeMoveFixed(self)
            self.mode.out.incLeft = 20000
            self.mode.out.incRight = 20000
            self.mode.out.taskStage = 1

        # loop routing +20k, -20k, +, -, ...
        if isinstance(self.mode, ModeMoveFixed) and self.mode.out == None:
            if DEBUG:
                self.get_logger().info(f"VedrusControlerNode: task_safety_short_forward_reverse: Set next mode: ModeMoveFixed: {-self.mode.incLeft}")
            self.mode.out = ModeMoveFixed(self)
            self.mode.out.taskStage = -self.mode.taskStage
            self.mode.out.incLeft = -self.mode.taskStage * 20000
            self.mode.out.incRight = -self.mode.taskStage * 20000

        # Execute the mode cycle. The mode will decide the next mode
        self.mode = self.mode.cycle()

    def task_safety_forward(self):
        if not self.__canWork():
            if DEBUG:
                self.get_logger().info('VedrusControlerNode: waiting for all keepalives')
            return

        # initial mode and its parameters (here: routing)
        if self.mode is None:
            self.mode = ModeSafetyStop(self)
            self.mode.out = ModeForwardSlow(self)

        # Execute the mode cycle. The mode will decide the next mode
        self.mode = self.mode.cycle()

    def task_stay_and_rotate_to_person(self):
        if not self.__canWork():
            if DEBUG:
                self.get_logger().info('VedrusControlerNode: waiting for all keepalives')
            return

        # initial mode and its parameters (here: routing)
        if self.mode is None:
            self.mode = ModeSafetyStop(self)
            self.mode.out = ModeRotateToPerson(self)
            self.mode.out.forward = False

        # Execute the mode cycle. The mode will decide the next mode
        self.mode = self.mode.cycle()

    def task_find_person_rotate_and_follow(self):
        if not self.__canWork():
            if DEBUG:
                self.get_logger().info('VedrusControlerNode: waiting for all keepalives')
            return

        # initial mode and its parameters (here: routing)
        if self.mode is None:
            self.mode = ModeSafetyStop(self)
            self.mode.out = ModeRotateToPerson(self)
            self.mode.out.forward = True

        # Execute the mode cycle. The mode will decide the next mode
        self.mode = self.mode.cycle()


    # keepalive
    def safety_keepalive_callback(self, data):
        self.keepaliveSafety = time.time()

    def _handle_crash(self):
        self.crash = True

        for frame_id in ['left', 'right']:
            msg = MotorCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.speed = 0
            msg.crash = True
            self.publisherMotor.publish(msg)

    def timer_keepalive(self):
        if self.crash:
            return

        # Send my keepalive
        msg = KeepAlive()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'controller'
        self.publisher_keepalive.publish(msg)

        # Check all keepalive properties
        current_time = time.time()
        if self.keepaliveSafety is not None and current_time - self.keepaliveSafety >= 2:
            self.get_logger().info('Got safety keepalive timeout!')
            self._handle_crash()
            return
        if self.keepaliveMotorLeft is not None and current_time - self.keepaliveMotorLeft >= 2:
            self.get_logger().info('Got left motor keepalive timeout!')
            self._handle_crash()
            return
        if self.keepaliveMotorRight is not None and current_time - self.keepaliveMotorRight >= 2:
            self.get_logger().info('Got right motor keepalive timeout!')
            self._handle_crash()
            return
        if self.keepaliveBearing is not None and current_time - self.keepaliveBearing >= 2:
            self.get_logger().info('Got bearing keepalive timeout!')
            self._handle_crash()
            return
            
def main(args=None):
    global node

    rclpy.init(args=args)
    node = VedrusControlerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
