import rclpy # type: ignore
from std_msgs.msg import Float32 # type: ignore
from sensor_msgs.msg import Imu # type: ignore
from vedrus_interfaces.msg import MotorCommand, MotorPID, Safety, KeepAlive, Status, StatusItem

class ModeParent():
    node = None
    
    def __init__(self, node):
        self.node = node
        pass

    def _statusInit(self):
        self.status = Status()
        self.status.header.stamp = self.node.get_clock().now().to_msg()
        self.status.header.frame_id = 'controller'

    def _statusAppend(self, name, value):
        statusItem = StatusItem()
        statusItem.name = name
        statusItem.value = value
        self.status.items.append(statusItem)

    def _statusPublish(self):
        self.node.publisherStatus.publish(self.status)

    # Углы 1 и 2 отличаются не больше gap
    def _anglesMatch(self, angle1, angle2, gap):
        diff = abs(angle1 - angle2)
        return (diff <= gap or (360 - diff) <= gap)
