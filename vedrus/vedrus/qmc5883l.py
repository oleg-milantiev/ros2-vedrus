import py_qmc5883l

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class MagNode(Node):
	def __init__(self):
		super().__init__('mag_node')
		self.publisher = self.create_publisher(Float32, '/imu/bearing', 10)

		self.sensor = py_qmc5883l.QMC5883L(i2c_bus=2, output_data_rate=0b00000000) # 10 hz
		#self.sensor = py_qmc5883l.QMC5883L(i2c_bus=2, output_data_rate=0b00000100) # 50 hz
		self.sensor.calibration = [[1.0313066562489555, -0.031047791919103184, 863.5276384091891], [-0.031047791919103188, 1.030791068052312, 1322.8836301489387], [0.0, 0.0, 1.0]]
		self.sensor.set_declination(7.1465)

		while rclpy.ok():
			msg = Float32()
			msg.data = self.sensor.get_bearing()
			self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = MagNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()