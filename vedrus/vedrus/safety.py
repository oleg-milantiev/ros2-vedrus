import rclpy
from rclpy.node import Node
from vedrus_interfaces.msg import Safety

class SafetyNode(Node):
	def __init__(self):
		super().__init__('safety_node')

def main(args=None):
	rclpy.init(args=args)
	node = SafetyNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
