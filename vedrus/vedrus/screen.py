import rclpy
from rclpy.node import Node
from vedrus_interfaces.msg import Motor, Sonar
from textual import events
from textual.app import App
from textual.widgets import Button

class VedrusScreenNode(Node):

	def __init__(self):
		super().__init__('vedrus_screen_node')
		self.create_subscription(Motor, '/vedrus/motor', self.motor_callback, 10)

	def motor_callback(self, msg):
		if data.header.frame_id == 'left_front':
			self.plot.add_point(data.power)
			self.app.update()

class VedrusTuiApp(App):

	async def on_start(self):
		self.plot = Plot()
		self.add_renderable(self.plot)

def main(args=None):
	rclpy.init(args=args)

	node = VedrusScreenNode()
	app = VedrusTuiApp()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
