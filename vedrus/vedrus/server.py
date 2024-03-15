import asyncio
from aiohttp import web
import os
import rclpy
from rclpy.node import Node
import threading
from vedrus_interfaces.msg import Safety, KeepAlive


'''
TODO
- move to websockets
'''

class ServerNode(Node):
	def web_homepage(self, request):
		with open('/opt/ros/iron/src/vedrus/html/index.html', 'r') as file:
			return web.Response(text=file.read(), content_type='text/html')

	async def web_status(self, request):
		data = {
			'safety': self.safety,
		}
		return web.json_response(data)

	async def web_control(self, request):
		data = await request.json()
		# Handle the POST request data
		return web.Response(text='OK')

	def aiohttp_server(self):
		app = web.Application()
		app.add_routes([web.get('/', self.web_homepage)])
		app.router.add_get('/status', self.web_status)
		app.router.add_post('/control', self.web_control)

		runner = web.AppRunner(app)
		return runner

	def run_server(self, runner):
		self.loop = asyncio.new_event_loop()
		asyncio.set_event_loop(self.loop)
		self.loop.run_until_complete(runner.setup())
		site = web.TCPSite(runner, '0.0.0.0', 8080)
		self.loop.run_until_complete(site.start())
		self.loop.run_forever()

	def __init__(self):
		super().__init__('vedrus_server_node')

		self.web_thread = threading.Thread(target=self.run_server, args=(self.aiohttp_server(),))
		self.web_thread.start()

		self.create_subscription(
			Safety,
			'/vedrus/safety',
			self.safety_callback,
			10)

	safety = []

	def safety_callback(self, msg):
		time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

		self.safety.append({
			'time': time,
			'id': msg.header.frame_id,
			#'azimuth': msg.azimuth - 360. if msg.azimuth > 180. else msg.azimuth, # -180 ... +180
			'azimuth': msg.azimuth,
			'range': msg.range,
		})

		self.safety = [item for item in self.safety if item['time'] >= (time - 1)]

def main(args=None):
	rclpy.init(args=args)
	server_node = ServerNode()

	server_node.get_logger().info('Starting...')

	try:
		rclpy.spin(server_node)
	except KeyboardInterrupt:
		pass

	server_node.loop.stop()
	server_node.get_logger().info('Shutting down...')
	rclpy.shutdown()

if __name__ == '__main__':
	main()
