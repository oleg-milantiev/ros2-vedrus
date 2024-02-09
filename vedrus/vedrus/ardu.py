'''
TODO:
- move from timer to serial-async
- move to C++
- ? one big or few minor missages ?
'''
import sys
import rclpy
import serial
from rclpy.node import Node

from std_msgs.msg import UInt8, UInt16, Float32, String
from sensor_msgs.msg import Temperature, RelativeHumidity
from vedrus_interfaces.msg import Motor, MotorMove


class VedrusArduNode(Node):
    buf = ''
    side = None
    start = None

    def move_motor(self, msg):
        buf = [ord('c')]
        buf.append(ord('b') if msg.breaking else ord('m'))

        if self.side == 'left':
            buf.append(ord('f') if msg.forward else ord('r'))
        else:
            buf.append(ord('r') if msg.forward else ord('f'))

        buf.append(msg.power)

        print(msg)
        self.ser.write(buf)

    def __init__(self):
        super().__init__('vedrus_ardu')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', rclpy.Parameter.Type.STRING),
            ]
        )

        self.ser = serial.Serial(self.get_parameter('device').get_parameter_value().string_value, 115200, rtscts=True)

        self.timer = self.create_timer(0.1, self.read_serial)
        self.timer  # prevent unused variable warning

    def read_serial(self):
        if self.ser.in_waiting > 0:
            # b'141,65,21.24,45.52,735.97,22,END\r\nVEDRUS,0,0,1,1,140,66,21.24,45.52,735.97,27,END\r\nVEDRUS,15,18,1,1,140,66,21.24,45.50,735.97,42,END\r\nVEDRUS,11,0,1,1,140,65,21.24,45.50,735.97,27,END\r\nVEDRUS,0,0,1,1,'
            self.buf += self.ser.read(self.ser.in_waiting).decode('utf-8')

            #['141,65,21.24,45.52,735.97,22,END', 'VEDRUS,312,299,1,1,141,66,21.29,45.39,736.10,162,END', 'VEDRUS,140,134,1,1,141,67,21.29,45.39,736.10,128,END', 'VEDRU']
            lines = self.buf.split('\r\n')
            div = []

            for line in lines:
                div = line.split(',')

                if len(div) == 12 and (div[0] == 'VEDR' or div[0] == 'VEDL') and div[ len(div) - 1 ] == 'END':
                    if self.side is None:
                        self.side = 'left' if div[0] == 'VEDL' else 'right'
                        self.start = div[0]

                        self.subscription = self.create_subscription(
                            MotorMove,
                            'vedrus/'+ self.side +'/move',
                            self.move_motor,
                            10)
                        self.subscription  # prevent unused variable warning

                        self.publisher_temperature = self.create_publisher(Temperature, 'vedrus/'+ self.side +'/temperature', 10)
                        self.publisher_humidity = self.create_publisher(RelativeHumidity, 'vedrus/'+ self.side +'/humidity', 10)
                        self.publisher_sonar1 = self.create_publisher(Float32, 'vedrus/'+ self.side +'/front/sonar', 10)
                        self.publisher_sonar2 = self.create_publisher(Float32, 'vedrus/'+ self.side +'/rear/sonar', 10)

                        self.publisher_motor_front = self.create_publisher(Motor, 'vedrus/'+ self.side +'/front/motor', 10)
                        self.publisher_motor_rear = self.create_publisher(Motor, 'vedrus/'+ self.side +'/rear/motor', 10)

                        self.publisher_raw = self.create_publisher(String, 'vedrus/'+ self.side +'/raw', 10)

                    if self.side is not None:
                        msg = String()
                        msg.data = line
                        self.publisher_raw.publish(msg)

                if self.side is not None and len(div) == 12 and div[0] == self.start and div[ len(div) - 1 ] == 'END':

                    try:
                        '''
                        1: current1       -> vedrus_interfaces/Motor->current
                                             vedrus/left/front/motor
                        2: current2       -> vedrus_interfaces/Motor->current
                                             vedrus/left/rear/motor
                        3: speed1         -> vedrus_interfaces/Motor->tick
                                             vedrus/left/front/motor
                        4: speed2         -> vedrus_interfaces/Motor->tick
                                             vedrus/left/rear/motor
                        5: sonar1         -> std_msgs/msg/Float32
                                             vedrus/left/front/sonar
                        6: sonar2         -> std_msgs/msg/Float32
                                             vedrus/left/rear/sonar
                        7: temperature    -> sensor_msgs/msg/Temperature
                                         vedrus/left/temperature
                        8: humidity       -> sensor_msgs/msg/RelativeHumidity
                                             vedrus/left/humidity
                        9: pressure_hi    TBD
                        10: current_sys   TBD
                        '''

                        msg = Temperature()
                        msg.temperature = float(div[7])
                        msg.variance = 0.
                        self.publisher_temperature.publish(msg)

                        msg = RelativeHumidity()
                        msg.relative_humidity = float(div[8])
                        msg.variance = 0.
                        self.publisher_humidity.publish(msg)

                        msg = Float32()
                        msg.data = float(div[5]) / 57.
                        self.publisher_sonar1.publish(msg)

                        msg = Float32()
                        msg.data = float(div[6]) / 57.
                        self.publisher_sonar2.publish(msg)

                        msg = Motor()
                        msg.tick = int(div[3])
                        msg.current = float(div[1])
                        self.publisher_motor_front.publish(msg)

                        msg = Motor()
                        msg.tick = int(div[4])
                        msg.current = float(div[2])
                        self.publisher_motor_rear.publish(msg)

                    except ValueError:
                        print('error')
                        pass

            self.buf = '' if self.side is not None and len(div) == 12 and div[0] == self.start and div[ len(div) - 1 ] == 'END' else lines[ len(lines) - 1 ]


def main(args=None):
    rclpy.init(args=args)

    vedrus_ardu_node = VedrusArduNode()
    rclpy.spin(vedrus_ardu_node)

    vedrus_ardu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
