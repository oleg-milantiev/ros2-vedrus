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

from std_msgs.msg import UInt8, UInt16, Float32
from sensor_msgs.msg import Temperature, RelativeHumidity
from vedrus_interfaces.msg import Motor, MotorMove


ser = serial.Serial('/dev/ttyUSB0', 115200, rtscts=True)

# todo move to argv
side = 'left'

class VedrusArduNode(Node):
    buf = ''
    count = 0

    def move_motor(self, msg):
        buf = [ord('c')]
        buf.append(ord('b') if msg.breaking else ord('m'))
        buf.append(ord('f') if msg.forward else ord('r'))
        buf.append(msg.power)

        print(msg)
#        print(buf)
        ser.write(buf)

    def __init__(self):
        super().__init__('vedrus_'+ side +'_node')

        self.subscription = self.create_subscription(
            MotorMove,
            'vedrus/'+ side +'/move',
            self.move_motor,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher
        self.publisher_temperature = self.create_publisher(Temperature, 'vedrus/'+ side +'/temperature', 10)
        self.publisher_humidity = self.create_publisher(RelativeHumidity, 'vedrus/'+ side +'/humidity', 10)
        self.publisher_sonar1 = self.create_publisher(Float32, 'vedrus/'+ side +'/front/sonar', 10)
        self.publisher_sonar2 = self.create_publisher(Float32, 'vedrus/'+ side +'/rear/sonar', 10)

        self.publisher_motor_front = self.create_publisher(Motor, 'vedrus/'+ side +'/front/motor', 10)
        self.publisher_motor_rear = self.create_publisher(Motor, 'vedrus/'+ side +'/rear/motor', 10)

        self.timer = self.create_timer(0.1, self.read_serial)
        self.timer  # prevent unused variable warning

    def read_serial(self):
        if ser.in_waiting > 0:
            # b'141,65,21.24,45.52,735.97,22,END\r\nVEDRUS,0,0,1,1,140,66,21.24,45.52,735.97,27,END\r\nVEDRUS,15,18,1,1,140,66,21.24,45.50,735.97,42,END\r\nVEDRUS,11,0,1,1,140,65,21.24,45.50,735.97,27,END\r\nVEDRUS,0,0,1,1,'
            self.buf += ser.read(ser.in_waiting).decode('utf-8')

            #['141,65,21.24,45.52,735.97,22,END', 'VEDRUS,312,299,1,1,141,66,21.29,45.39,736.10,162,END', 'VEDRUS,140,134,1,1,141,67,21.29,45.39,736.10,128,END', 'VEDRU']
            lines = self.buf.split('\r\n')
            div = []

            for line in lines:
                div = line.split(',')

                if len(div) == 12 and (div[0] == 'VEDL' or div[0] == 'VEDR') and div[ len(div) - 1 ] == 'END':

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
                        msg.data = float(div[5])
                        self.publisher_sonar1.publish(msg)

                        msg = Float32()
                        msg.data = float(div[6])
                        self.publisher_sonar2.publish(msg)

                        msg = Motor()
                        msg.tick = int(div[3])
                        msg.current = float(div[1])
                        self.publisher_motor_front.publish(msg)

                        msg = Motor()
                        msg.tick = int(div[4])
                        msg.current = float(div[2])
                        self.publisher_motor_rear.publish(msg)

                        #print('c='+ str(self.count) + ', lines='+ str(len(lines)))
                        #self.count += 1

                    except ValueError:
                        print('error')
                        pass

            self.buf = '' if len(div) == 12 and (div[0] == 'VEDL' or div[0] == 'VEDR') and div[ len(div) - 1 ] == 'END' else lines[ len(lines) - 1 ]


def main(args=None):
    rclpy.init(args=args)

    vedrus_ardu_node = VedrusArduNode()
    rclpy.spin(vedrus_ardu_node)

    vedrus_ardu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
