import rclpy
from rclpy.node import Node
import can
from can.interface import Bus
from can import Message
from can import Notifier
from can import BufferedReader

from std_msgs.msg import String
from std_msgs.msg import Bool

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'vcan0'

# Publisher: button, 1
# Listener: led state, 2

class Integration(Node):
    def __init__(self):
        super().__init__('ora_integration')
        self.button_publisher = self.create_publisher(Bool, 'btn', 10)
        self.led_subscriber = self.create_subscription(Bool, 'led', self.led_callback, 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.can_bus = Bus()

    # Read incoming data
    def timer_callback(self):
        while True:
            can_msg = self.can_bus.recv(timeout=0.01)

            if can_msg is None:
                # No more messages, break out of the loop
                break

            #Process
            if can_msg.arbitration_id == 1:
                self.handle_btn(can_msg.data)
            else:
                self.get_logger().warn(f'Uknown message: ID: {can_msg.arbitration_id}, {can_msg.data.hex(sep=" ", bytes_per_sep=1)}')

    def handle_btn(self, data: bytearray):
        msg = Bool()
        if len(data) >= 1:
            if data[0] == 1:
                msg.data = True
            elif data[0] == 0:
                msg.data = False
            else:
                self.get_logger().warn(f'Unknown LED message: {data.hex(sep=" ", bytes_per_sep=1)}')
            self.button_publisher.publish(msg)

    # Send outgoing data
    def led_callback(self, msg: Bool):
        if msg.data == True:
            led_can_msg = Message(arbitration_id=2, data=[1])
        elif msg.data == False:
            led_can_msg = Message(arbitration_id=2, data=[0])
        else:
            self.get_logger().fatal('bruh')
        self.can_bus.send(led_can_msg)

def main(args=None):
    rclpy.init(args=args)

    integration = Integration()

    rclpy.spin(integration)

    integration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()