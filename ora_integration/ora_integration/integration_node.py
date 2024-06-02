import rclpy, can, struct

from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from can.interface import Bus
from can import Message

# interface properties
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'

# CAN Messages
# 0 = Unused (Reserved for ODrive)
# 1 = Unused (Reserved for ODrive)
# 2 = Desired Right Wheel Speed (Sub)
# 3 = Desired Left Wheel Speed (Sub)
# 4 = Encoder Right Wheel Speed (Pub)
# 5 = Encoder Left Wheel Speed (Pub)
# 6 = Autonomous State (Pub)
# 7 = E-stop (Pub)

class Integration(Node):
    def __init__(self):
        super().__init__('ora_integration')

        #CAN Bus
        self.can_bus = Bus()

        #Subscribers (Topic -> CAN)
        self.wheel_l_sub = self.create_subscription(Float32, 'wheels/left/raw', lambda msg: self.handle_wheel_desired(msg, 'left'), 10)        #2
        self.wheel_r_sub = self.create_subscription(Float32, 'wheels/right/raw', lambda msg: self.handle_wheel_desired(msg, 'right'), 10)      #3

        #Publishers (CAN -> Topic)
        self.wheel_l_pub = self.create_publisher(Float32, 'wheels/left/encoder', 10)        #4
        self.wheel_r_pub = self.create_publisher(Float32, 'wheels/right/encoder', 10)       #5
        self.auton_mode = self.create_publisher(Bool, 'auton_mode', 10)                     #6
        self.estop = self.create_publisher(Bool, 'estop', 10)                               #7

        #Timer
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('CAN Node Started.')

    # Read incoming messages
    def timer_callback(self):
        while True:
            can_msg = self.can_bus.recv(timeout=0.01)

            if can_msg is None:
                # No more messages, break out of the loop
                break
            
            self.get_logger().info(f'Received message wtih ID {can_msg.arbitration_id}')

            #Process incoming messages
            match can_msg.arbitration_id:
                case 4:
                    #handle right wheel
                    self.handle_wheel_encoder(can_msg.data, 'right')
                case 5:
                    #handle left wheel
                    self.handle_wheel_encoder(can_msg.data, 'left')
                case 6:
                    #handle auto mode
                    self.handle_auton_mode(can_msg.data)
                case 7:
                    #handle estop
                    self.handle_estop(can_msg.data)
                case _:
                    self.get_logger().warn(f'Unknown message: ID: {can_msg.arbitration_id}, {can_msg.data.hex(sep=" ", bytes_per_sep=1)}')

    def handle_wheel_encoder(self, data: bytearray, wheel: str):
        if not (wheel == 'left' or wheel == 'right'):
            self.get_logger().warn('Invalid wheel received')
            return
        
        ros_float = Float32()
        value = struct.unpack('d', data)
        ros_float.data = value
        if wheel == 'left':
            self.wheel_l_pub.publish(ros_float)
        elif wheel == 'right':
            self.wheel_r_pub.publish(ros_float)

    def handle_auton_mode(self, data: bytearray):
        msg = Bool()
        if data[0] == 1:
            msg.data = True
        elif data[0] == 0:
            msg.data = False
        
        self.auton_mode.publish(msg)

    def handle_estop(self, data: bytearray):
        msg = Bool()
        if data[0] == 1:
            msg.data = True
        elif data[0] == 0:
            msg.data = False
        
        self.estop.publish(msg)

    # Send outgoing messages
    def handle_wheel_desired(self, msg: Float32, wheel: str):
        if not (wheel == 'left' or wheel == 'right'):
            self.get_logger().warn('Invalid wheel received')
            return
        
        value = msg.data
        can_bytes = bytearray(struct.pack('d', value))
        if wheel == 'left':
            wheel_can_msg = Message(arbitration_id=2, data=can_bytes)
        elif wheel == 'right':
            wheel_can_msg = Message(arbitration_id=3, data=can_bytes)
        #self.get_logger().info(str(wheel_can_msg))
        self.can_bus.send(wheel_can_msg)

def main(args=None):
    rclpy.init(args=args)

    integration = Integration()

    rclpy.spin(integration)

    integration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()