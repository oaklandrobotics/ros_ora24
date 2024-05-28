import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from odrive_can.msg import ControlMessage
import can

WHEEL_DIST = 28 / 39.37
WHEEL_DIA = 1 * 0.3048
WHEEL_RADIUS = WHEEL_DIA / 2

class WheelParser(Node):
    
    def __init__(self):
        super().__init__('wheel_parser')
        self.cmd_vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.right_motor_pub_raw = self.create_publisher(Float32, 'right_wheel_speed/raw', 10)
        self.left_motor_pub_raw = self.create_publisher(Float32, 'left_wheel_speed/raw', 10)
        self.right_motor_pub = self.create_publisher(ControlMessage, 'right_wheel_speed/odrive', 10)
        self.left_motor_pub = self.create_publisher(ControlMessage, 'left_wheel_speed/odrive', 10)
        self.get_logger().info('Node has been started.')
        
        # Initialize the CAN bus
        """ self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.get_logger().info('CAN bus has been initialized.') """
        
    def cmd_vel_callback(self, vel):
        linear = vel.linear.x
        angular = vel.angular.z
        
        # Calculate the desired speed for each wheel
        speed_wish_right = (1 / WHEEL_RADIUS) * (linear + (WHEEL_DIST * angular) / 2)
        speed_wish_left = (1 / WHEEL_RADIUS) * (linear - (WHEEL_DIST * angular) / 2)
        
        # Logging for debugging
        self.get_logger().info(f"Received cmd_vel: linear={linear}, angular={angular}")
        self.get_logger().info(f"Publishing wheel speeds: right={speed_wish_right}, left={speed_wish_left}")
        
        # Publish the raw desired speed for each wheel
        self.right_motor_pub_raw.publish(Float32(data=speed_wish_right))
        self.left_motor_pub_raw.publish(Float32(data=speed_wish_left))

        # Publish the desired speed for each wheel to the ODrive node
        # Right
        control_msg_right = ControlMessage()
        control_msg_right.control_mode = 2 # velocity control
        # TODO Verify this is the right input mode
        control_msg_right.input_mode = 1 # passthrough 
        control_msg_right.input_vel = speed_wish_right
        #control_msg_right.input_pos = 0 # do these need to be set?
        #control_msg_right.input_torque = 0

        # Left
        control_msg_left = ControlMessage()
        control_msg_left.control_mode = 2
        # TODO ditto above
        control_msg_left.input_mode = 1
        control_msg_left.input_vel = speed_wish_left
        
        self.right_motor_pub.publish(control_msg_right)
        self.left_motor_pub.publish(control_msg_left)

        # Send the desired speed to the motor controller
        #self.send_can_message(speed_wish_right, speed_wish_left)
        
    def send_can_message(self, speed_right, speed_left):
        # Create the CAN message
        can_id = 0x01 # change this to real arbitration ID
        data = self.create_can_data(speed_right, speed_left)
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        
        try:
            # Send the CAN message
            self.bus.send(msg)
            self.get_logger().info(f"Sent CAN message: ID={msg.arbitration_id}, data={msg.data}")
        except can.CanError:
            self.get_logger().error("Failed to send CAN message")

    def create_can_data(self, speed_right, speed_left):
        # Create byte array for the CAN message
        speed_right = int(speed_right * 1000)
        speed_left = int(speed_left * 1000)
        
        # Create byte array
        data = bytearray()
        data.extend(speed_right.to_bytes(2, byteorder='little', signed=True))
        data.extend(speed_left.to_bytes(2, byteorder='little', signed=True))
        return data

        
        
def main(args=None):
    rclpy.init(args=args)
    
    wheel_parser = WheelParser()
    
    rclpy.spin(wheel_parser)
    
    # Destroy
    wheel_parser.destroy_node()
    rclpy.shutdown()
        
    if __name__ == '__main__':
        main()