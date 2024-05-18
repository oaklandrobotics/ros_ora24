import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import can

class WheelParser(Node):
    WHEEL_DIST = 3 #something
    
    def __init__(self):
        super().__init__('wheel_parser')
        self.cmd_vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.right_motor_pub = self.create_publisher(Float32, 'right_wheel_speed', 10)
        self.left_motor_pub = self.create_publisher(Float32, 'left_wheel_speed', 10)
        self.get_logger().info('Node has been started.')
        
        # Initialize the CAN bus
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.get_logger().info('CAN bus has been initialized.')
        
    def cmd_vel_callback(self, vel):
        linear = vel.linear.x
        angular = vel.angular.z
        
        # Calculate the desired speed for each wheel
        speed_wish_right = linear + angular * self.WHEEL_DIST / 2
        speed_wish_left = linear - angular * self.WHEEL_DIST / 2
        
        # Logging for debugging
        self.get_logger().info(f"Received cmd_vel: linear={linear}, angular={angular}")
        self.get_logger().info(f"Publishing wheel speeds: right={speed_wish_right}, left={speed_wish_left}")
        
        # Publish the desired speed for each wheel
        self.right_motor_pub.publish(Float32(data=speed_wish_right))
        self.left_motor_pub.publish(Float32(data=speed_wish_left))
        
        # Send the desired speed to the motor controller
        self.send_can_message(speed_wish_right, speed_wish_left)
        
def send_can_message(self, speed_right, speed_left):
    # Create the CAN message
    can_id = 0x01 # change this to real arbitration ID
    data = self.create_can_data(speed_right, speed_left)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    
    try:
        # Send the CAN message
        self.bus.send(msg)
        self.get_logger().info("Sent CAN message: ID=%s, data=%s", msg.arbitration_id, msg.data)
    except can.CanError:
        self.get_logger().error("Failed to send CAN message")
        
def main(args=None):
    rclpy.init(args=args)
    
    wheel_parser = WheelParser()
    
    rclpy.spin(wheel_parser)
    
    # Destroy
    wheel_parser.destroy_node()
    rclpy.shutdown()
        
    if __name__ == '__main__':
        main()