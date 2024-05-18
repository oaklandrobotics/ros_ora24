import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class WheelParser(Node):
    WHEEL_DIST = 3
    
    def __init__(self):
        super().__init__('wheel_parser')
        self.cmd_vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.right_motor_pub = self.create_publisher(Float32, 'right_wheel_speed', 10)
        self.left_motor_pub = self.create_publisher(Float32, 'left_wheel_speed', 10)
        
    def cmd_vel_callback(self, vel):
        linear = vel.linear.x
        angular = vel.angular.z
        
        # Calculate the desired speed for each wheel
        speed_wish_right = linear + angular * self.WHEEL_DIST / 2
        speed_wish_left = linear - angular * self.WHEEL_DIST / 2
        
        # Publish the desired speed for each wheel
        self.right_motor_pub.publish(Float32(data=speed_wish_right))
        self.left_motor_pub.publish(Float32(data=speed_wish_left))
        
def main(args=None):
    rclpy.init(args=args)
    
    wheel_parser = WheelParser()
    
    rclpy.spin(wheel_parser)
    
    # Destroy
    wheel_parser.destroy_node()
    rclpy.shutdown()
        
    if __name__ == '__main__':
        main()