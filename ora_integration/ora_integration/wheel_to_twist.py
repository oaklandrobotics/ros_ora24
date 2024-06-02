import rclpy
import message_filters
from rclpy.node import Node
#from odrive_can.msg import ControllerStatus
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

WHEEL_DIST = 28 / 39.37
WHEEL_DIA = 1 * 0.3048
WHEEL_RADIUS = WHEEL_DIA / 2

class TwistParser(Node):
    def __init__(self):
        super().__init__('wheel_to_twist')
        self.wheel_r_sub = message_filters.Subscriber(self, Float32, 'wheels/right/encoder')
        self.wheel_l_sub = message_filters.Subscriber(self, Float32, 'wheels/left/encoder')
        self.twist_pub = self.create_publisher(TwistStamped, 'twist', 10)
        self.ts = message_filters.ApproximateTimeSynchronizer((self.wheel_r_sub, self.wheel_l_sub), 10, slop=0.1, allow_headerless=True) #is there a better way to do this?
        self.ts.registerCallback(self.twist_callback)
        self.get_logger().info('Hello from wheel_to_twist')

    def twist_callback(self, wheel_r: Float32, wheel_l: Float32):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        # Calculate linear and angular velocities
        linear = (WHEEL_RADIUS / 2) * (wheel_r.vel_estimate + wheel_l.vel_estimate)
        angular = (WHEEL_RADIUS / WHEEL_DIST) * (wheel_r.vel_estimate - wheel_l.vel_estimate)

        # Publish 'em
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        self.twist_pub.publish(twist)

def main():
    rclpy.init()
    
    twist_parser = TwistParser()

    rclpy.spin(twist_parser)

    twist_parser.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()