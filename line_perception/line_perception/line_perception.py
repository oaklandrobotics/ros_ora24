import rclpy
from rclpy.node import Node

class LinePerception(Node):
    def __init__(self):
        super.__init__('line_perception')

def main():
    rclpy.init()
    
    line_perception = LinePerception()

    rclpy.spin(line_perception)

    line_perception.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
