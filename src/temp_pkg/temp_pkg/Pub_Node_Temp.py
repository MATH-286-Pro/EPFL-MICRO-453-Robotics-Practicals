import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Publisher_Temperature')
        self.publisher_ = self.create_publisher(Int32, 'Temperature', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = random.randint(27, 43)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()