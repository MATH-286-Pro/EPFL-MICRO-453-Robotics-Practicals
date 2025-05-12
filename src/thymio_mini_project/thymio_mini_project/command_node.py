import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

m  = 1
cm = 0.01
mm = 0.001

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('VelocityProcessor')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/thymio_front_center_proximity_sensor',
            self.sensor_callback,
            10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def sensor_callback(self, msg: LaserScan):
        self.dist_front = msg.ranges
        # self.get_logger().info('Distance: {msg.ranges}')


    def timer_callback(self):
        msg = Twist()
        if self.dist_front[0] < 5*cm:
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: {msg.linear.x}')
        self.i += 1
    


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()