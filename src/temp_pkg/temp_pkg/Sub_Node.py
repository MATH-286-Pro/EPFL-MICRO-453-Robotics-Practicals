import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MinimalPublisher(Node):

    def __init__(self):
        # 用两个不同的属性保存订阅对象，避免互相覆盖
        super().__init__('compute_node')  

        self.temp_sub = self.create_subscription(
            Int32,
            'Temperature',
            self.Temp_callback,
            10)
        self.humi_sub = self.create_subscription(
            Int32,
            'Humidity',
            self.Humi_callback,
            10)

        # 初始时还没收到任何值，用 None 标记
        self.T = None
        self.H = None



    def Temp_callback(self, msg):
        self.T = msg.data
        self.compute_and_log()

    def Humi_callback(self, msg):
        self.H = msg.data
        self.compute_and_log()


    def compute_and_log(self):
        if self.T is None or self.H is None:
            return

        T = self.T
        H = self.H
        heat_index = (
            -42.379
            + 2.04901523 * T
            + 10.14333127 * H
            - 0.22475541 * T * H
            - 6.83783e-3 * T ** 2
            - 5.481717e-2 * H ** 2
            + 1.22874e-3 * T ** 2 * H
            + 8.5282e-4 * T * H ** 2
            - 1.99e-6 * T ** 2 * H ** 2
        )

        self.get_logger().info(
            f'Heat index computed from T={T:.1f} °C, H={H:.1f}% → {heat_index:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()