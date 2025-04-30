import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__("node_name")
        self.get_logger().info("Minimal Node has been started")

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()
#    rclpy.spin(minimal_node)
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
