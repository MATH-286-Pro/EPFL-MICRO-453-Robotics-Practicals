import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetModelList, DeleteEntity

m  = 1
cm = 0.01
mm = 0.001

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('VelocityProcessor')

        self.dist_front = 10
        timer_period = 0.1  # seconds
        self.i = 0

    # 服务通信
        # 1) 创建服务客户端
        self.cli_get = self.create_client(GetModelList, '/get_model_list')
        self.cli_del = self.create_client(DeleteEntity, '/delete_entity')

        # 等待服务就绪
        self.get_logger().info('Waiting for /get_model_list service...')
        self.cli_get.wait_for_service()
        self.get_logger().info('Waiting for /delete_entity service...')
        self.cli_del.wait_for_service()

    # 话题通信
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_center_front = self.create_subscription(
            LaserScan,
            '/thymio_front_center_proximity_sensor',
            self.center_front_callback,
            10)

        self.sub_center_left = self.create_subscription(
            LaserScan,
            '/thymio_left_center_proximity_sensor',
            self.center_left_callback,
            10)

        self.sub_center_right = self.create_subscription(
            LaserScan,
            '/thymio_right_center_proximity_sensor',
            self.center_right_callback,
            10)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def delete(self):
        del_req = DeleteEntity.Request()
        del_req.name = 'thymio'
        future = self.cli_del.call_async(del_req)
        # 注册回调，服务返回后自动执行
        future.add_done_callback(self.delete_response_callback)

    def delete_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('Deleted model: thymio')
            else:
                self.get_logger().warn('删除失败: thymio')
        except Exception as e:
            self.get_logger().error(f'调用 delete_entity 服务出错: {e}')

    def center_front_callback(self, msg: LaserScan):
        self.dist_front = msg.ranges[0]
        # self.get_logger().info('Distance: {msg.ranges}')
      
    def center_left_callback(self, msg: LaserScan):
        self.dist_left = msg.ranges[0]
        # self.get_logger().info('Distance: {msg.ranges}')

    def center_right_callback(self, msg: LaserScan):
        self.dist_right = msg.ranges[0]
        # self.get_logger().info('Distance: {msg.ranges}')

    def timer_callback(self):
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.dist_front < 5*cm:
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
            self.delete()
        else:
            msg.linear.x = 0.1
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