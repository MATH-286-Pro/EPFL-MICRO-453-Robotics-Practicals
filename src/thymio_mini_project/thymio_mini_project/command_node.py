import debugpy
import rclpy
import xacro
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from ament_index_python.packages import get_package_share_path

debugpy.listen(('localhost', 5678))
debugpy.wait_for_client()

m  = 1
cm = 0.01
mm = 0.001

# 两个预设的位姿
spawn_pos_list = [
    np.array([0.5, 0.3, 0.0, 0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]),
    np.array([1.3, 0.1, 0.0, 0, 0, np.sin(np.pi/2), np.cos(np.pi/2)])
]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('velocity_processor')

        # 避障用距离读数
        self.dist_front = 10.0
        self.dist_left  = 10.0
        self.dist_right = 10.0

        # 当前已生成的序号：初始 0
        self.state = 0
        # 标志：正在删除/生成，就不再触发新的删除
        self.busy = False

        # 1) 预处理 xacro → URDF string
        xacro_path = get_package_share_path('thymio_mini_project') / 'urdf' / 'thymio.urdf.xacro'
        self.robot_description_ = xacro.process_file(str(xacro_path)).toxml()

        # 2) 客户端：DeleteEntity、SpawnEntity
        self.cli_del   = self.create_client(DeleteEntity, '/delete_entity')
        self.cli_spawn = self.create_client(SpawnEntity,  '/spawn_entity')
        self.get_logger().info('Waiting for delete_entity service...')
        self.cli_del.wait_for_service()
        self.get_logger().info('Waiting for spawn_entity service...')
        self.cli_spawn.wait_for_service()

        # 3) Publisher + Subscriber + Timer
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, '/thymio_front_center_proximity_sensor',
                                 self.center_front_cb, 10)
        self.create_subscription(LaserScan, '/thymio_left_center_proximity_sensor',
                                 self.center_left_cb, 10)
        self.create_subscription(LaserScan, '/thymio_right_center_proximity_sensor',
                                 self.center_right_cb, 10)

        # 100ms 更新一次
        self.create_timer(0.1, self.timer_cb)

    # --- LaserScan 回调 ---
    def center_front_cb(self, msg: LaserScan):
        self.dist_front = msg.ranges[0]
    def center_left_cb(self, msg: LaserScan):
        self.dist_left = msg.ranges[0]
    def center_right_cb(self, msg: LaserScan):
        self.dist_right = msg.ranges[0]

    # --- 定时避障 & 触发删除/生成 ---
    def timer_cb(self):
        # 如果正在处理删除/生成，就跳过
        if self.busy:
            return

        # 构建速度消息
        cmd = Twist()
        # 简单左右避障
        if self.dist_left > self.dist_right:
            cmd.angular.z = +1.0
        elif self.dist_left < self.dist_right:
            cmd.angular.z = -1.0

        # 前方过近，就停车并触发删除/生成
        if self.dist_front < 5*cm and self.state < len(spawn_pos_list):
            cmd.linear.x = 0.0
            self.pub.publish(cmd)

            # 标记为“忙”，然后开始删除
            self.busy = True
            self.call_delete()
        else:
            # 正常前进
            cmd.linear.x = 0.1
            self.pub.publish(cmd)

    # --- 发起删除 ---
    def call_delete(self):
        req = DeleteEntity.Request()
        req.name = 'thymio'
        fut = self.cli_del.call_async(req)
        fut.add_done_callback(self.delete_response_callback)

    # --- 删除响应后直接发起生成 ---
    def delete_response_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f'删除失败: {res.status_message}')
                # 出错也要释放 busy，否则就永远卡死
                self.busy = False
                return
            self.get_logger().info('已删除实体 thymio，开始生成下一个')

            # 生成下一个位置（state+1）
            p = spawn_pos_list[self.state]
            self.call_spawn(p)
        except Exception as e:
            self.get_logger().error(f'调用 delete_entity 失败: {e}')
            self.busy = False

    # --- 发起生成 ---
    def call_spawn(self, xyzw):
        req = SpawnEntity.Request()
        req.name            = 'thymio'
        req.xml             = self.robot_description_
        req.robot_namespace = ''
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyzw[0], xyzw[1], xyzw[2]
        pose.orientation.x, pose.orientation.y = xyzw[3], xyzw[4]
        pose.orientation.z, pose.orientation.w = xyzw[5], xyzw[6]
        req.initial_pose = pose

        fut = self.cli_spawn.call_async(req)
        fut.add_done_callback(self.spawn_response_callback)

    # --- 生成响应后更新 state & 释放 busy ---
    def spawn_response_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f'生成失败: {res.status_message}')
            else:
                self.get_logger().info('已生成新实体 thymio')
                self.state += 1
        except Exception as e:
            self.get_logger().error(f'调用 spawn_entity 失败: {e}')
        # 无论成功还是失败，都把 busy 释放，让 timer 继续正常工作
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

