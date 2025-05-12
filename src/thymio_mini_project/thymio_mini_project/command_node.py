import debugpy
import rclpy
import xacro
import numpy as np
from rclpy.node import Node
from collections import deque

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from ament_index_python.packages import get_package_share_path

from tf_transformations import euler_from_quaternion



# debugpy.listen(('localhost', 5678))
# debugpy.wait_for_client()

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
        self.q_length = 3
        self.front_buf = deque(maxlen=self.q_length)  

        self.init_queue()

        self.dist_front = 100.0
        self.dist_left  = 100.0
        self.dist_right = 100.0

        self.yaw_start = 0.0

        self.turn = False
        self.turn_direction = 0

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
        self.create_subscription(LaserScan, '/thymio_front_center_proximity_sensor', self.center_front_cb, 10)
        self.create_subscription(LaserScan, '/thymio_front_left_proximity_sensor',  self.center_left_cb,  10)
        self.create_subscription(LaserScan, '/thymio_front_right_proximity_sensor', self.center_right_cb, 10)
        self.create_subscription(Odometry,  '/odom', self.yaw_callback, 10)

        # 100ms 更新一次
        self.create_timer(0.1, self.timer_cb)

    def init_queue(self):
        for _ in range(3):
            self.front_buf.append(100.0)

    # --- LaserScan 回调 ---
    def center_front_cb(self, msg: LaserScan):
        # 读取第一个射线
        d = msg.ranges[0]
        self.front_buf.append(d)
        self.dist_front = float(np.median(self.front_buf))
        # self.dist_front = float(sum(self.front_buf)/len(self.front_buf))
    def center_left_cb(self, msg: LaserScan):
        self.dist_left = msg.ranges[0]
    def center_right_cb(self, msg: LaserScan):
        self.dist_right = msg.ranges[0]
    def yaw_callback(self, msg: Odometry):
        # 解析四元数
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.yaw = np.rad2deg(yaw)
        # self.get_logger().info(f'Yaw: {np.rad2deg(yaw):.2f} degree')


    def robot_turn(self, angle: float):
        # 当前和目标都是度
        yaw_current = self.yaw
        yaw_stop    = self.yaw_start + angle

        # 先规约到 [-180, 180]
        yaw_diff = (yaw_stop - yaw_current + 180) % 360 - 180

        # 参数：  
        THRESHOLD = 5.0   # 误差阈值 30°
        Kp        = 0.03   # 比例系数，0.01 * 90° = 0.9 rad/s → 约 51°/s
        MAX_SPEED = 1.0    # 最大角速度（rad/s 或 deg/s，看你用哪种单位）
        MIN_FIXED = 0.3    # 当误差小于阈值时的固定速度

        self.get_logger().info(f"yaw_diff={yaw_diff:.1f}°")

        if abs(yaw_diff) > 1.0:  # 大于 1° 还没到位
            cmd = Twist()
            if abs(yaw_diff) > THRESHOLD:
                # 比例控制（并限制最大值）
                raw_speed = Kp * yaw_diff
                # 限幅
                speed = max(-MAX_SPEED, min(MAX_SPEED, raw_speed))
            else:
                # 误差较小，用固定小速度细调
                speed = np.sign(yaw_diff) * MIN_FIXED

            cmd.angular.z = speed
            self.pub.publish(cmd)
        else:
            # |误差| ≤ 1°：停止
            cmd = Twist()
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self.turn = False
            self.turn_direction = 0
    
    def robot_check_turn_direction(self):
        if self.turn_direction != 0:
            return 0
        
        if self.dist_left == np.inf:
            self.turn_direction = +90
            return 1
        elif self.dist_right == np.inf:
            self.turn_direction = -90
            return 1
        else:
            self.turn_direction = 0
            return 0 
        

    # --- 定时避障 & 触发删除/生成 ---
    def timer_cb(self):
        # 如果正在处理删除/生成，就跳过
        if self.busy:
            return

        # 构建速度消息
        cmd = Twist()

        self.robot_check_turn_direction()

        # 没开始转弯 + 前面不是墙 
        #    前进

        # 没开始转弯 + 前面是墙  + turn_direction 不等于 0
        #    self.turn 标志位

        # 没开始转弯 + 前面是墙 + turn_direction = 0
        #    无则传送

        # 开始转弯
        #    继续转弯
        #    如果完成，将 turn_direction 置为 0
        #    如果完成，将 turn 置为 False

        if self.turn == False and self.dist_front > 12.0*cm:
            cmd.linear.x = 0.2
            self.pub.publish(cmd)

        elif self.turn == False and self.dist_front > 5*cm and self.dist_front < 8.0*cm:
            cmd.linear.x = 0.1
            self.pub.publish(cmd)

        elif self.turn == False and self.dist_front < 5*cm and self.turn_direction != 0:
            self.yaw_start = self.yaw
            self.turn      = True

        elif self.turn == False and self.dist_front < 5*cm and self.turn_direction == 0:
            self.busy = True
            self.call_delete()

        elif self.turn == True:
            self.robot_turn(self.turn_direction)


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
            self.call_spawn(p) #0000FF

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
                self.init_queue() #0000FF
                self.dist_front = 100.0

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

