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

# Teloport Position
spawn_pos_list = [
    np.array([0.5, 0.3, 0.0, 0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]),
    np.array([1.3, 0.1, 0.0, 0, 0, np.sin(np.pi/2), np.cos(np.pi/2)])
]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('velocity_processor')

        # Dist Filter
        self.q_length = 3
        self.front_buf = deque(maxlen=self.q_length)  

        self.init_queue()

        self.dist_front = 100.0
        self.dist_left  = 100.0
        self.dist_right = 100.0

        self.yaw_start = 0.0

        self.turn = False
        self.turn_direction = 0

        # Current Pos Index
        self.state = 0

        # Sign for delete & spawn
        self.busy = False

        # 1) xacro → URDF string
        xacro_path = get_package_share_path('thymio_mini_project') / 'urdf' / 'thymio.urdf.xacro'
        self.robot_description_ = xacro.process_file(str(xacro_path)).toxml()

        # 2) Client：DeleteEntity、SpawnEntity
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

        # 100ms
        self.create_timer(0.1, self.timer_cb)

    def init_queue(self):
        for _ in range(3):
            self.front_buf.append(100.0)

    # --- LaserScan Callback ---
    def center_front_cb(self, msg: LaserScan):
        d = msg.ranges[0]
        self.front_buf.append(d)
        self.dist_front = float(np.median(self.front_buf))
        # self.dist_front = float(sum(self.front_buf)/len(self.front_buf))
    def center_left_cb(self, msg: LaserScan):
        self.dist_left = msg.ranges[0]
    def center_right_cb(self, msg: LaserScan):
        self.dist_right = msg.ranges[0]
    def yaw_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.yaw = np.rad2deg(yaw)
        # self.get_logger().info(f'Yaw: {np.rad2deg(yaw):.2f} degree')


    def robot_turn(self, angle: float):
        yaw_current = self.yaw
        yaw_stop    = self.yaw_start + angle

        # Regularization [-180, 180]
        yaw_diff = (yaw_stop - yaw_current + 180) % 360 - 180

        THRESHOLD = 5.0   
        Kp        = 0.03
        MAX_SPEED = 1.0    
        MIN_FIXED = 0.3  

        self.get_logger().info(f"yaw_diff={yaw_diff:.1f}°")

        if abs(yaw_diff) > 1.0:
            cmd = Twist()
            if abs(yaw_diff) > THRESHOLD:
                raw_speed = Kp * yaw_diff
                speed = max(-MAX_SPEED, min(MAX_SPEED, raw_speed))
            else:
                speed = np.sign(yaw_diff) * MIN_FIXED

            cmd.angular.z = speed
            self.pub.publish(cmd)
        else:
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
        

    def timer_cb(self):
        # if busy then jump
        if self.busy:
            return

        cmd = Twist()

        self.robot_check_turn_direction()

        # Not turning + no wall ahead
        #    Move forward

        # Not turning + wall ahead + turn_direction is not 0
        #    Set self.turn flag

        # Not turning + wall ahead + turn_direction is 0
        #    Teleport if no direction to turn

        # Turning
        #    Continue turning
        #    If completed, set turn_direction to 0
        #    If completed, set turn to False

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


    # --- Delete Entity ---
    def call_delete(self):
        req = DeleteEntity.Request()
        req.name = 'thymio'
        fut = self.cli_del.call_async(req)
        fut.add_done_callback(self.delete_response_callback)

    def delete_response_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f'Delete: {res.status_message}')
                self.busy = False
                return
            self.get_logger().info('Delete thymio Done')

            # Next Position（state+1）
            p = spawn_pos_list[self.state]
            self.call_spawn(p) #0000FF

        except Exception as e:
            self.get_logger().error(f'Delete_entity fail: {e}')
            self.busy = False

    # --- Spwan Entity ---
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

    def spawn_response_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f'Spwan fail: {res.status_message}')
            else:
                self.get_logger().info('Spwaen thymio Done')
                self.state += 1
                self.init_queue() #0000FF
                self.dist_front = 100.0

        except Exception as e:
            self.get_logger().error(f'spawn_entity fail: {e}')
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

