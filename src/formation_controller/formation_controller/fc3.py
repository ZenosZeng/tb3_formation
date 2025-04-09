import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import numpy as np

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        
        # 配置参数
        self.leader_id = '1'    # 领航者机器人ID
        self.follower_ids = ['2', '3', '4']  # 跟随者机器人ID列表
        self.namespace = 'TB3'  # 机器人命名空间
        
        # 编队形状定义（相对于领航者的偏移量[x, y]）
        d = 1
        self.formation_offsets = {
            '2': [-d, 0],   # TB3_2
            '3': [-d, -d],  # TB3_3
            '4': [-d, d]    # TB3_4
        }
        
        # 初始化存储机器人位置和朝向的字典
        self.positions = {f'{self.namespace}_{id}': None for id in [self.leader_id] + self.follower_ids}
        self.orientations = {f'{self.namespace}_{id}': None for id in [self.leader_id] + self.follower_ids}
        
        # 订阅所有机器人的里程计信息
        self.odom_subs = []
        for robot_id in [self.leader_id] + self.follower_ids:
            topic_name = f'/{self.namespace}_{robot_id}/odom'
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    topic_name,
                    lambda msg, robot_id=robot_id: self.odom_callback(msg, robot_id),
                    10
                )
            )
        
        # 创建所有机器人的速度指令发布器
        self.cmd_publishers = {
            robot_id: self.create_publisher(
                Twist,
                f'/{self.namespace}_{robot_id}/cmd_vel',
                10
            )
            for robot_id in [self.leader_id] + self.follower_ids
        }
        
        # 设置控制循环的定时器（10Hz）
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 设置sin曲线的参数
        self.A = 1  # 曲线幅度
        self.k = 1  # 曲线频率
        self.linear_speed = 0.2  # 领航者的前进速度（m/s）
        self.start_time = time.time()  # 记录启动时间

    def odom_callback(self, msg, robot_id):
        """里程计回调函数，存储机器人位置信息和朝向信息"""
        position = msg.pose.pose.position
        self.positions[f'{self.namespace}_{robot_id}'] = (position.x, position.y)
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.orientations[f'{self.namespace}_{robot_id}'] = yaw
        
    def control_loop(self):
        """控制循环，计算并发布控制指令"""
        leader_pos = self.positions.get(f'{self.namespace}_{self.leader_id}')
        leader_yaw = self.orientations.get(f'{self.namespace}_{self.leader_id}')
        if leader_pos is None or leader_yaw is None:
            return  # 尚未收到领航者的位置或朝向信息
        
        # 获取当前时间和已过去的时间
        current_time = time.time() - self.start_time
        x = self.linear_speed * current_time  # 假设领航者沿x轴匀速前进

        # 计算sin曲线的目标y值
        y = self.A * np.sin(self.k * x)

        # 计算角度：根据y(x)的导数（曲线的切线角度）
        dy_dx = self.A * self.k * np.cos(self.k * x)  # 曲线在x位置的导数
        theta = np.arctan2(dy_dx, 1)  # 曲线的切线角度，即领航者的角度

        # 计算领航者的速度
        leader_cmd_vel = Twist()
        leader_cmd_vel.linear.x = self.linear_speed  # 线速度
        leader_cmd_vel.angular.z = theta - leader_yaw  # 控制角度的误差
        
        # 限制最大速度
        leader_cmd_vel.linear.x = np.clip(leader_cmd_vel.linear.x, -0.5, 0.5)
        leader_cmd_vel.angular.z = np.clip(leader_cmd_vel.angular.z, -1.0, 1.0)

        # 发布领航者的速度命令
        self.cmd_publishers[self.leader_id].publish(leader_cmd_vel)
        
        # 控制跟随者
        for follower_id in self.follower_ids:
            follower_pos = self.positions.get(f'{self.namespace}_{follower_id}')
            follower_yaw = self.orientations.get(f'{self.namespace}_{follower_id}')
            if follower_pos is None or follower_yaw is None:
                continue  # 尚未收到该跟随者的位置或朝向信息
            
            # 计算目标位置（领航者位置 + 编队偏移，考虑领航者的朝向）
            offset_x, offset_y = self.formation_offsets[follower_id]
            target_x = leader_pos[0] + offset_x * np.cos(leader_yaw) - offset_y * np.sin(leader_yaw)
            target_y = leader_pos[1] + offset_x * np.sin(leader_yaw) + offset_y * np.cos(leader_yaw)
            
            # 计算全局坐标系下的位置误差
            error_x_global = target_x - follower_pos[0]
            error_y_global = target_y - follower_pos[1]
            
            # 转换到跟随者的局部坐标系
            cos_yaw = np.cos(follower_yaw)
            sin_yaw = np.sin(follower_yaw)
            error_x_local = cos_yaw * error_x_global + sin_yaw * error_y_global
            error_y_local = -sin_yaw * error_x_global + cos_yaw * error_y_global
            
            # 使用P控制器计算线速度和角速度
            cmd_vel = Twist()
            cmd_vel.linear.x = 1 * error_x_local  # 前向误差控制线速度
            cmd_vel.angular.z = 0.5 * error_y_local  # 侧向误差控制角速度
            
            # 限制速度
            cmd_vel.linear.x = np.clip(cmd_vel.linear.x, -0.5, 0.5)
            cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -1.0, 1.0)
            
            # 发布控制指令
            self.cmd_publishers[follower_id].publish(cmd_vel)
    
    def stop_robots(self):
        """停止所有机器人"""
        stop_cmd = Twist()
        for publisher in self.cmd_publishers.values():
            publisher.publish(stop_cmd)
    
    def destroy(self):
        """节点销毁时停止所有机器人"""
        self.stop_robots()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = FormationController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.destroy()  # 终止时停止机器人

if __name__ == '__main__':
    main()
