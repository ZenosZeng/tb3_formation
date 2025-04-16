import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,String
from geometry_msgs.msg import Twist
import yaml
from math import sqrt, cos, sin, atan2

def get_distance(x,y):
    '''
    计算2D坐标系下向量的距离
    '''
    return np.linalg.norm(np.array([x,y]))

def u2vw(u,theta_agent,offset):
    m = sqrt(u[0]**2 + u[1]**2)
    theta_d = atan2(u[1],u[0])
    theta_err = theta_agent-theta_d
    V = m*cos(theta_err)
    W = -m*sin(theta_err)/offset
    return V,W

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.num_robots = 4
        self.offset = 0.1 # unicycle robot的偏移量

        # 一次性读取 /sensor_info 的数据
        self.sensor_info = self.get_sensor_info()
        self.get_logger().info(f"Sensor info: {self.sensor_info}")

        # 订阅sensor_node的data和info
        self.subscription = self.create_subscription(
            Float32MultiArray,'/sensor_data',self.sensor_callback,10)
        # self.subscription2 = self.create_subscription(
        #     String,'/sensor_info',self.sensor_info_callback,10)

        # 创建四个tb的cmd发布器
        self.cmd_publishers = {
            1: self.create_publisher(Twist, '/TB3_1/cmd_vel', 10),
            2: self.create_publisher(Twist, '/TB3_2/cmd_vel', 10),
            3: self.create_publisher(Twist, '/TB3_3/cmd_vel', 10),
            4: self.create_publisher(Twist, '/TB3_4/cmd_vel', 10)
        }

        self.adjacency_matrix = self.load_config('adjacency_matrix')  # 读取邻接矩阵
        self.offset = self.load_config('offset')  # 读取偏移量

        # 创建定时器，控制消息发布的时间间隔
        self.last_publish_time = self.get_clock().now()
        self.last_print_time = self.get_clock().now()
        self.origin_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        self.get_logger().info("Control node started.")

    def load_config(self, param_name):
        '''
        读取yaml文件中对应全局变量
        '''
        file_path = '/home/zenos/ROS2WS/tb3_formation/src/'+\
            'formation_controller/formation_controller/config.yml'
        with open(file_path, 'r',encoding='UTF-8') as file:
            config = yaml.safe_load(file)
        return config[param_name]
    
    def get_sensor_info(self):
        '''
        一次性读取 /sensor_info 的数据
        '''
        self.get_logger().info("Waiting for /sensor_info message...")

        # 定义一个 Future 对象，用于等待消息
        future = rclpy.task.Future()

        # 回调函数，用于接收消息并设置 Future 的结果
        def callback(msg):
            if not future.done():
                future.set_result(msg.data)

        # 创建临时订阅器
        subscription = self.create_subscription(String, '/sensor_info', callback, 10)

        # 等待消息
        rclpy.spin_until_future_complete(self, future)

        # 删除订阅器
        self.destroy_subscription(subscription)

        # 返回解析后的数据
        if future.done():
            self.get_logger().info("/sensor_info received.")
            return future.result().split(',')  # 将字符串分割为列表
        else:
            self.get_logger().error("Failed to receive /sensor_info message.")
            return []

    def sensor_callback(self, msg: Float32MultiArray):
        # 将传感器名称和传感器值组成字典
        sensor_dict = dict(zip(self.sensor_info, msg.data))
        self.control_laws(sensor_dict)

    def control_laws(self, dic):
        u = np.zeros(4)
        # 控制增益
        kp=1

        # 控制律设计
        u[1 ]= [1,0.1]
        u2 = [1,0.1]
        u3 = [1,0.1]
        u4 = [1,0.1]

        # 控制律映射
        v1,w1 = u2vw(u1,dic['yaw_1'],self.offset)
        v2,w2 = u2vw(u2,dic['yaw_2'],self.offset)
        v3,w3 = u2vw(u3,dic['yaw_3'],self.offset)
        v4,w4 = u2vw(u4,dic['yaw_4'],self.offset)

        
        for edge in edges:
            from_id, to_id, dx, dy, distance = edge

            # 控制 to_id 向 from_id 移动（例如让机器人2靠近机器人1）
            twist = Twist()
            twist.linear.x = gains * dx 
            twist.linear.y = gains * dy  # TurtleBot3 默认不支持 y 向速度，如果是全向底盘可以保留
            twist.angular.z = 0.0

            if to_id in self.publishers:
                self.robot_publishers[to_id].publish(twist)
                self.get_logger().info(
                    f"Sent velocity to TB3_{to_id}: dx={dx:.2f}, dy={dy:.2f}, dist={distance:.2f}")
            else:
                self.get_logger().warn(f"No publisher for TB3_{to_id}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
