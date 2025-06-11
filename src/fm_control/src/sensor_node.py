from math import cos, sin
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from tf_transformations import euler_from_quaternion

# 控制配置文件的绝对路径（偷懒写法）
CONFIG_PATH = "/home/zenos/ws/tb3_formation/config/control_config.yml"


def get_config_para(param):
    with open(CONFIG_PATH, "r", encoding="UTF-8") as file:
        config = yaml.safe_load(file)

    return config[param]


class SensorNode(Node):
    """
    传感器节点
    """

    def __init__(self):
        super().__init__("sensor_node")

        # 订阅四个 TurtleBot3 的里程计话题
        self.subscriber_turtlebot1 = self.create_subscription(
            Odometry, "/robot_1/odom", self.odom_callback, 1
        )
        self.subscriber_turtlebot2 = self.create_subscription(
            Odometry, "/robot_2/odom", self.odom_callback, 1
        )
        self.subscriber_turtlebot3 = self.create_subscription(
            Odometry, "/robot_3/odom", self.odom_callback, 1
        )
        self.subscriber_turtlebot4 = self.create_subscription(
            Odometry, "/robot_4/odom", self.odom_callback, 1
        )

        # 创建2个发布者,分别发布data和info  info用来储存变量名称，逗号分隔
        self.publisher_sensor_data = self.create_publisher(
            Float32MultiArray, "sensor_data", 1
        )
        self.publisher_sensor_info = self.create_publisher(String, "sensor_info", 1)
        # self.published_msg = None

        # 初始化容器和config参数
        self.positions = {}  # 存储机器人的位置字典
        self._adjacency_matrix = get_config_para("ADJACENCY_MATRIX")  # 读取邻接矩阵
        self._offset = get_config_para("OFFSET")  # 读取偏移量
        self.get_logger().info("Config loaded.")

        # 创建定时器，控制消息发布的时间间隔
        self.last_publish_time = self.get_clock().now()
        self.last_print_time = self.get_clock().now()
        self.origin_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        self.get_logger().info("Sensor node started.")

    def odom_callback(self, msg: Odometry):
        """
        里程计回调函数
        """
        robot_id = int(
            msg.header.frame_id.split("_")[1][0]
        )  # 读取机器人ID,原始数据/robot_1/odom

        position = msg.pose.pose.position  # 提取位置 xyz
        orientation_q = msg.pose.pose.orientation  # 提取四元数 xyzw

        # 将四元数转换为欧拉角
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)  # 提取yaw角（车的\theta角，x轴逆时针为正）

        # 存储位置和方向（带有offset）
        self.positions[robot_id] = (
            position.x + self._offset * cos(yaw),
            position.y + self._offset * sin(yaw),
            yaw,
        )

        # 存储位置和方向
        # self.positions[robot_id] = (position.x,
        #                             position.y,
        #                             yaw)

        # 发布sensor数据
        self.calculate_sensor_data()

    def calculate_sensor_data(self):
        """
        计算传感器数据，并发布
        """
        # 如果位置数据不完整，跳过计算
        if len(self.positions) < 4:
            self.get_logger().info("Waiting for all robots' odometry data.")
            return None

        # 创建传感data和info列表
        sensor_info = []
        sensor_data = []

        # 计算所有的相对位置，注意区分全局坐标系和个体坐标系
        for i in range(1, 5):  # robot_id 1234
            for j in range(1, 5):
                if self._adjacency_matrix[i - 1][j - 1] == 1:
                    # 识别到邻接矩阵中的边a_ij
                    x_i, y_i, yaw_i = self.positions[i]
                    x_j, y_j, _ = self.positions[j]
                    x_ij = x_i - x_j
                    y_ij = y_i - y_j

                    # if i<=2: # leader or co-leader
                    # 计算全局坐标系下的相对位置
                    sensor_data.append([round(x_ij, 4), round(y_ij, 4)])
                    sensor_info.append(f"x_{i}{j}")
                    sensor_info.append(f"y_{i}{j}")
                    # else: # followers
                    #     # 计算个体坐标系下的相对位置
                    #     x_ij_local = x_ij * cos(yaw_i) + y_ij * sin(yaw_i)
                    #     y_ij_local = -x_ij * sin(yaw_i) + y_ij * cos(yaw_i)
                    #     sensor_data.append([x_ij_local, y_ij_local])
                    #     sensor_info.append(f"x_{i}{j}_local")
                    #     sensor_info.append(f"y_{i}{j}_local")

        # 记录所有车的yaw角
        for i in range(1, 5):
            _, _, yaw_i = self.positions[i]
            sensor_data.append([round(yaw_i, 4)])
            sensor_info.append(f"yaw_{i}")

        # 发布sensor data/info
        sensor_data_msg = Float32MultiArray()
        for data in sensor_data:
            sensor_data_msg.data.extend(data)
        sensor_info_msg = String()
        for info in sensor_info:
            sensor_info_msg.data += f"{info},"

        self.publisher_sensor_data.publish(sensor_data_msg)
        self.publisher_sensor_info.publish(sensor_info_msg)

        # 计算传感器的发布频率
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = self.current_time
        if dt < 1e-9:
            freq = 0
        else:
            freq = round(1 / dt, 2)

        # 按照0.5Hz打印传感器数据
        t_since_last_print = (
            self.current_time - self.last_print_time
        ).nanoseconds / 1e9
        if t_since_last_print >= 2:
            self.last_print_time = self.current_time
            t_since_origin = (
                self.current_time.nanoseconds - self.origin_time.nanoseconds
            ) / 1e9
            self.get_logger().info(f"**********{t_since_origin:.2f}s**********")
            self.get_logger().info(f"Sensor data: {sensor_data_msg.data}")
            self.get_logger().info(f"Sensor info: {sensor_info_msg.data}")
            self.get_logger().info(f"Sensor freq: {freq} Hz")


def main(args=None):
    """
    主函数,ros2节点入口
    """
    rclpy.init(args=args)
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
