import os
import csv
from datetime import datetime
from math import cos, sin, atan2

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
import yaml

from common.utils import saturation, u2vw_global, sign, tanh_sign

# 配置文件的绝对路径（偷懒写法）
CONFIG_PATH = "/home/zenos/ws/tb3_formation/config/control_config.yml"
RECORD_PATH = "/home/zenos/ws/tb3_formation/data"

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # 读取配置文件
        with open(CONFIG_PATH, "r", encoding="UTF-8") as file:
            config = yaml.safe_load(file)
 
        self._num_robots = config["NUM_OF_ROBOTS"]
        self._adjacency_matrix = config["ADJACENCY_MATRIX"]  # 读取邻接矩阵
        self._distance_matrix = config["DESIRED_DISTANCE_MATRIX"]  # 读取距离矩阵
        self._offset = config["OFFSET"]  # 读取偏移量
        self._v_max = config["V_MAX"]
        self._control_mode = config["CONTROL_MODE"]
        self._lpf_ratio = config["LPF_RATIO"]

        self._k = config["K"]
        self._alpha = config["ALPHA"]
        self._beta = config["BETA"]

        self._tanh_beta = config["TANH_BETA"]
        self._tanh_k = config["TANH_K"]

        self.get_logger().info("1 Config loaded.")

        # 一次性读取 /sensor_info 的数据
        self.sensor_info = self.get_sensor_info()
        self.get_logger().info(f"2 Sensor info received: {self.sensor_info}")

        # 订阅 /sensor_data
        self.subscription = self.create_subscription(
            Float32MultiArray, "/sensor_data", self.sensor_callback, 10
        )
        self._sensor_data = {}

        # 计时器
        self.last_control_time = self.get_clock().now()
        self.last_print_time = self.get_clock().now()
        self.origin_time = self.get_clock().now()

        # 控制循环的定时器 1000hz
        self.controller_timer = self.create_timer(0.001, self.control_loop)

        # 记录循环的定时器 50hz
        self.record_timer = self.create_timer(0.02, self.record_loop)
        self.d_error_dic = {}
        self.orientation_error = 0.0
        self.record_table = []

        # 记录文件输出循环的定时器 0.5hz
        self.output_timer = self.create_timer(2.0, self.output_loop)
        dt = datetime.fromtimestamp(self.origin_time.nanoseconds / 1e9)
        time_str = dt.strftime("%Y%m%d_%H%M%S")
        self._data_dir = RECORD_PATH + f"/{time_str}"
        os.makedirs(self._data_dir, exist_ok=True)  # 确保数据目录存在

        self.get_logger().info("3 Timer created.")

        # 创建四个tb的cmd发布器
        self.cmd_publishers = {
            1: self.create_publisher(Twist, "/robot_1/cmd_vel", 10),
            2: self.create_publisher(Twist, "/robot_2/cmd_vel", 10),
            3: self.create_publisher(Twist, "/robot_3/cmd_vel", 10),
            4: self.create_publisher(Twist, "/robot_4/cmd_vel", 10),
        }

        # 创建指令buffer 用于cmd的LPF
        self.u_buffer = [[0, 0] for i in range(4)]
        self.actual_freq = 1000

        self.get_logger().info("Control node running...")

    def stop_all_robots(self):
        self.get_logger().info("Stopping all robots...")
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.angular.z = 0.0
        for i in range(1, self._num_robots + 1):
            self.cmd_publishers[i].publish(stop_msg)

    def get_sensor_info(self):
        self.get_logger().info("Waiting for /sensor_info message...")

        # 定义一个 Future 对象，用于等待消息
        future = rclpy.task.Future()

        # 回调函数，用于接收消息并设置 Future 的结果
        def callback(msg):
            if not future.done():
                future.set_result(msg.data)

        # 创建临时订阅器
        subscription = self.create_subscription(String, "/sensor_info", callback, 10)

        # 等待消息
        rclpy.spin_until_future_complete(self, future)

        # 删除订阅器
        self.destroy_subscription(subscription)

        # 返回解析后的数据
        if future.done():
            self.get_logger().info("/sensor_info received.")
            return future.result().split(",")  # 将字符串分割为列表
        else:
            self.get_logger().error("Failed to receive /sensor_info message.")
            return []

    def sensor_callback(self, msg: Float32MultiArray):
        sensor_dict = dict(zip(self.sensor_info, msg.data))
        self._sensor_data = sensor_dict

    def reference_motion(self, t, d_12):
        if t<8:
            # 前5s走直线
            vd = [0.3, 0.0]
            po_d = [d_12, 0.0]  # 期望位置
            po_d_dot = [0.0, 0.0]  # 期望位置的导数
            return (vd, po_d, po_d_dot)
        
        t-=8

        # 圆轨迹
        a = 5 # 圆半径
        omega = 0.05 # 椭圆运动的角速度，减小omega可以增加周期

        vx = a * omega * cos(omega * t)
        vy = a * omega * sin(omega * t)

        theta = atan2(vy, vx)
        dtheta = (1 / cos(omega * t) ** 2) * omega

        vd = np.array([vx, vy])
        po_d = [d_12 * cos(theta), d_12 * sin(theta)]
        po_d_dot = [-d_12 * sin(theta) * dtheta, d_12 * cos(theta) * dtheta]

        return (vd, po_d, po_d_dot)

    def get_t_since_origin(self):
        return (self.get_clock().now().nanoseconds - self.origin_time.nanoseconds) / 1e9

    def control_loop(self):
        if not self._sensor_data:
            self.get_logger().info("sensor_data not init yet. control loop stand by.")
            return

        # 创建新的控制器变量
        u = [[] for i in range(4)]
        vnw = [[] for i in range(4)]  # v and omega

        # 参考速度信号和传感器信息
        vd, po_d, po_d_dot = self.reference_motion(self.get_t_since_origin(), d_12=2)
        sensor_dic = self._sensor_data

        # 控制增益
        k = self._k
        alpha = self._alpha
        beta = self._beta

        tanh_beta = self._tanh_beta
        tanh_k = self._tanh_k

        # 控制器中间变量
        r = [[0, 0] for i in range(self._num_robots)]
        for i in range(1, self._num_robots):  # 遍历所有除了领导者的agent 1-3
            for j in range(self._num_robots):  # 逐个检查是否是i的邻居 0-3
                if self._adjacency_matrix[i][j] == 0 or i == j:  # 不是邻居
                    continue
                else:  # 是邻居
                    p_ij = np.array(
                        [sensor_dic[f"x_{i+1}{j+1}"], sensor_dic[f"y_{i+1}{j+1}"]]
                    )

                    d = self._distance_matrix[i][j]  # desired distance
                    p_ij_m = np.linalg.norm(p_ij)  # L2 norm
                    sigma_ij = p_ij_m ** 2 - d ** 2  # square distance error
                    r_j = p_ij * sigma_ij
                    r[i] = np.add(r[i], r_j)

                    # 记录距离误差
                    key = f"d{i+1}{j+1}_err"
                    self.d_error_dic[key] = p_ij_m - d

        # 方向误差
        po = np.array([-sensor_dic["x_21"], -sensor_dic["y_21"]])  # p1-p2
        po_bar = po - po_d
        self.orientation_error = np.linalg.norm(po_bar)

        # SI控制律设计
        if self._control_mode == "rigid":  # 刚性编队定向跟踪控制器
            for i in range(self._num_robots):
                if i == 0:
                    u[i] = vd
                elif i == 1:
                    eta = (
                        alpha
                        * np.dot(po_bar, po_d_dot)
                        / (np.linalg.norm(r[i] - alpha * po_bar) ** 2)
                    )
                    u[i] = -(k - eta) * (r[i] - alpha * po_bar) + vd
                    u[i] = saturation(u[i], self._v_max)
                else:
                    u[i] = (
                        -k * r[i]
                        - beta * sign(r[i])
                        - tanh_beta * tanh_sign(r[i], tanh_k)
                    )
                    u[i] = saturation(u[i], self._v_max)
        elif self._control_mode == "rp":  # 相对位置控制器，测试用
            for i in range(self._num_robots):
                if i == 0:
                    u[i] = vd
                elif i == 1:
                    p_ij = np.array([sensor_dic["x_21"], sensor_dic["y_21"]])
                    desired_p_ij = [-2, 0]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k * p_ij_bar
                    u[i] = saturation(u[i], self._v_max)
                elif i == 2:
                    p_ij = np.array([sensor_dic["x_32"], sensor_dic["y_32"]])
                    desired_p_ij = [0, -1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k * p_ij_bar
                    u[i] = saturation(u[i], self._v_max)
                elif i == 3:
                    p_ij = np.array([sensor_dic["x_41"], sensor_dic["y_41"]])
                    desired_p_ij = [0, -1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k * p_ij_bar
                    u[i] = saturation(u[i], self._v_max)
        elif self._control_mode == "rp_ff":  # 相对位置控制器+vd前馈
            for i in range(self._num_robots):
                if i == 0:
                    u[i] = vd
                elif i == 1:
                    p_ij = np.array([sensor_dic["x_21"], sensor_dic["y_21"]])
                    desired_p_ij = [-2, 0]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k * p_ij_bar + vd
                    u[i] = saturation(u[i], self._v_max)
                elif i == 2:
                    p_ij = np.array([sensor_dic["x_32"], sensor_dic["y_32"]])
                    desired_p_ij = [0, -1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k * p_ij_bar + vd
                    u[i] = saturation(u[i], self._v_max)
                elif i == 3:
                    p_ij = np.array([sensor_dic["x_41"], sensor_dic["y_41"]])
                    desired_p_ij = [0, -1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k * p_ij_bar + vd
                    u[i] = saturation(u[i], self._v_max)

        # 给速度指令加一个低通滤波
        for i in range(4):
            for j in range(2):
                self.u_buffer[i][j] = (
                    self._lpf_ratio * u[i][j]
                    + (1 - self._lpf_ratio) * self.u_buffer[i][j]
                )

        # Unicycle控制律映射
        vnw[0] = u2vw_global(self.u_buffer[0], sensor_dic["yaw_1"], self._offset)
        vnw[1] = u2vw_global(self.u_buffer[1], sensor_dic["yaw_2"], self._offset)
        vnw[2] = u2vw_global(self.u_buffer[2], sensor_dic["yaw_3"], self._offset)
        vnw[3] = u2vw_global(self.u_buffer[3], sensor_dic["yaw_4"], self._offset)

        # 生成并发布每个车的twist话题
        for i, vw in enumerate(vnw):
            twist = Twist()
            twist.linear.x = vw[0]
            twist.linear.y = 0.0  # TurtleBot3默认不支持y向速度
            twist.angular.z = vw[1]
            self.cmd_publishers[i + 1].publish(twist)

        # 计算控制器的发布频率
        dt = (self.get_clock().now() - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = self.get_clock().now()
        if dt < 1e-9:
            self.actual_freq = None
        else:
            self.actual_freq = round(1 / dt, 2)

        # 按照0.5Hz打印传控制器数据
        t_since_last_print = (self.get_clock().now() - self.last_print_time).nanoseconds / 1e9
        if t_since_last_print >= 2:
            self.last_print_time = self.get_clock().now()
            self.get_logger().info(f"**********{self.get_t_since_origin():.2f}s**********")
            self.get_logger().info(f"Controller freq: {self.actual_freq} Hz")

    def record_loop(self):
        # 将仿真时间、距离误差和方向误差存入record_table
        row = []
        headers = []

        headers.append("t")
        row.append(round(self.get_t_since_origin(),3))

        headers.append("freq")
        row.append(self.actual_freq)

        for key, value in self.d_error_dic.items():
            headers.append(key)
            row.append(value)
        headers.append("o_error")
        row.append(self.orientation_error)

        # 如果record_table为空，先写表头
        if not self.record_table:
            self.record_table.append(headers)
        self.record_table.append(row)

    def output_loop(self):
        # 输出record_table到csv文件
        if not self.record_table:
            self.get_logger().info("No data to output.")
            return

        output_file = os.path.join(
            self._data_dir, "control_log.csv"
        )

        # 如果文件已存在，先清空内容（以写模式打开会自动清空）
        with open(output_file, "w", newline="", encoding='UTF-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.record_table)

        self.get_logger().info(f"Data output to {output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
