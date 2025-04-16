from math import sqrt, cos, sin, atan2

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,String
from geometry_msgs.msg import Twist
import yaml

def saturation(x,bound):
    '''
    限制输入值在给定范围内
    '''
    x = np.array(x)
    x_m = np.linalg.norm(x)
    if x_m > bound:
        return x/x_m*bound
    else:
        return x

def sign(x):
    '''
    返回输入值的符号
    '''
    x = np.array(x)
    y = []
    for number in x:
        if number > 0:
            y.append(1)
        elif number < 0:
            y.append(-1)
        else:
            y.append(0)
    return np.array(y)

def u2vw_global(u,theta_agent,offset):
    """
    将二维速度向量转换为机器人线速度和角速度。(全局坐标系下的u)

    参数:
        u (array-like): 目标速度向量 [u_x, u_y]。
        theta_agent (float): 机器人当前的朝向角（弧度）。
        offset (float): 机器人中心到旋转轴的偏移距离。

    返回:
        numpy.ndarray: 包含线速度 V 和角速度 W 的数组 [V, W]。

    注意:
        - 线速度 V 是沿机器人前进方向的速度。
        - 角速度 W 是机器人绕自身旋转的速度。
    """
    m = sqrt(u[0]**2 + u[1]**2)
    theta_d = atan2(u[1],u[0])
    theta_err = theta_agent-theta_d
    v = m*cos(theta_err)
    omega = -m*sin(theta_err)/offset
    return np.array([v,omega])

def u2vw_local(u,offset):
    """
    将二维速度向量转换为机器人线速度和角速度。(局部坐标系下的u)

    参数:
        u (array-like): 目标速度向量 [u_x, u_y]。
    """
    v= u[0]
    omega = u[1]/offset
    return np.array([v,omega])

class ControlNode(Node):
    '''
    ControlNode类是一个ROS2节点，用于控制多个TurtleBot3机器人形成特定的队形。
    该节点通过订阅传感器数据和发布控制命令，实现对机器人的控制。
    属性:
        num_robots (int): 机器人数量，默认为4。
        offset (float): 机器人之间的偏移量，用于控制律计算。
        sensor_info (list): 从 /sensor_info 话题中读取的传感器信息列表。
        subscription (Subscription): 订阅 /sensor_data 话题的订阅器。
        cmd_publishers (dict): 每个机器人对应的控制命令发布器。
        adjacency_matrix (list): 从配置文件中读取的邻接矩阵，用于描述机器人之间的连接关系。
        last_publish_time (Time): 上一次发布控制命令的时间。
        last_print_time (Time): 上一次打印日志的时间。
        origin_time (Time): 节点启动的时间。
        current_time (Time): 当前时间。
    方法:
        __init__(): 初始化ControlNode节点，设置订阅器、发布器和定时器，并加载配置文件。
        load_config(param_name): 从配置文件中读取指定参数的值。
        get_sensor_info(): 一次性读取 /sensor_info 话题的数据，并返回解析后的传感器信息列表。
        sensor_callback(msg): 传感器数据的回调函数，接收传感器数据并调用控制律函数进行处理。
        control_laws(dic): 控制律函数，根据传感器数据计算控制命令，并发布给各个机器人。
    使用说明:
        1. 确保配置文件 `config.yml` 存在，并包含 `adjacency_matrix` 和 `offset` 参数。
        2. 启动节点后，节点会自动订阅 /sensor_data 和 /sensor_info 话题，
            并发布控制命令到各个机器人。
        3. 修改控制律函数 `control_laws` 以实现自定义的机器人控制逻辑。
    '''
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
        self.distance_matrix = self.load_config('desired_distance_matrix') # 读取距离矩阵
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
        """
        传感器回调函数。
        此函数接收传感器数据消息，将传感器名称与对应的传感器值组成一个字典，
        并将该字典传递给控制律函数进行处理。
        参数:
            msg (Float32MultiArray): 
                包含传感器数据的消息，其中 `data` 是一个浮点数数组，
                表示传感器的测量值。
        """
        sensor_dict = dict(zip(self.sensor_info, msg.data))
        self.control_laws(sensor_dict)

    def reference_motion(self,t,d_12):
        """
        计算参考运动。
        参数:
            t (float): 当前时间。
            d_12,leader和co_leader之间的期望距离
        返回:
            tuple: 包含以下三个元素的元组：
                - vd (numpy.ndarray): 目标速度向量 [vx, vy]。
                - po_d (list): leader 和 co-leader 之间的相对位置，方向控制
                - po_d_dot (list): 上面的导数
        说明:
            该函数基于时间 t 计算参考运动的速度、位置和位置变化率。
            其中，`d_12` 表示 leader 和 co-leader 之间的固定距离。
        """

        vx = 1*cos(0.2*t)
        vy = 0.6*sin(0.2*t)
        dtheta = 3/5*(1/cos(0.4*t)**2)*0.4
        theta = atan2(vy,vx)
        vd = np.array([vx,vy])
        po_d = [d_12*cos(theta),d_12*sin(theta)]
        po_d_dot = [-d_12*sin(theta)*dtheta,d_12*cos(theta)*dtheta]

        return (vd,po_d,po_d_dot)

    def get_t_since_origin(self):
        '''
        返回距离开始的时间
        '''
        return (self.current_time.nanoseconds - self.origin_time.nanoseconds)/1e9

    def control_laws(self, dic):
        """
        控制律函数，用于计算并发布控制命令。
        参数:
            dic (dict): 包含机器人状态信息的字典，包括每个机器人的偏航角（yaw）。
        功能描述:
            1. 初始化控制器变量 `u` 和 `vnw`，分别表示控制输入和速度映射结果。
            2. 设置控制增益参数 `kp`、`alpha` 和 `beta`。
            3. 设计控制律，初始化每个机器人的控制输入 `u`。
            4. 使用 `u2vw` 函数将控制输入 `u` 映射为速度和角速度 `vnw`。
            5. 遍历计算得到的速度和角速度 `vnw`，生成对应的 `Twist` 消息并通过 ROS 发布器发布。
        注意:
            - `u2vw` 是一个自定义函数，用于将控制输入映射为速度和角速度。
            - TurtleBot3 默认不支持 y 方向的线速度，因此 `twist.linear.y` 被设置为 0.0。
            - `self.cmd_publishers` 是一个发布器列表，用于向每个机器人发布控制命令。
        """
        # 创建新的控制器变量
        u = [[] for i in range(4)]
        vnw = [[] for i in range(4)] # v and omega

        # reference motion信号
        vd, po_d, po_d_dot = self.reference_motion(self.get_t_since_origin(),d_12=2)

        # 控制增益
        k=0.1
        alpha=10
        beta=0

        # 控制器中间变量
        r = [ [0,0] for i in range(self.num_robots)]
        for i in range(1,self.num_robots):  # 遍历所有除了领导者的agent
            for j in range(self.num_robots):  # 逐个检查是否是邻居
                if self.adjacency_matrix[i][j]==0 or i==j :  # 不是邻居
                    continue
                else: # 是邻居
                    if i==1: # co-leader 读取的相对位置是全局坐标
                        p_ij = np.array([dic[f'x_{i+1}{j+1}'],dic[f'y_{i+1}{j+1}']])
                    else: # followers  读取的相对位置是local坐标
                        p_ij = np.array([dic[f'x_{i+1}{j+1}_local'],dic[f'y_{i+1}{j+1}_local']])

                    d = self.distance_matrix[i][j] # desired distance
                    p_ij_m = np.linalg.norm(p_ij) # L2 norm
                    sigma_ij = p_ij_m**2-d**2 # square distance error
                    r_j = p_ij*sigma_ij
                    r[i] = np.add(r[i],r_j)

        po = np.array([-dic['x_21'],-dic['y_21']]) # p1-p2
        po_bar = po-po_d

        # SI控制律设计
        v_max = 1
        for i in range(self.num_robots):
            if i==0:
                u[i] = vd
            elif i==1:
                eta = alpha*np.dot(po_bar,po_d_dot)/(np.linalg.norm(r[i]-alpha*po_bar)**2)
                u[i] = -(k-eta)*(r[i]-alpha*po_bar) + vd
                u[i] = saturation(u[i],v_max)
            else:
                u[i] = -k*r[i] - beta*sign(r[i])
                u[i] = saturation(u[i],v_max)

        # Unicycle控制律映射
        vnw[0] = u2vw_global(u[0],dic['yaw_1'],self.offset)
        vnw[1] = u2vw_global(u[1],dic['yaw_2'],self.offset)
        vnw[2] = u2vw_local(u[2],self.offset)
        vnw[3] = u2vw_local(u[3],self.offset)

        # 生成并发布每个车的twist话题
        for i,vw in enumerate(vnw):
            twist = Twist()
            twist.linear.x = vw[0]
            twist.linear.y = 0.0  # TurtleBot3默认不支持y向速度
            twist.angular.z = vw[1]
            self.cmd_publishers[i+1].publish(twist)

        # 计算控制器的发布频率
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_publish_time).nanoseconds/1e9
        self.last_publish_time = self.current_time
        if dt<1e-9:
            freq = 0
        else:
            freq = round(1/dt,2)

        # 按照0.5Hz打印传控制器数据
        t_since_last_print = \
            (self.current_time - self.last_print_time).nanoseconds/1e9
        if t_since_last_print >= 2:
            self.last_print_time = self.current_time
            self.get_logger().info(f"**********{self.get_t_since_origin():.2f}s**********")
            self.get_logger().info(f"Controller freq: {freq} Hz")


def main(args=None):
    '''
    ros2 node入口
    '''
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
