from math import sqrt, cos, sin, atan2
import math

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,String
from geometry_msgs.msg import Twist
import yaml

CONFIG_PATH = '/home/zenos/ws/tb3_formation/config/formation_config.yml'
def get_config_para(param):
    '''
    加载全局config文件中的参数，用绝对路径
    '''
    with open(CONFIG_PATH, 'r',encoding='UTF-8') as file:
        config = yaml.safe_load(file)

    return config[param]

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

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def u2vw_global(u, theta, offset):
    speed = sqrt(u[0]**2 + u[1]**2)
    # 如果速度太小 直接return
    if speed < 1e-3 :
        return np.array([0.0, 0.0])

    # 投影到前向方向的线速度
    v = u[0] * math.cos(theta) + u[1] * math.sin(theta)
    # 朝向误差控制角速度
    omega = (u[0] * -math.sin(theta) + u[1] * math.cos(theta))/offset

    return np.array([v, omega])

# def u2vw_local(u,offset):
#     """
#     将二维速度向量转换为机器人线速度和角速度。(局部坐标系下的u)

#     参数:
#         u (array-like): 目标速度向量 [u_x, u_y]。
#     """
#     v= u[0]
#     omega = u[1]/offset
#     return np.array([v,omega])

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self._num_robots = get_config_para('NUM_OF_ROBOTS')
        self._adjacency_matrix = get_config_para('ADJACENCY_MATRIX')  # 读取邻接矩阵
        self._distance_matrix = get_config_para('DESIRED_DISTANCE_MATRIX') # 读取距离矩阵
        self._offset = get_config_para('OFFSET')  # 读取偏移量
        self._v_max = get_config_para('V_MAX')
        self.control_mode = get_config_para('CONTROL_MODE')
        self.lpf_ratio = get_config_para('LPF_RATIO')

        self._k = get_config_para('K')
        self._alpha = get_config_para('ALPHA')
        self._beta = get_config_para('BETA')

        self.get_logger().info("Config loaded.")

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
            1: self.create_publisher(Twist, '/robot_1/cmd_vel', 10),
            2: self.create_publisher(Twist, '/robot_2/cmd_vel', 10),
            3: self.create_publisher(Twist, '/robot_3/cmd_vel', 10),
            4: self.create_publisher(Twist, '/robot_4/cmd_vel', 10)
        }

        # 创建定时器，控制消息发布的时间间隔
        self.last_publish_time = self.get_clock().now()
        self.last_print_time = self.get_clock().now()
        self.origin_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        # 创建指令buffer 用于滤波 LPF
        self.u_buffer = [ [0,0] for i in range(4) ]

        self.get_logger().info("Control node started.")

        rclpy.get_default_context().on_shutdown(self.stop_all_robots)

    def stop_all_robots(self):
        self.get_logger().info("Shutting down, stopping all robots...")
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for pub in self.cmd_publishers.values():
            pub.publish(stop_msg)

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

        # 调整椭圆轨迹的参数
        a, b = 0.3, 0.2  # 椭圆的长轴和短轴，增大a和b可以放大椭圆
        omega = 0.1  # 椭圆运动的角速度，减小omega可以增加周期

        vx = a * cos(omega * t)
        vy = b * sin(omega * t)

        theta = atan2(vy,vx)
        dtheta = b/a*(1/cos(omega*t)**2)*omega

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
        k = self._k
        alpha = self._alpha
        beta = self._beta

        # 控制器中间变量
        r = [ [0,0] for i in range(self._num_robots)]
        for i in range(1,self._num_robots):  # 遍历所有除了领导者的agent 1-3
            for j in range(self._num_robots):  # 逐个检查是否是i的邻居 0-3
                if self._adjacency_matrix[i][j]==0 or i==j :  # 不是邻居
                    continue
                else: # 是邻居
                    # if i==1: # co-leader 读取的相对位置是全局坐标
                    p_ij = np.array([dic[f'x_{i+1}{j+1}'],dic[f'y_{i+1}{j+1}']])
                    # else: # followers  读取的相对位置是local坐标
                    #     p_ij = np.array([dic[f'x_{i+1}{j+1}_local'],dic[f'y_{i+1}{j+1}_local']])

                    d = self._distance_matrix[i][j] # desired distance
                    p_ij_m = np.linalg.norm(p_ij) # L2 norm
                    sigma_ij = p_ij_m**2-d**2 # square distance error
                    r_j = p_ij*sigma_ij
                    r[i] = np.add(r[i],r_j)

        # 方向误差
        po = np.array([-dic['x_21'],-dic['y_21']]) # p1-p2
        po_bar = po-po_d

        # SI控制律设计
        control_mode = self.control_mode  # 可以随任务修改
        v_max = self._v_max

        if control_mode=='rigid': # 刚性编队定向跟踪控制器
            for i in range(self._num_robots):
                if i==0:
                    u[i] = vd
                elif i==1:
                    eta = alpha*np.dot(po_bar,po_d_dot)/ \
                        (np.linalg.norm(r[i]-alpha*po_bar)**2)
                    u[i] = -(k-eta)*(r[i]-alpha*po_bar) + vd
                    u[i] = saturation(u[i],v_max)
                else:
                    u[i] = -k*r[i] - beta*sign(r[i])
                    u[i] = saturation(u[i],v_max)
        elif control_mode == 'rp': # 相对位置控制器，测试用
            for i in range(self._num_robots):
                if i==0:
                    u[i] = vd
                elif i==1:
                    p_ij = np.array([dic['x_21'],dic['y_21']])
                    desired_p_ij = [-2,0]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k*p_ij_bar
                    u[i] = saturation(u[i],v_max)
                elif i==2:
                    p_ij = np.array([dic['x_32'],dic['y_32']])
                    desired_p_ij = [0,-1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k*p_ij_bar
                    u[i] = saturation(u[i],v_max)
                elif i==3:
                    p_ij = np.array([dic['x_41'],dic['y_41']])
                    desired_p_ij = [0,-1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k*p_ij_bar
                    u[i] = saturation(u[i],v_max)
        elif control_mode == 'rp_ff': # 相对位置控制器+vd前馈
            for i in range(self._num_robots):
                if i==0:
                    u[i] = vd
                elif i==1:
                    p_ij = np.array([dic['x_21'],dic['y_21']])
                    desired_p_ij = [-2,0]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k*p_ij_bar + vd
                    u[i] = saturation(u[i],v_max)
                elif i==2:
                    p_ij = np.array([dic['x_32'],dic['y_32']])
                    desired_p_ij = [0,-1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k*p_ij_bar + vd
                    u[i] = saturation(u[i],v_max)
                elif i==3:
                    p_ij = np.array([dic['x_41'],dic['y_41']])
                    desired_p_ij = [0,-1]
                    p_ij_bar = p_ij - desired_p_ij
                    u[i] = -k*p_ij_bar + vd
                    u[i] = saturation(u[i],v_max)

        # 给速度指令加一个低通滤波
        for i in range(4):
            for j in range(2):
                self.u_buffer[i][j] = \
                    self.lpf_ratio * u[i][j] + \
                    (1-self.lpf_ratio) * self.u_buffer[i][j]

        # Unicycle控制律映射
        vnw[0] = u2vw_global(self.u_buffer[0],dic['yaw_1'],self._offset)
        vnw[1] = u2vw_global(self.u_buffer[1],dic['yaw_2'],self._offset)
        vnw[2] = u2vw_global(self.u_buffer[2],dic['yaw_3'],self._offset)
        vnw[3] = u2vw_global(self.u_buffer[3],dic['yaw_4'],self._offset)

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
            # for i in range(self._num_robots):
            #     self.get_logger().info(vnw[i])

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
