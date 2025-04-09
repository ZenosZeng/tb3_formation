import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.num_robots = 4
        self.offset = 0.1 # unicycle robot的偏移量

        # 订阅sensor_node的info
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/sensor_info',
            self.sensor_callback,
            10)

        # 创建四个tb的cmd发布器
        self.cmd_publishers = {
            1: self.create_publisher(Twist, '/TB3_1/cmd_vel', 10),
            2: self.create_publisher(Twist, '/TB3_2/cmd_vel', 10),
            3: self.create_publisher(Twist, '/TB3_3/cmd_vel', 10),
            4: self.create_publisher(Twist, '/TB3_4/cmd_vel', 10)
        }

        # 定义伴随矩阵（4x4）（此处需要与sensor_node一致）
        # a_ij = 1 时，表示机器人 i 与机器人 j 之间需要控制
        self.adjacency_matrix = np.array([[0,0,0,0],
                                          [1,0,1,1],
                                          [0,1,0,1],
                                          [1,1,1,0]], dtype=int)
        
        # 定义期望距离的矩阵
        d1=2
        d2=1
        d3=5**0.5
        self.desired_distance = np.array([[0,0,0,0],
                                          [2,0,1,2.236],
                                          [0,1,0,2],
                                          [1,2.236,2,0]], dtype=float)

        self.get_logger().info("Control node started.")
        self.origin_time = self.get_clock().now()


    def sensor_callback(self, msg: Float32MultiArray):
        data = msg.data

        # 检查sensor_info正确性
        if len(data) % 5 != 0:
            self.get_logger().warn("Invalid sensor_info data length.")
            return
        
        # 计算每个robot的刚性控制项r=\Sigma p_ij*\sigma_ij 
        r = [ [0 for i in range(2)] for j in range(self.num_robots)]

        for i in range(0, len(data), 5):
            from_id = int(data[i])
            to_id = int(data[i + 1])
            dx = data[i + 2]
            dy = data[i + 3]
            distance = data[i + 4]

            control_edges.append((from_id, to_id, dx, dy, distance))

        self.apply_control(control_edges)

    def apply_control(self, edges):
        # 简单的比例控制策略：根据 dx, dy 推导速度控制
        # 可以扩展为李雅普诺夫控制、刚性控制等
        gains = 0.5

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
