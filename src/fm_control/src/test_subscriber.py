import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time


class MultiRobotSubscriber(Node):
    def __init__(self):
        super().__init__("multi_robot_subscriber")

        # 机器人数量
        self.number_of_robots = 4
        # 命名空间前缀
        self.namespace = "TB3"

        # 创建订阅者字典
        self.cmd_vel_subscribers = {}
        self.odom_subscribers = {}

        # 为每个机器人创建订阅者
        for i in range(1, self.number_of_robots + 1):
            robot_namespace = f"{self.namespace}_{i}"
            cmd_vel_topic = f"/{robot_namespace}/cmd_vel"
            odom_topic = f"/{robot_namespace}/odom"

            # 创建并保存 cmd_vel 订阅者
            self.cmd_vel_subscribers[robot_namespace] = self.create_subscription(
                Twist,
                cmd_vel_topic,
                lambda msg, ns=robot_namespace: self.cmd_vel_callback(msg, ns),
                10,
            )

            # 创建并保存 odom 订阅者
            self.odom_subscribers[robot_namespace] = self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, ns=robot_namespace: self.odom_callback(msg, ns),
                10,
            )

        # 初始化上次打印时间
        self.last_print_time = time()

    def cmd_vel_callback(self, msg, namespace):
        # 处理 cmd_vel 消息
        current_time = time()
        if current_time - self.last_print_time >= 2.0:
            linear_velocity = round(msg.linear.x, 4)
            angular_velocity = round(msg.angular.z, 4)
            self.get_logger().info(
                f"[{namespace} /cmd_vel] Linear Velocity: {linear_velocity:.4f}, Angular Velocity: {angular_velocity:.4f}"
            )
            self.last_print_time = current_time

    def odom_callback(self, msg, namespace):
        # 处理 odom 消息
        current_time = time()
        if current_time - self.last_print_time >= 2.0:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            position_x = round(position.x, 4)
            position_y = round(position.y, 4)
            position_z = round(position.z, 4)
            orientation_x = round(orientation.x, 4)
            orientation_y = round(orientation.y, 4)
            orientation_z = round(orientation.z, 4)
            orientation_w = round(orientation.w, 4)
            self.get_logger().info(
                f"[{namespace} /odom] Position -> x: {position_x:.4f}, y: {position_y:.4f}, z: {position_z:.4f}"
            )
            self.get_logger().info(
                f"[{namespace} /odom] Orientation -> x: {orientation_x:.4f}, y: {orientation_y:.4f}, z: {orientation_z:.4f}, w: {orientation_w:.4f}"
            )
            self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
