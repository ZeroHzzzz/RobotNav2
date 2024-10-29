import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import serial
import re

class Nav2PointNavigator(Node):
    def __init__(self):
        super().__init__('nav2_point_navigator')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.result_sub = self.create_subscription(String, '/bt_navigator/transition_event', self.result_callback, 10)

        # 定义导航点
        self.points = {
            'nurse': (4.09, 2.15, 0.0),
            'point1': (4.09, 2.15, 0.0),
            'point2': (4.53, -2.24, 0.0),
            'start': (0.0, 0.0, 0.0),
        }
        self.navigation_sequence = []
        self.current_goal = None
        self.navigation_complete = False

        # 初始化串行通信
        self.serial_port = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=None
        )

        # 定时器相关
        self.timer_period = 2.0  # 每2秒发布一次
        self.goal_timer = None

    def result_callback(self, msg):
        """根据导航结果事件，判断目标是否成功到达."""
        if 'SUCCEEDED' in msg.data:
            self.get_logger().info("Goal reached successfully.")
            self.navigation_complete = True

            # 停止定时器，避免继续发布
            if self.goal_timer:
                self.goal_timer.cancel()
                self.goal_timer = None
        elif 'FAILED' in msg.data:
            self.get_logger().warn("Failed to reach the goal. Retrying...")
            # 如果失败，可以选择是否继续重试
            if self.current_goal:
                self.publish_goal(self.current_goal)  # 重试当前目标

    def wait_for_serial_data(self, validate_format=False):
        """等待并验证下位机的串口数据格式."""
        pattern = re.compile(r'^FD \w\w CC$') if validate_format else None
        while True:
            data = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received: {data}")
            if validate_format and pattern.match(data):
                break
            elif not validate_format:
                break
            else:
                self.get_logger().warn(f"Unexpected data format: {data}")
        return data

    def publish_goal(self, point):
        """发布导航目标点并启动定时器持续发布，直到到达目标."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)
        self.current_goal = point
        self.navigation_complete = False
        self.get_logger().info(f"Published goal: {point}")

        # 启动定时器，定期重复发布目标
        if self.goal_timer is None:
            self.goal_timer = self.create_timer(self.timer_period, self.publish_goal_timer_callback)

    def publish_goal_timer_callback(self):
        """定期发布当前目标点，确保指令持续有效。"""
        if self.current_goal and not self.navigation_complete:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = self.current_goal[0]
            goal_pose.pose.position.y = self.current_goal[1]
            goal_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_pose)
            self.get_logger().info(f"Re-publishing goal: {self.current_goal}")

    def run_navigation(self):
        # 第一次导航到 nurse 点
        nurse_point = self.points['nurse']
        self.get_logger().info(f"Navigating to nurse point: {nurse_point}")
        self.publish_goal(nurse_point)

        # 等待导航完成
        while not self.navigation_complete:
            rclpy.spin_once(self)

        # 根据下位机反馈决定导航顺序
        self.send_data('FD 03 00 00 00 00 00 00 CB')
        response = self.wait_for_serial_data(validate_format=True)

        if response.startswith("FD"):
            if response[3:5] == "31":
                self.navigation_sequence = ['point1', 'point2', 'start']
            elif response[3:5] == "32":
                self.navigation_sequence = ['point2', 'point1', 'start']
        else:
            self.get_logger().warn("Unexpected data; using default sequence.")
            self.navigation_sequence = ['point1', 'point2', 'start']

        # 导航到其他目标点并执行任务
        for point_key in self.navigation_sequence:
            point = self.points[point_key]
            self.get_logger().info(f"Navigating to {point_key}: {point}")
            self.publish_goal(point)
            
            # 等待当前点导航完成
            while not self.navigation_complete:
                rclpy.spin_once(self)

            # 发送任务完成信号
            self.send_data('FD 04 0A 00 00 00 00 00 CB')
            self.wait_for_serial_data()

        self.get_logger().info("Navigation sequence completed")

    def send_data(self, cmd):
        self.serial_port.write(bytes.fromhex(cmd))
        self.get_logger().info(f"Sent: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    navigator = Nav2PointNavigator()
    navigator.run_navigation()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
