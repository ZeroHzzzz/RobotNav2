import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class ICPFusionNode(Node):
    def __init__(self):
        super().__init__('icp_fusion_node')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info("ICP Fusion Node has started.")

    def publish_icp_initial_pose(self, icp_pose):
        # 创建PoseWithCovarianceStamped消息
        initial_pose_msg = PoseWithCovarianceStamped()

        # 设置时间戳
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = "map"

        # 设置位姿（假设 icp_pose 是一个包含 [x, y, theta] 的元组）
        initial_pose_msg.pose.pose.position.x = icp_pose[0]
        initial_pose_msg.pose.pose.position.y = icp_pose[1]
        initial_pose_msg.pose.pose.position.z = 0.0  # 平面上不需要z方向

        # 使用 tf2_ros 库进行四元数转换
        quaternion = tf_transformations.quaternion_from_euler(0, 0, icp_pose[2])
        initial_pose_msg.pose.pose.orientation.x = quaternion[0]
        initial_pose_msg.pose.pose.orientation.y = quaternion[1]
        initial_pose_msg.pose.pose.orientation.z = quaternion[2]
        initial_pose_msg.pose.pose.orientation.w = quaternion[3]

        # 假设协方差矩阵，示例中用单位矩阵
        initial_pose_msg.pose.covariance = [0.0] * 36
        initial_pose_msg.pose.covariance[0] = 0.5  # x方向的协方差
        initial_pose_msg.pose.covariance[7] = 0.5  # y方向的协方差
        initial_pose_msg.pose.covariance[35] = 0.1  # theta方向的协方差

        # 发布初始位姿
        self.initial_pose_pub.publish(initial_pose_msg)
        self.get_logger().info("Published ICP initial pose to AMCL.")


def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    node = ICPFusionNode()

    # 模拟 ICP 输出的位姿 (x, y, yaw)
    # 你可以替换这个为实际的ICP计算结果
    icp_pose = (2.0, 3.0, np.pi / 4)  # 示例位姿 (x, y, yaw)

    # 发布位姿到AMCL
    node.publish_icp_initial_pose(icp_pose)

    # 保持节点运行
    rclpy.spin(node)

    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
