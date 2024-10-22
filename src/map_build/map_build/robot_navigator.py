from enum import Enum
import time
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import (AssistedTeleop, BackUp, Spin, ComputePathThroughPoses,
                              ComputePathToPose, FollowPath, FollowWaypoints,
                              NavigateThroughPoses, NavigateToPose, SmoothPath)
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):

    def __init__(self, node_name='basic_navigator'):
        super().__init__(node_name=node_name)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goToPose(self, pose):
        """Send a `NavToPose` action request."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error(f"Goal to {pose.pose.position.x}, {pose.pose.position.y} was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f"Task failed with status code: {self.status}")
                return True
        return False

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self):
        self.get_logger().info("Waiting until Nav2 is active...")
        # Add any logic needed to ensure the system is ready for navigation

    def _amclPoseCallback(self, msg):
        self.get_logger().info("Received AMCL pose")
        self.initial_pose_received = True

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        self.get_logger().info("Publishing initial pose")
        self.initial_pose_pub.publish(msg)

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        self.get_logger().info(f"Received feedback: {self.feedback}")

