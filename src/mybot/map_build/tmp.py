import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import serial
import re

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import AssistedTeleop, BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath, FollowWaypoints, FollowGPSWaypoints, \
    NavigateThroughPoses, NavigateToPose
from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3
    
class PointPublish(Node):
    def __init__(self):
        super().__init__('nav2_point_navigator')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        
        self.serial_port = serial.Serial(
            port='/dev/ttyACM1',
            baudrate=115200,
            timeout=None
        )
        
        # self.points = {
        #     'nurse': (2.49, 0.553, 0.7071),
        #     'point1': (2.63, -0.0369, -0.7071),
        #     'point2': (1.16, -0.0181, -0.7071),
        #     'end': (0.0, 0.0, 0.0),
        # }
        
        # self.points = {
        #     'nurse': (0.809, 0.0, 0.0),
        #     'point1': (4.68, 1.91, 0.0),
        #     'point2': (4.68, -1.80, 0.0),
        #     'end': (0.0, 0.0, 0.0),
        # }
        
        self.points = {
            'nurse': (0.609, 0.0, 0.0),
            'point1': (4.64, 1.97, 0.0),
            'point2': (4.68, -1.85, 0.0),
            'end': (0.0, 0.0, 0.0),
        }
        # self.points = {
        #     'nurse': (0.609, 0.0, 0.0),
        #     'point1': (4.44, 1.97, 0.0),
        #     'point2': (4.48, -1.85, 0.0),
        #     'end': (0.0, 0.0, 0.0),
        # }
        self.navigation_sequence = ['point1', 'point2', 'end']
        
    def destroyNode(self):
        self.destroy_node()

    def destroy_node(self):
        self.nav_through_poses_client.destroy()
        self.nav_to_pose_client.destroy()
        super().destroy_node()
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def goThroughPoses(self, poses, behavior_tree=''):
        """Send a `NavThroughPoses` action request."""
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = behavior_tree

        self.info(f'Navigating with {len(goal_msg.poses)} goals....')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Goal with {len(poses)} poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def spin(self, spin_dist=1.57, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def backup(self, backup_dist=0.15, backup_speed=0.025, time_allowance=10):
        self.debug("Waiting for 'Backup' action server")
        while not self.backup_client.wait_for_server(timeout_sec=1.0):
            self.info("'Backup' action server not available, waiting...")
        goal_msg = BackUp.Goal()
        goal_msg.target = Point(x=float(backup_dist))
        goal_msg.speed = backup_speed
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Backing up {goal_msg.target.x} m at {goal_msg.speed} m/s....')
        send_goal_future = self.backup_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Backup request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return
    
    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True
    
    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

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
        
    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return
    
    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _getGoalPoint(self, point):
        goalPoint = PoseStamped()
        goalPoint.header.frame_id = 'map'
        goalPoint.pose.position.x = point[0]
        goalPoint.pose.position.y = point[1]
        # goalPoint.pose.orientation.w = 1.0
        if point[2] > 0:  # 逆时针旋转 90 度
            goalPoint.pose.orientation.z = 0.7071
            goalPoint.pose.orientation.w = 0.7071
        elif point[2] < 0:  # 顺时针旋转 90 度
            goalPoint.pose.orientation.z = -0.7071
            goalPoint.pose.orientation.w = 0.7071
        else:  # 未旋转
            goalPoint.pose.orientation.w = 1.0
        return goalPoint
    
    def send_data(self, cmd):
        response = None
        self.get_logger().info(f"Sent: {cmd}")
        self.serial_port.write(bytes.fromhex(cmd))
        time.sleep(0.5)
        # if self.serial_port.in_waiting > 0:
        #     received_data = self.serial_port.read(self.serial_port.in_waiting)
        #     received_data= ' '.join(f'{byte:02X}' for byte in received_data)
        response = self.serial_port.read_all()
        response = ' '.join(f'{byte:02X}' for byte in response)
        return response

    def run_navigation(self):
        
        self.waitUntilNav2Active()
        while True:
            self.get_logger().info('waiting')
            # 检查串口缓冲区中是否有数据
            if self.serial_port.in_waiting > 0:
                # 读取当前缓冲区中的所有字节
                response = self.serial_port.read(self.serial_port.in_waiting)
                
                # 将接收到的字节转换成十六进制字符串格式进行打印while True:
                response_hex = ' '.join(f'{byte:02X}' for byte in response)
                self.get_logger().info(f"Received data: {response_hex}")
                
                # 执行所需的操作
                self.get_logger().info('start the task')
                break
        
            # 小延时，避免频繁占用 CPU 资源
            time.sleep(0.1)
        
        nurse_point = self.points['nurse']
        self.get_logger().info(f"Navigating to nurse point: {nurse_point}")
        self.goToPose(self._getGoalPoint(nurse_point))
        # result = self.getResult()
        i = 0
        response = None
        while not self.isTaskComplete():
            i += 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info("waiting")

        time.sleep(1)
        # response = self.send_data('FD 02 00 00 00 00 00 00 CB')
        cmd = "FD 02 00 00 00 00 00 00 CB"
        self.get_logger().info(f"Sent: {cmd}")
        self.serial_port.write(bytes.fromhex(cmd))
        
        response = None
        i = 0
        while True:
            i += 1
            self.get_logger().info('waiting')
            # 检查串口缓冲区中是否有数据
            if self.serial_port.in_waiting > 0 or i > 50:
                # 读取当前缓冲区中的所有字节
                response = self.serial_port.read(self.serial_port.in_waiting)
                
                # 将接收到的字节转换成十六进制字符串格式进行打印while True:
                response = ' '.join(f'{byte:02X}' for byte in response)
                self.get_logger().info(f"Received data: {response}")
                
                # 执行所需的操作
                self.get_logger().info('start the task')
                break
        
            # 小延时，避免频繁占用 CPU 资源
            time.sleep(0.1)
        
        # time.sleep(0.3)
        # response = self.serial_port.read_all()
        # response = ' '.join(f'{byte:02X}' for byte in response)
        # self.get_logger().info(response)
        
        # self.get_logger().info(result)
        # 不知道为什么result一直是UNKNOWN,因此这里直接认为已经到达
        # self.send_data('FD 02 00 00 00 00 00 00 CB')
        # response = self.waitSerial(validate_format=True)
        if response.startswith("FD"):
            if response[3:5] == "31":
                self.navigation_sequence = ['point1', 'point2', 'end']
            elif response[3:5] == "33":
                self.navigation_sequence = ['point2', 'point1', 'end']
        else:
            self.get_logger().warn("Unexpected data; using default sequence.")
            self.navigation_sequence = ['point1', 'point2', 'end']
        
        # 导航其他点
        for point_key in self.navigation_sequence:
            point = self.points[point_key]
            self.get_logger().info(f"Navigating to {point_key}: {point}")
            self.goToPose(self._getGoalPoint(point))
            
            i = 0
            while not self.isTaskComplete():
                i += 1
                feedback = self.getFeedback()
                if feedback and i % 5 == 0:
                    self.get_logger().info("waiting")
                    
            # 发送任务完成信号
            time.sleep(1)
            # self.send_data('FD 04 44 00 00 00 00 00 CB')
            if point_key != "end":
                # cmd = 'FD 04 44 0A 19 00 00 00 CB'
                # cmd = 'FD 04 3A 0A 19 00 00 00 CB'
                # self.get_logger().info(f"Sent: {cmd}")
                # self.serial_port.write(bytes.fromhex(cmd))
                while True:
                    # 检查串口缓冲区中是否有数据
                    cmd = 'FD 04 44 0A 19 00 00 00 CB'
                    self.get_logger().info(f"Sent: {cmd}")
                    self.serial_port.write(bytes.fromhex(cmd))
                    if self.serial_port.in_waiting > 0:
                        # 读取当前缓冲区中的所有字节
                        response = self.serial_port.read(self.serial_port.in_waiting)
                        
                        # 将接收到的字节转换成十六进制字符串格式进行打印while True:
                        response_hex = ' '.join(f'{byte:02X}' for byte in response)
                        self.get_logger().info(f"Received data: {response_hex}")
                        
                        # 执行所需的操作
                        self.get_logger().info('going to the next point')
                        break
                    
                    # 小延时，避免频繁占用 CPU 资源
                    time.sleep(0.1)
                # time.sleep(30)
                self.get_logger().info("go to next point")

def main():
    rclpy.init()
    navigator = PointPublish()
    navigator.run_navigation()
    navigator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()