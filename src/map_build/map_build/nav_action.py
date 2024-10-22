from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time  # 用于暂停的模块

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = -2.50
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # set our demo's goal poses to follow
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.5
    goal_pose1.pose.position.y = 0.55
    goal_pose1.pose.orientation.w = 0.707
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.5
    goal_pose2.pose.position.y = -3.75
    goal_pose2.pose.orientation.w = 0.707
    # goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.6
    goal_pose3.pose.position.y = -4.75
    goal_pose3.pose.orientation.w = 0.707
    # goal_poses.append(goal_pose3)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    current_waypoint = -1  # 追踪当前的航点
    while not navigator.isTaskComplete():

        # 获取导航反馈
        feedback = navigator.getFeedback()
        if feedback:
            # 检查是否已经到达了新的航点
            if feedback.current_waypoint != current_waypoint:
                current_waypoint = feedback.current_waypoint
                print(f'Arrived at waypoint {current_waypoint + 1}/{len(goal_poses)}')
                
                # 在到达航点后暂停10秒
                time.sleep(10)

        now = navigator.get_clock().now()

        # 取消任务示例
        if now - nav_start > Duration(seconds=600.0):
            navigator.cancelTask()

        # 变更航点示例
        if now - nav_start > Duration(seconds=35.0):
            goal_pose4 = PoseStamped()
            goal_pose4.header.frame_id = 'map'
            goal_pose4.header.stamp = now.to_msg()
            goal_pose4.pose.position.x = 0.0
            goal_pose4.pose.position.y = -2.50
            goal_pose4.pose.orientation.w = 0.0
            goal_poses = [goal_pose4]
            nav_start = now
            navigator.followWaypoints(goal_poses)

    # 根据返回结果处理
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
