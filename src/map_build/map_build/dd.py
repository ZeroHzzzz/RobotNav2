from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # 设置机器人的初始位置
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x =  0.00
    initial_pose.pose.position.y = -2.50
    initial_pose.pose.orientation.z = 0.7068
    initial_pose.pose.orientation.w = 0.7073
    navigator.setInitialPose(initial_pose)

    # 等待导航系统完全启动
    navigator.waitUntilNav2Active()

    # 定义路点位置字典
    waypoints = {
        'point1': [2.074988, 2.019314],
        'point2': [-1.907477, 2.047290],
        # 'point3': [-3.6, -4.75]
    }

    # 创建路点的 PoseStamped 列表
    goal_poses = []
    for point, coords in waypoints.items():
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = coords[0]
        goal_pose.pose.position.y = coords[1]
        goal_pose.pose.orientation.z = 0.707
        goal_pose.pose.orientation.w = 0.707  # 假设所有路点都具有相同的方向
        # goal_poses.append(goal_pose)

        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Remaining distance to {point}: {feedback.current_pose}")
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Successfully reached {point}")
        elif result == TaskResult.CANCELED:
            print(f"Task to {point} was canceled")
            break  # 任务取消，停止
        elif result == TaskResult.FAILED:
            print(f"Task to {point} failed")
            break  # 任务失败，停止
    # 获取并平滑路径
    # path = navigator.getPathThroughPoses(goal_poses)
    # smoothed_path = navigator.smoothPath(path)

    # # # 跟随平滑后的路径
    # navigator.followPath(smoothed_path)

    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # 实现一些应用逻辑，可以放在这里
    #     #
    #     ################################################

    #     # 处理反馈信息
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         # print('剩余距离到目标位置: ' +
    #         #       '{0:.3f}'.format(feedback.distance_to_goal) +
    #         #       '\n机器人当前速度: ' +
    #         #       '{0:.3f}'.format(feedback.speed))
    #         print(i)

    # # 根据任务结果做出反应
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('目标到达成功！')
    # elif result == TaskResult.CANCELED:
    #     print('任务被取消！')
    # elif result == TaskResult.FAILED:
    #     print('任务失败！')
    # else:
    #     print('任务返回了无效的状态！')

    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()
