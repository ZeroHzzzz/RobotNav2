# slam_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 slam_toolbox 节点
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # 同步模式建图
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # 使用仿真时间
            ],
        ),
    ])
