# save_map_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动地图保存节点
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # 使用仿真时间
                {'save_map_timeout': 5000},  # 超时时间设置为5秒
                {'yaml_filename': 'maps/map1.yaml'},  # 替换为你的地图保存路径
            ]
        )
    ])
