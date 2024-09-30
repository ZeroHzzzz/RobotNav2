# all_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 Gazebo、SLAM 和地图保存的启动文件路径
    map_launch = os.path.join(
        get_package_share_directory('map_build'),  # 替换为你的包名
        'launch',
        'map_launch.py'  # 启动 Gazebo 仿真
    )
    
    slam_launch = os.path.join(
        get_package_share_directory('map_build'),
        'launch',
        'slam_launch.py'  # 启动 SLAM 算法
    )
    
    save_launch = os.path.join(
        get_package_share_directory('map_build'),
        'launch',
        'save_launch.py'  # 启动地图保存节点
    )

    return LaunchDescription([
        # 启动 Gazebo 仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
        ),
        # 启动 SLAM 算法
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
        ),
        # 启动地图保存节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(save_launch),
        ),
    ])
