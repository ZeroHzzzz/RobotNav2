import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nav')
    config_path = os.path.join(pkg_share, 'config', 'nav_package.yml')

    # 定义launch参数: mode (map 或 slam)
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='map',
        description='选择运行模式: "map" 表示使用已有地图定位, "slam" 表示建图'
    )

    mode = LaunchConfiguration('mode')

    # 加载子launch文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py'))
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'nav2.launch.py')),
        launch_arguments={'mode': mode}.items()
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'robot_state_launch.py'))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rviz.launch.py'))
    )

    return LaunchDescription([
        declare_mode_arg,
        gazebo_launch,
        nav2_launch,
        robot_state_launch,
        rviz_launch
    ])
