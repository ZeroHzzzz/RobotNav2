import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file_path = os.path.join(
        get_package_share_directory('map_build'),
        'worlds',
        'map1.world'
    )
    
    urdf_file = os.path.join(
        get_package_share_directory('map_build'),
        'urdf',
        'fishbot_base.urdf'
    )
    
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    return LaunchDescription([
        # 启动 Gazebo 并加载 world 文件
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path],
            output='screen'),

        # 启动 slam_toolbox (或者 gmapping)
        # Node(
        #     package='slam_toolbox',
        #     executable='sync_slam_toolbox_node',  # 适用于slam_toolbox
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': True
        #     }],
        # ),
        
        # # 启动机器人模型
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_description}],
        # ),
    ])
