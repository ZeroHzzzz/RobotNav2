import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nav')

    gazebo_world_path = os.path.join(pkg_share, 'worlds', 'my_world.world')
    urdf_model_path = os.path.join(pkg_share, 'urdf', 'fishbot_base.urdf')

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'robot_name_in_model', '-file', urdf_model_path, '-x', '0', '-y', '0', '-z', '0.1', '-Y', '0'],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_cmd,
        spawn_entity_cmd
    ])
