import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nav')
    # rviz_config_path = os.path.join(pkg_share, 'rviz', 'my_rviz_config.rviz')

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d']
    )

    return LaunchDescription([
        rviz2_node
    ])
