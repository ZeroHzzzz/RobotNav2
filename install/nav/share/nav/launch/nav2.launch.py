import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    pkg_share = get_package_share_directory('nav')

    map_yaml_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    amcl_yaml_path = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    # Map Server
    mapserver_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_yaml_path]
    )

    # AMCL节点 (仅在定位模式下启动)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml_path],
        condition=IfCondition(PythonExpression([mode, " == 'map'"]))
    )

    # SLAM节点 (仅在建图模式下启动)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression([mode, " == 'slam'"]))
    )

    return LaunchDescription([
        mapserver_node,
        amcl_node,
        slam_toolbox_node
    ])
