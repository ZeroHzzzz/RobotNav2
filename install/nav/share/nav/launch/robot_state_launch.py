import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nav')

    urdf_model_path = os.path.join(pkg_share, 'urdf', 'fishbot_base.urdf')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
