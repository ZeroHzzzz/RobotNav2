import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    set = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    
    # Path to the launch files
    map_build_real_launch_path = os.path.join(
        get_package_share_directory('map_build'), 'launch', 'real.py'
    )
    navigation_cartographer_launch_path = os.path.join(
        get_package_share_directory('map_build'), 'launch', 'navigation2_cartographer.launch.py'
    )

    # Include real.py launch
    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_build_real_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Execute tmp node
    tmp_node = ExecuteProcess(
        cmd=['ros2', 'run', 'map_build', 'tmp'],
        output='screen'
    )

    # Include navigation2_cartographer.launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_cartographer_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    return LaunchDescription([
        set,
        use_sim_time_arg,
        real_launch,
        tmp_node,
        navigation_launch
    ])
