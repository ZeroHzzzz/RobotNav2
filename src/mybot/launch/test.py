from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bt_file = os.path.join(
        get_package_share_directory('map_build'),
        'behavior_trees',
        'drive_square_bt.xml'
    )
    bringup_dir = get_package_share_directory('map_build')
    launch_dir = os.path.join(bringup_dir, 'launch')

    nav_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    world = os.path.join(bringup_dir, 'worlds', 'map.world') # sdf
    urdf_file = os.path.join(bringup_dir, 'urdf', 'new.urdf')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'map.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    robot_name = 'bot'

    slam = 'False'
    autostart = 'True'
    use_sim_time = 'True'
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    pose = {'x': '0.00',
            'y': '-2.50',
            'z': '0.00',
            'R': '0.00',
            'P': '0.00',
            'Y': '1.57'}
    
    return LaunchDescription([
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[{
        #         'default_bt_xml_filename': bt_file
        #     }]
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bringup_launch.py')),
            launch_arguments={
                            'slam': slam,
                            'map': map_yaml_file,
                            'use_sim_time': "True",
                            'params_file': nav_params_file,
                            'autostart': autostart}.items()),
        
        # ExecuteProcess(
        #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
        #         '-s', 'libgazebo_ros_factory.so', world],
        #     cwd=[launch_dir], output='screen'),

        # ExecuteProcess(
        #     cmd=['gzclient'],
        #     cwd=[launch_dir], output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'robot_description': robot_description}],
            remappings=remappings),

        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     output='screen',
        #     arguments=[
        #         '-entity', robot_name,
        #         '-file', urdf_file,
        #         '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
        #         '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],]),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rviz_launch.py')),
            launch_arguments={'rviz_config': rviz_config_file}.items()),
    ])
