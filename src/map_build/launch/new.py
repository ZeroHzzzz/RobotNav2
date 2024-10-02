import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'map_build'
    pkg_share = FindPackageShare(package=package_name).find(package_name) 

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    map_name = 'map.yaml'
    urdf_name = 'fishbot_base.urdf'
    robot_name = 'bot'
    world_name = "map1.world"
    
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    map_yaml_file = os.path.join(pkg_share, 'maps', map_name)
    nav_defaul_config_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    urdf_file = os.path.join(pkg_share, 'urdf', urdf_name)
    world_file = os.path.join(pkg_share, 'worlds', world_name)
    
    # config
    use_sim_time = 'True'
    use_composition = 'True'
    use_respawn = 'False'
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 将机器人模型加载到gazebo中
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name,  '-file', urdf_file ], output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)


    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        launch_arguments={'rviz_config': rviz_config_file}.items())
    
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'slam': 'True',
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': nav_defaul_config_file,
                          'autostart': 'True',
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    ld = LaunchDescription()
    
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    return ld