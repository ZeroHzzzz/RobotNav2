import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'map_build'
    pkg_share = FindPackageShare(package=package_name).find(package_name) 

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    map_name = 'turtlebot3_world.yaml'
    urdf_name = 'turtlebot3_waffle.urdf'
    robot_name = 'turtlebot3_waffle'
    robot_sdf_name = "waffle.model"
    world_name = "world_only.model"
    
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    map_yaml_file = os.path.join(pkg_share, 'maps', map_name)
    nav_defaul_config_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav_config_file = os.path.join(pkg_share, 'config', 'nav2.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', urdf_name)
    world_file = os.path.join(pkg_share, 'worlds', world_name)
    robot_sdf_file = os.path.join(pkg_share, 'worlds', robot_sdf_name)
    
    # config
    use_sim_time = 'true'
    use_composition = 'True'
    use_respawn = 'False'
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    
    # start_gazebo_cmd =  ExecuteProcess(
    #     cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
    #     output='screen')
    
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    # 将机器人模型加载到gazebo中
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name,  '-file', urdf_file ], output='screen')
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf_file,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])


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
        launch_arguments={'rviz_config': rviz_config_file,
                          'use_sim_time': use_sim_time}.items())
    
    
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

    transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['-0.0046412', '0' , '0.094079','0','0','0','base_link','laser_frame']
        )
    
    
    ld = LaunchDescription()
    
    ld.add_action(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))
    # ld.add_action(transform_publisher)
    # ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    # ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    return ld