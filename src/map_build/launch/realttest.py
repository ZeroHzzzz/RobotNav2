import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('map_build')
    launch_dir = os.path.join(bringup_dir, 'launch')
    scan_driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')
                     
    # Create the launch configuration variables
    slam = 'False'
    namespace = ''
    use_namespace = 'False'
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'map3.yaml')
    use_sim_time = False
    nav_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    collision_params_file = os.path.join(bringup_dir, 'config', 'collision.yaml')
    autostart = 'True'
    use_composition = 'True'
    use_respawn = 'False'
    urdf_file = os.path.join(bringup_dir, 'urdf', 'new.urdf')
    # urdf_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    tmp_yaml = os.path.join(bringup_dir, 'config', 'tmp.yaml')
    
    # Launch configuration variables specific to simulation
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    use_simulator = 'True'
    use_robot_state_pub = 'True'
    use_rviz = 'True'
    headless = 'False'
    world = os.path.join(bringup_dir, 'worlds', 'map.world') # sdf
    pose = {'x': '0.00',
            'y': '-2.50',
            'z': '0.00',
            'R': '0.00',
            'P': '0.00',
            'Y': '1.57'}
    robot_name = 'turtlebot3_waffle'
    robot_sdf = os.path.join(bringup_dir, 'worlds', 'robot.sdf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare a launch argument for log level
    log_level = LaunchConfiguration('log_level', default='info')

    # Declare log level as a launch argument
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Log level for all nodes (e.g., debug, info, warn, error, fatal)'
    )

    # Specify the actions
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level])

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[scan_driver_dir],
        arguments=['--ros-args', '--log-level', log_level])

    scan2odom = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 15.0}],
        arguments=['--ros-args', '--log-level', log_level])
    
    # scan2odom = Node(
    #     package='ros2_laser_scan_matcher',
    #     executable='laser_scan_matcher',
    #     name='ros2_laser_scan_matcher',
    #     output='screen',
    #     parameters=[tmp_yaml],
    # )

    nav_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': "False",
                              'params_file': nav_params_file,
                              'container_name': 'nav2_container'}.items())
    nav_container = Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[nav_params_file, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen')
    
    amcl_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': "False",
                              'autostart': autostart,
                              'params_file': nav_params_file,
                              'container_name': 'nav2_container'}.items())


    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav_params_file],
        arguments=['--ros-args', '--log-level', log_level])

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[scan_driver_dir],
        arguments=['--ros-args', '--log-level', log_level])
    
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', FindPackageShare('map_build').find('map_build') + '/config',
            '-configuration_basename', 'backpack_2d_localization.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )
    move_commander = Node(
        package='map_build',
        executable='move',
        name='move_commander',
        output='screen',
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file,
                          'log_level': log_level}.items())
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the log level declaration
    ld.add_action(declare_log_level_cmd)

    ld.add_action(driver_node)
    ld.add_action(scan2odom)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(bringup_cmd)
    ld.add_action(nav_container)
    ld.add_action(nav_cmd)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    # ld.add_action(amcl_cmd)

    ld.add_action(move_commander)
    # ld.add_action(velocity_smoother)
    ld.add_action(rviz_cmd)
    return ld
