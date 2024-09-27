import os
from navigation_launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    package_name = 'nav'
    robot_name_in_model = 'fishbot'
    urdf_name = "fishbot_base.urdf"
    
    # 获取package路径和URDF路径
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'world/map1.world')
    amcl_yaml_path = os.path.join(pkg_share, 'config', 'amcl.yaml')  # AMCL参数文件
    map_yaml_path = os.path.join(pkg_share, 'config', 'mapserver.yaml')  # Map server参数文件

    # 定义launch参数: mode (map 或 slam)
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='map',  # 默认为定位模式
        description='选择运行模式: "map" 表示使用已有地图定位, "slam" 表示建图'
    )

    # 获取mode参数
    mode = LaunchConfiguration('mode')
    
    ld = LaunchDescription()
    ld.add_action(declare_mode_arg)

    # Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path, '-x', '0', '-y', '0', '-z', '0.1', '-Y', '0'],
        output='screen'
    )

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
        condition=IfCondition(LaunchConfiguration('mode').equals('map'))  # 当模式为map时启动
    )

    # SLAM节点 (仅在建图模式下启动)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('mode').equals('slam'))  # 当模式为slam时启动
    )

    # Map Saver节点 (仅在建图模式下启动)
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--save_map_timeout', '5000', '--output', '/home/user/maps/my_map'],
        condition=IfCondition(LaunchConfiguration('mode').equals('slam'))  # 仅在slam模式下保存地图
    )

    # Nav2 Controller
    nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Nav2 Planner
    nav2_planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Nav2 Behavior Tree
    nav2_behavior_tree_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 定义生命周期管理器节点
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},  # 使用仿真时间
            {'autostart': True},  # 自动启动并管理节点
            {'node_names': [
                'map_server', 'amcl', 'nav2_controller', 'nav2_planner', 'slam_toolbox', 'bt_navigator'
            ]}
        ]
    )

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

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')]
    )

    # 向launch描述中添加各节点
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(mapserver_node)
    ld.add_action(amcl_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(map_saver_node)
    ld.add_action(nav2_controller_node)
    ld.add_action(nav2_planner_node)
    ld.add_action(nav2_behavior_tree_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
