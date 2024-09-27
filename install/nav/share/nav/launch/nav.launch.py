import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

# 读取 YAML 配置文件
def load_yaml_config():
    pkg_share = get_package_share_directory('nav')  # 使用ament来找到包路径
    config_path = os.path.join(pkg_share, 'config', 'nav_package.yml')
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")
    
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    config = load_yaml_config()

    # 从 YAML 配置文件中获取参数
    package_name = config['package_name']
    robot_name_in_model = config['robot_name_in_model']
    urdf_name = config['urdf_name']
    gazebo_world_path = config['gazebo_world_path']
    amcl_yaml_path = config['amcl_yaml_path']
    map_yaml_path = config['map_yaml_path']
    rviz_config_path = config['rviz_config_path']
    mode = config['mode']  # 模式

    # 获取package路径和URDF路径
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, gazebo_world_path)
    amcl_yaml_path = os.path.join(pkg_share, amcl_yaml_path)
    map_yaml_path = os.path.join(pkg_share, map_yaml_path)
    rviz_config_path = os.path.join(pkg_share, rviz_config_path)

    # 定义launch参数: mode (map 或 slam)
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='选择运行模式: "map" 表示使用已有地图定位, "slam" 表示建图'
    )

    # 获取mode参数
    mode = LaunchConfiguration('mode')

    ld = LaunchDescription()
    ld.add_action(declare_mode_arg)

    
    # Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
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
        condition=IfCondition(PythonExpression([mode, " == 'map'"]))  # 当模式为map时启动
    )

    # SLAM节点 (仅在建图模式下启动)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression([mode, " == 'slam'"]))  # 当模式为slam时启动
    )

    # Map Saver节点 (仅在建图模式下启动)
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--save_map_timeout', '5000', '--output', '/home/user/maps/my_map'],
        condition=IfCondition(PythonExpression([mode, " == 'slam'"]))  # 仅在slam模式下保存地图
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
        arguments=['-d', rviz_config_path]
    )

    # 生命周期管理器节点
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'map_server', 'controller_server', 
                'planner_server', 'bt_navigator', 
                'amcl', 'slam_toolbox',
            ]}
        ]
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
