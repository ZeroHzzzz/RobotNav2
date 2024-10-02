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
    urdf_name = "fishbot_base.urdf"
    world_name = "map1.world"
    rviz_name = "config.rviz"
    nav_name = "nav.yaml"
    map_name = "map.yaml"
    robot_name_in_model = 'bot'

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # 获取 nav2_bringup 包的路径
    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
    localization_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    navigation_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    
    os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 设置参数的默认值（你可以根据需要进行修改）
    use_sim_time = 'true'  # 如果要使用仿真时间则改为 'true'
    world_file_path = os.path.join(pkg_share, f'worlds/{world_name}')
    map_file_path = os.path.join(pkg_share, f'maps/{map_name}')
    nav_conifg_path = os.path.join(pkg_share, f'config/{nav_name}')
    nav_default_path = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    rviz_config_file_path = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    autostart = 'true'  # 自动启动导航堆栈
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    ld = LaunchDescription()
    
    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 将机器人模型加载到gazebo中
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')

    # rviz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        launch_arguments={'rviz_config': rviz_config_file_path}.items())

    # robot state
    with open(urdf_model_path, 'r') as infp:
        robot_description = infp.read()
        
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)
    
    # nav
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'slam': True,
                          'map': map_file_path,
                          'use_sim_time': use_sim_time,
                          'params_file': nav_default_path,
                          'autostart': autostart,
                          'use_composition': True, # 组合启动
                          'use_respawn': False}.items()) # 节点崩溃时是否自动重启该节点
    
    
    # Add any conditioned actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
