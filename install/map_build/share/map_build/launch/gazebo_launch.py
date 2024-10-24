import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_in_model = 'bot'
    package_name = 'map_build'
    urdf_name = "fishbot_base.urdf"
    world_name = "map1.world"
    rviz_name = "config.rviz"
    nav_name = "nav.yaml"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 

    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    world_file_path = os.path.join(pkg_share, f'worlds/{world_name}')
    rviz_config_path = os.path.join(pkg_share, f'config/{rviz_name}')
    nav_conifg_path = os.path.join(pkg_share, f'config/{nav_name}')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 将机器人模型加载到gazebo中
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # 确保使用仿真时间
            'message_queue_size': 100,
            'max_laser_range': 5.0,
        }],
    )

    # 机器人状态发布
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[os.path.join(pkg_share, 'urdf', 'fishbot_base.urdf')]
    )

    # # 启动导航堆栈（路径规划和控制）
    # bringup_launch = Node(
    #     package='nav2_bringup',
    #     executable='navigation_launch.py',
    #     name='nav2_bringup',
    #     output='screen',
    #     parameters=[nav_conifg_path]
    # )

    # 启动 explore_lite 自动探索
    # explore_node = Node(
    #     package='explore_lite',
    #     executable='explore',
    #     name='explore',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True
    #     }]
    # )

    # 可视化
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],  # 使用构建的配置文件路径
        )
    
    ld.add_action(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(slam_toolbox)
    # ld.add_action(bringup_launch)
    # ld.add_action(explore_node)
    ld.add_action(rviz)
    
    return ld
