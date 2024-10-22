from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    bringup_dir = get_package_share_directory('map_build')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'default.urdf')
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml') 
    rviz_dir = os.path.join(bringup_dir, 'rviz', 'check_odom.rviz')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',             #设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                )
    
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    # ros2 run ros2_laser_scan_matcher laser_scan_matcher
    # laser2odom = Node(
    #     package='ros2_laser_scan_matcher',
    #     executable='laser_scan_matcher',
    #     name='laser_scan_matcher',
    #     output='screen')
    laser2odom = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 10.0}],
            )
        
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'robot_description': robot_description}],
        remappings=remappings)
    
    return LaunchDescription([
        driver_node,
        #  rviz_node,
        laser2odom,
        robot_state_publisher_cmd,
    ])
