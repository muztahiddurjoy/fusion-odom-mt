#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    # Replace 'your_package_name' with your actual package name
    pkg_share = get_package_share_directory('fusion_odom')
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # GPS Publisher Node
        Node(
            package='fusion_odom',
            executable='gps_publisher',
            name='gps_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Sensor Fusion Bridge Node
        Node(
            package='fusion_odom',
            executable='sensor_fusion_bridge',
            name='sensor_fusion_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Navsat Transform Node (converts GPS to odometry)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gps/fix', '/gps/fix'),
                ('/odometry/filtered', '/odometry/local'),
                ('/odometry/gps', '/odometry/gps')
            ]
        ),
        
        # EKF Local Filter (fuses IMU and GPS odometry)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_filter',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odometry/filtered', '/odometry/local')
            ]
        ),
        
        # Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'localization.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])