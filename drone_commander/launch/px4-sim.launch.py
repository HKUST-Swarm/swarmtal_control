#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 launch file for drone_commander, converting the original ROS1 launch.

It declares several LaunchArguments, remaps topics, and loads parameters
from a YAML config file: config/drone_commander_config.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # 1) Declare launch arguments (similar to ROS1 <arg> tags)
    vo_imu_topic_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='Drone ID'
    )
    vo_imu_topic_arg = DeclareLaunchArgument(
        'vo_imu_topic',
        default_value='d2vins/imu_propagation',
        description='Topic for visual odometry IMU propagation'
    )
    vo_topic_arg = DeclareLaunchArgument(
        'vo_topic',
        default_value='d2vins/odometry',
        description='Topic for visual odometry'
    )
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Node output (screen or log)'
    )
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            get_package_share_directory('drone_commander'),
            'config',
            'drone_commander_config.yaml'
        ),
        description='Full path to the drone_commander config YAML file'
    )
    
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='Drone ID'
    )
    
    drone_id = LaunchConfiguration('drone_id')
    namespace = [TextSubstitution(text='/uav'), drone_id]

    # 2) Node: drone_commander
    #    - prefix with "nice --20"
    #    - remap topics from /drone_commander/xxx to new names
    #    - load param from YAML file plus some extra inline parameters
    drone_commander_node = Node(
        package='drone_commander',
        executable='drone_commander_node',
        name='drone_commander',
        output=LaunchConfiguration('output'),
        namespace=namespace,
        remappings=[
            ('visual_odometry', LaunchConfiguration('vo_imu_topic')),
            ('visual_odometry_image', LaunchConfiguration('vo_topic')),
        ],
        parameters=[
            LaunchConfiguration('config_path'),   # 加载 YAML
            {'drone_id': drone_id}  # 加载参数
        ],
    )

    odom_transformer_node = Node(
        package='drone_commander',
        executable='px4_odom_converter.py',  # 确保脚本可执行 & 已安装
        name='odom_transformer',
        namespace=namespace,
        output='screen',
        remappings=[
            ('odom_in', 'mavros/odometry/in'),
            ('odom_out', 'mavros/odometry/in_correct')
        ]
    )

    return LaunchDescription([
        vo_imu_topic_arg,
        vo_topic_arg,
        output_arg,
        config_path_arg,
        drone_id_arg,
        drone_commander_node,
        odom_transformer_node
    ])
