#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800') #for A3 is 256000 for C1 460800
    frame_id = LaunchConfiguration('frame_id', default='laser_scanner_link')
    inverted = LaunchConfiguration('inverted', default='true')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard') # for a3 is Sensitivity for c1 is Standard
    log_level = LaunchConfiguration('log_level', default='info')

# see http://bucket.download.slamtec.com/cd82fe93553fea5d15237cb3d6a45a406ef641aa/LR001_SLAMTEC_rplidar_protocol_v2.0_en.pdf
# Standard: max_distance: 25.0 m, Point number: 4.0K
# Express: max_distance: 25.0 m, Point number: 7.9K
# Boost: max_distance: 25.0 m, Point number: 15.9K
# Sensitivity: max_distance: 25.0 m, Point number: 15.9K
# Stability: max_distance: 25.0 m, Point number: 10.0K

    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        DeclareLaunchArgument(
            'log_level',
            default_value=log_level,
            description='Log level (default: info), can be debug, info, warn, error'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            remappings=[('start_motor', 'car/start_scan'),
                        ('stop_motor', 'car/stop_scan')],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])

