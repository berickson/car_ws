#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    nav2_params_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params/',
        'nav2_params.yaml')

    behavior_tree_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_time.xml')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        param_rewrites={'default_nav_to_pose_bt_xml': behavior_tree_path, 'use_sim_time': 'False'},
        convert_types=True)

    nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_composition': "0",
            'slam': "1",
            'map': 'map.yaml',
            'params_file': configured_nav2_params,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[nav_bringup_cmd]
        ))
    return ld
