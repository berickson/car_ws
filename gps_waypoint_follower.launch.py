import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    #gps_wpf_dir = get_package_share_directory(
    #    "nav2_gps_waypoint_follower_demo")
    #launch_dir = os.path.join(gps_wpf_dir, 'launch')
    # params_dir = os.path.join(gps_wpf_dir, "config")

    behavior_tree_path = os.path.join(
        bringup_dir,
        'behavior_trees',
        'navigate_w_replanning_time2.xml')
    params_dir = "."
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, 
        root_key="", 
        param_rewrites={'default_nav_to_pose_bt_xml': behavior_tree_path}, 
        convert_types=True
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    return ld
