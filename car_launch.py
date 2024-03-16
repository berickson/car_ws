from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_config = LaunchConfiguration('default_config', default=[get_package_share_directory('car'), '/params/default_parameters.yaml'])
    robot_id = EnvironmentVariable('ROBOT_ID')
    car_config = LaunchConfiguration('car_config', default=[get_package_share_directory('car'), '/params/', robot_id, '_parameters.yaml'])

    return LaunchDescription([

        Node(
            package='car',
            executable='car',
            parameters=[default_config, car_config],
        ),
    ])