import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments="serial --dev /dev/ttyACM0 -b921600".split())

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('lidar_launch.py')
    )

    nmea_navsat_driver = Node(
      namespace="car/gps",
      package="nmea_navsat_driver",
      executable="nmea_topic_driver",
      )

    car = Node(package="car",
            executable="car")

    car_action_server = Node(package="car_actions",
            executable="car_action_server")

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge")

    # launch conde_detector.launch.py from cone_detector package
    cone_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("cone_detector"), "launch"), 
            "/cone_detector.launch.py"]))

    ld.add_action(micro_ros_agent)
    ld.add_action(lidar_launch)
    ld.add_action(nmea_navsat_driver)
    ld.add_action(car)
    ld.add_action(foxglove_bridge)
    ld.add_action(cone_detector)
    ld.add_action(car_action_server)

    return ld
