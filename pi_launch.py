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

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge")

    ld.add_action(micro_ros_agent)
    ld.add_action(lidar_launch)
    ld.add_action(nmea_navsat_driver)
    ld.add_action(car)
    ld.add_action(foxglove_bridge)

    return ld
