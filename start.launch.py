from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    node_tf_base_footprint_to_base_link = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "--frame-id base_footprint --child-frame-id base_link --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1".split())

    node_tf_based_link_to_base_laser = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "--frame-id base_link --child-frame-id laser_scanner_link --x .19 --y 0 --z 0.22 --qx 0 --qy 1 --qz 0 --qw 0".split())
    
    car = Node(package="car",
                executable="car")

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge")


    ld.add_action(node_tf_base_footprint_to_base_link)
    ld.add_action(node_tf_based_link_to_base_laser)
    ld.add_action(car)
    ld.add_action(foxglove_bridge)

    return ld
