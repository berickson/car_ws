#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from utils.gps_utils import latLonYaw2Geopose
import subprocess


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

    def mapviz_wp_cb(self, msg: PointStamped):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return

        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wp = [latLonYaw2Geopose(msg.point.y, msg.point.x)]
        self.navigator.followGpsWaypoints(wp)
        if (self.navigator.isTaskComplete()):
            self.get_logger().info("wps completed successfully")


def main():
    rclpy.init()
    navigator = BasicNavigator("basic_navigator")
    navigator.waitUntilNav2Active(localizer='robot_localization')
    try:
        yaw = -90 * 3.14/180.;

        wp_start = latLonYaw2Geopose(33.802208958398154, -118.12336015943343, yaw)
        wp_hall = latLonYaw2Geopose(33.802179596462175, -118.12335425382896, yaw)
        wp_couch_back = latLonYaw2Geopose(33.8021484, -118.123372, yaw)
        # wp = [wp_start, wp_hall, wp_couch_back]
        wp = [wp_hall]
        print()
        print(wp[0])
        navigator.followGpsWaypoints(wp)
        while not navigator.isTaskComplete():
            print(navigator.getFeedback())
            print('waiting to complete')
        print("completed successfully")
    except KeyboardInterrupt:
        print("Interrupted")
    navigator.cancelTask()

    try:
        # Run the ROS2 node
        subprocess.run(["ros2", "run", "car_actions", "race"], check=True)
    except subprocess.CalledProcessError:
        print("Failed to race")

    # ros2 run car_actions race

    rclpy.shutdown()

if __name__ == "__main__":
    main()
