#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from utils.gps_utils import latLonYaw2Geopose


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
    yaw = -90 * 3.14/180.;
    # wp = [latLonYaw2Geopose(33.8021086, -118.1236532, yaw)]

    # starting position 33.802178, longitude: -118.123382,
    # hallway
    
    wp = [
        latLonYaw2Geopose(33.8021534, -118.123382, yaw),
        latLonYaw2Geopose(33.8021484, -118.123372, yaw),
          ]
    print()
    print(wp[0])
    navigator.followGpsWaypoints(wp)
    while not navigator.isTaskComplete():
        print(navigator.getFeedback())
        print('waiting to complete')
    print("completed successfully")

if __name__ == "__main__":
    main()
