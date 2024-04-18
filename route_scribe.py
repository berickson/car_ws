#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped

class RouteScribe(Node):
  def __init__(self):
    super().__init__(node_name="route_scribe")
    self.clicked_point_sub = self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_cb, qos_profile_sensor_data)
    self.get_logger().info("Route scribe node started")
    
  def clicked_point_cb(self, msg:PointStamped):
    print(f"[{msg.point.y}, {msg.point.x} ],")

if __name__ == '__main__':
  rclpy.init()
  node = RouteScribe()
  rclpy.spin(node)
  rclpy.shutdown()