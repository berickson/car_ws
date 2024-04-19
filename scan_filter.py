#!/usr/bin/env python3

# import all the ros stuff, including imu and gps vel messages
import rclpy
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import LaserScan 
from rclpy.qos import qos_profile_sensor_data, qos_profile_default
import math

class ScanFilter(Node):
  def __init__(self):

    super().__init__(node_name="scan_filter")
    

    self.declare_parameter("enabled", True)
    self.scan_sub = self.create_subscription(LaserScan, "/scan_raw", self.scan_cb, qos_profile=qos_profile_sensor_data)
    self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile=qos_profile_default)
    print("scan_filter node started")
  
  def scan_cb(self, msg:LaserScan):
    # create a copy of the message

    msg_filtered = LaserScan()
    msg_filtered.header = msg.header
    msg_filtered.angle_min = msg.angle_min
    msg_filtered.angle_max = msg.angle_max
    msg_filtered.angle_increment = msg.angle_increment
    msg_filtered.time_increment = msg.time_increment
    msg_filtered.scan_time = msg.scan_time
    msg_filtered.range_min = msg.range_min
    msg_filtered.range_max = msg.range_max
    msg_filtered.ranges = msg.ranges
    msg_filtered.intensities = msg.intensities


    if self.get_parameter("enabled").get_parameter_value().bool_value == True:
      # filter out close points with low intensities
      for i in range(len(msg.ranges)):
        if msg.ranges[i] < 2 and msg.intensities[i] < 4:
          msg_filtered.ranges[i] = float('nan')
          msg_filtered.intensities[i] = 0.0

        # filter out inconsistent points further out
        
        # readings are in pairs for some reason, look at i-1 and i+1 and make sure they are similar
        # use modulus to avoid going over edge
        i_prev_2 = (i-4) % len(msg.ranges) 
        i_prev = (i-2) % len(msg.ranges)
        i_next = (i+2) % len(msg.ranges)
        i_next_2 = (i+4) % len(msg.ranges)


        max_delta = msg.ranges[i] * 0.1 + 0.1;
        prev_2_matches = abs(msg.ranges[i] - msg.ranges[i_prev_2]) < max_delta
        next_matches = abs(msg.ranges[i] - msg.ranges[i_next]) < max_delta
        prev_matches = abs(msg.ranges[i] - msg.ranges[i_prev]) < max_delta
        next_2_matches = abs(msg.ranges[i] - msg.ranges[i_next_2]) < max_delta

        three_in_row = (prev_2_matches and prev_matches) or (next_matches and next_2_matches) or (prev_matches and next_matches)

        if msg.intensities[i] < 4 and not three_in_row:
          msg_filtered.ranges[i] = float('nan')
          msg_filtered.intensities[i] = 0.0
    
    self.scan_pub.publish(msg_filtered)
  


if __name__ == '__main__':
    rclpy.init()
    node = ScanFilter()
    rclpy.spin(node)
