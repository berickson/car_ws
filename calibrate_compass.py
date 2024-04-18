#!/usr/bin/env python3

# import all the ros stuff, including imu and gps vel messages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from car_msgs.msg import Update
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
import math

class CompassCalibration(Node):
  def __init__(self):

    super().__init__(node_name="compass_calibration")

    self.update_sub = self.create_subscription(Update, "/car/update", self.update_cb, qos_profile=qos_profile_sensor_data)
    print("Calibrating compass")
  
    self.mag_y_max = -1.0E6
    self.mag_y_min = 1.0E6
    self.mag_x_max = -1.0E6
    self.mag_x_min = 1.0E6
  
  def update_cb(self, msg:Update):
    changed = False
    if msg.mag_x > self.mag_x_max:
      self.mag_x_max = msg.mag_x
      changed = True
    if msg.mag_x < self.mag_x_min:
      self.mag_x_min = msg.mag_x
      changed = True
    if msg.mag_y > self.mag_y_max:
      self.mag_y_max = msg.mag_y
      changed = True
    if msg.mag_y < self.mag_y_min:
      self.mag_y_min = msg.mag_y
      changed = True

    if changed:
      print("----------------------------------------------------------")
      print("---- Drive in circles until the numbers stop changing ----")
      print("---- then save the results to your yaml configuration ----")
      print("----------------------------------------------------------")
      print(f"mag_x_max: {self.mag_x_max}")
      print(f"mag_x_min: {self.mag_x_min}")
      print(f"mag_y_max: {self.mag_y_max}")
      print(f"mag_y_min: {self.mag_y_min}")




    
if __name__ == '__main__':
    rclpy.init()
    node = CompassCalibration()
    rclpy.spin(node)

