#!/usr/bin/env python3

# import all the ros stuff, including imu and gps vel messages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from utils.gps_utils import euler_from_quaternion
import math

class CompassCalibration(Node):
  def __init__(self):

    super().__init__(node_name="compass_calibration")

    self.imu_sub = self.create_subscription(Imu, "/car/compass", self.imu_cb, qos_profile=qos_profile_sensor_data)
    self.vel_sub = self.create_subscription(TwistStamped, "/car/gps/vel", self.vel_cb, qos_profile=qos_profile_sensor_data)
  
  def imu_cb(self, msg:Imu):
    # save the imu message, thy come more frequently than the gps message
    self.compass_msg = msg
  
  def vel_cb(self, msg:TwistStamped):
    # only if going greater than 5 m/s
    velocity = math.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)
    if velocity < 5 or math.isnan(velocity):
      return
    
    # get the velocity from the gps message
    compass_yaw_degrees = euler_from_quaternion(self.compass_msg.orientation)[2] * 180.0 / math.pi
    gps_yaw_degrees = math.atan2(msg.twist.linear.y, msg.twist.linear.x) * 180.0 / math.pi
    delta = gps_yaw_degrees - compass_yaw_degrees
    if delta > 180:
      delta -= 360
    if delta < -180:
      delta += 360
    print(f"v: {velocity:6.1f} compass yaw: {compass_yaw_degrees:6.1f}, gps yaw: {gps_yaw_degrees:6.1f}, delta: {delta:6.1f} degrees")

    
if __name__ == '__main__':
    rclpy.init()
    node = CompassCalibration()
    rclpy.spin(node)

