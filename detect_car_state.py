#!/usr/bin/env python3

# Standard library imports
import math
import signal
import time

# ROS related imports
import rclpy
from rclpy.action import ActionClient
import rclpy.duration
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# ROS message imports
from car_msgs.msg import Update, Speedometer, Status
from geometry_msgs.msg import Point, PointStamped, Quaternion, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header

from typing import List

class CarStateDetector(Node):

    def __init__(self):
        super().__init__(node_name="detect_car_state_py")

        self.car_update_sub = self.create_subscription(Update, "/car/update", self.car_update_cb, qos_profile=qos_profile_sensor_data)
        self.fr_speedometer_sub = self.create_subscription(Speedometer, "/car/speedometers/fr", self.fr_speedometer_cb, qos_profile=qos_profile_sensor_data)
        self.fl_speedometer_sub = self.create_subscription(Speedometer, "/car/speedometers/fl", self.fl_speedometer_cb, qos_profile=qos_profile_sensor_data)
        self.motor_speedometer_sub = self.create_subscription(Speedometer, "/car/speedometers/motor", self.motor_speedometer_cb, qos_profile=qos_profile_sensor_data)

        #create publisher /car/states that publishes a string array of all the detected states
        self.status_pub = self.create_publisher(Status, "/car/status", qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.check_car_state)


        self.update_msg : Update = None
        self.fr_speedometer_msg = None
        self.fl_speedometer_msg = None
        self.motor_speedometer_msg = None

        self.rear_move_complete_time = None
        self.motor_start_meters = None
        self.fl_start_meters = None
        self.fr_start_meters = None


    def car_update_cb(self, msg: Update):
        self.update_msg = msg

    def fr_speedometer_cb(self, msg: Speedometer):
        self.fr_speedometer_msg = msg

    def fl_speedometer_cb(self, msg: Speedometer):
        self.fl_speedometer_msg = msg

    def motor_speedometer_cb(self, msg: Speedometer):
        self.motor_speedometer_msg = msg

    def check_car_state(self):
        state_strings = self.get_state_strings()
        status_msg = Status()
        status_msg.header = Header()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.states = state_strings
        if "stuck" in state_strings:
            status_msg.stuck = True
        self.status_pub.publish(status_msg)

    def check_for_stuck(self):
        if self.fr_speedometer_msg is None or self.fl_speedometer_msg is None or self.motor_speedometer_msg is None:
            return False
        
        if self.fl_start_meters == None:
            self.motor_start_meters = self.motor_speedometer_msg.meters
            self.fl_start_meters = self.fl_speedometer_msg.meters
            self.fr_start_meters = self.fr_speedometer_msg.meters
            return False
        
        # if the front has moved 5cm, we're good, reset everything
        if abs(self.fr_speedometer_msg.meters - self.fr_start_meters) > 0.05 or abs(self.fl_speedometer_msg.meters - self.fl_start_meters) > 0.05:
            self.motor_start_meters = self.motor_speedometer_msg.meters
            self.fl_start_meters = self.fl_speedometer_msg.meters
            self.fr_start_meters = self.fr_speedometer_msg.meters
            self.rear_move_complete_time = None
            return False
        
        # check to see if rear has completed 15 cm of movement
        if self.rear_move_complete_time is None:
            if abs(self.motor_speedometer_msg.meters - self.motor_start_meters) > 0.15:
                self.rear_move_complete_time = self.get_clock().now()
            return False
        
        # if the motor moved and the front still hasn't moved within 1 second of rear moving, we're stuck
        if (self.get_clock().now() - self.rear_move_complete_time).nanoseconds / 1.0E9 > 1.0:
            return True


    def get_state_strings(self) -> List[str]:

        state_strings = []

        stop_speed = 0.05

        if self.update_msg is None or self.fr_speedometer_msg is None or self.fl_speedometer_msg is None or self.motor_speedometer_msg is None:
            state_strings.append("waiting for messages")
            return state_strings        

        front_speed = (self.fr_speedometer_msg.v_smooth + self.fl_speedometer_msg.v_smooth) / 2
        rear_speed = self.motor_speedometer_msg.v_smooth

        if abs(front_speed) > stop_speed:
            state_strings.append("moving");
        else:
            state_strings.append("stopped")

        # moving forward
        if front_speed > stop_speed:
            state_strings.append("forward")

        pitch = self.update_msg.mpu_deg_pitch
        roll = self.update_msg.mpu_deg_roll

        if 60 < pitch < 120 or -60 > pitch > -120 or 60 < roll < 120 or -60 > roll > -120:
            state_strings.append("carried")

        if self.check_for_stuck():
            state_strings.append("stuck")

        # moving backward
        if front_speed < -1.0 * stop_speed:
            state_strings.append("backward")        

        if abs(rear_speed) > stop_speed and  abs(rear_speed) > 1.2 *  abs(front_speed) + 0.2:
            state_strings.append("wheelspin")


        if abs(front_speed) > stop_speed and  abs(front_speed) > 1.2 *  abs(rear_speed) + 0.2:
            state_strings.append("skid")

        return state_strings
        
def main():
    rclpy.init()
    try:
        node = CarStateDetector()
        rclpy.spin(node)
    finally:
        rclpy.shutdown

if __name__ == "__main__":
    main()
