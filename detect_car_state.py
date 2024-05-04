#!/usr/bin/env python3

# Standard library imports
import math
import signal
import time

# ROS related imports
import rclpy
from rclpy.action import ActionClient
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# ROS message imports
from car_msgs.msg import Update
from car_msgs.msg import Speedometer
from geometry_msgs.msg import Point, PointStamped, Quaternion, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header



class CarStateDetector(Node):

    def __init__(self):
        super().__init__(node_name="detect_car_state_py")

        self.car_update_sub = self.create_subscription(Update, "/car/update", self.car_update_cb, qos_profile=qos_profile_sensor_data)
        self.fr_speedometer_sub = self.create_subscription(Speedometer, "/car/speedometers/fr", self.fr_speedometer_cb, qos_profile=qos_profile_sensor_data)
        self.fl_speedometer_sub = self.create_subscription(Speedometer, "/car/speedometers/fl", self.fl_speedometer_cb, qos_profile=qos_profile_sensor_data)
        self.motor_speedometer_sub = self.create_subscription(Speedometer, "/car/speedometers/motor", self.motor_speedometer_cb, qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.check_car_state)


        self.update_msg : Update = None
        self.fr_speedometer_msg = None
        self.fl_speedometer_msg = None
        self.motor_speedometer_msg = None
        self.stuck_counter = 0


    def car_update_cb(self, msg: Update):
        self.update_msg = msg

    def fr_speedometer_cb(self, msg: Speedometer):
        self.fr_speedometer_msg = msg

    def fl_speedometer_cb(self, msg: Speedometer):
        self.fl_speedometer_msg = msg

    def motor_speedometer_cb(self, msg: Speedometer):
        self.motor_speedometer_msg = msg



    def check_car_state(self):
        state = []

        stop_speed = 0.05

        if self.update_msg is None or self.fr_speedometer_msg is None or self.fl_speedometer_msg is None or self.motor_speedometer_msg is None:
            state.append("waiting for messages")
            return

        accelerating = abs(self.update_msg.ax) > 0.1 or abs(self.update_msg.ay) > 0.1 or abs(self.update_msg.az) > 0.1
        

        front_speed = (self.fr_speedometer_msg.v_smooth + self.fl_speedometer_msg.v_smooth) / 2
        rear_speed = self.motor_speedometer_msg.v_smooth

        if abs(front_speed) > stop_speed:
            state.append("moving");
        else:
            state.append("stopped")

        # moving forward
        if front_speed > stop_speed:
            state.append("forward")

        
        if abs(rear_speed) > stop_speed and abs(front_speed) < stop_speed:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        if self.stuck_counter > 5:
            state.append("stuck")

        pitch = self.update_msg.mpu_deg_pitch
        roll = self.update_msg.mpu_deg_roll

        if 60 < pitch < 120 or -60 > pitch > -120 or 60 < roll < 120 or -60 > roll > -120:
            state.append("carried")




        # moving backward
        if front_speed < -1.0 * stop_speed:
            state.append("backward")        

        if abs(rear_speed) > stop_speed and  abs(rear_speed) > 1.2 *  abs(front_speed) + 0.2:
            state.append("wheelspin")


        if abs(front_speed) > stop_speed and  abs(front_speed) > 1.2 *  abs(rear_speed) + 0.2:
            state.append("skid")


        
        # log a trace message with state
        self.get_logger().info(f"Car state: {', '.join(state)}")


def main():
    rclpy.init()
    try:
        node = CarStateDetector()
        rclpy.spin(node)
    finally:
        rclpy.shutdown


if __name__ == "__main__":
    main()
