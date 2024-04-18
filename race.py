#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, Quaternion
from utils.gps_utils import latLonYaw2Geopose
from car_msgs.msg import Update
from car_msgs.action import FollowCone
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu
import time
import math


class RaceNode(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="race_py")

        # see: 
        # https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/robot_navigator.py
        self.navigator = BasicNavigator("basic_navigator")

        self.car_update_sub = self.create_subscription(Update, "/car/update", self.car_update_cb, qos_profile=qos_profile_sensor_data)

        self.gps_pub = self.create_publisher(NavSatFix, '/car/gps/fix', 10)
        self.compass_pub = self.create_publisher(Imu, '/car/compass', 10)

        self.enabled = False

    def publish_gps(self, lat, lon):
        msg = NavSatFix()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        msg.status.status = 0
        msg.status.service = 1
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        msg.position_covariance = [1.0E-6, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 1.0E-6]
        msg.position_covariance_type = 1
        self.gps_pub.publish(msg)
    def publish_compass_degrees(self, yaw_degrees):
        msg = Imu()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')

        yaw_radians = math.radians(yaw_degrees)

        # Compute quaternion
        half_yaw = yaw_radians * 0.5
        sin_half_yaw = math.sin(half_yaw)
        cos_half_yaw = math.cos(half_yaw)
        msg.orientation = Quaternion(x=0.0, y=0.0, z=sin_half_yaw, w=cos_half_yaw)
        msg.orientation_covariance = [1.0E-6, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 1.0E-6]
        self.compass_pub.publish(msg)


    def car_update_cb(self, msg: Update):
        self.enabled = msg.mode == "auto"
    
    def is_enabled(self):
        rclpy.spin_once(self)
        return self.enabled

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')        
        self.feedback = msg.feedback
        return


def main():
    rclpy.init()
    race_node = RaceNode()
    navigator = BasicNavigator("basic_navigator")
    navigator.waitUntilNav2Active(localizer='robot_localization')

    wp_start    =   [33.802174844465256, -118.12336180642738,-90 ]

    # send fake gps point to nav2
    print("setting starting location")
    for _ in range(5):
        race_node.publish_gps(wp_start[0], wp_start[1])
        race_node.publish_compass_degrees(math.pi / 180. * wp_start[2]);
    

    printed = False
    navigator.clearAllCostmaps()
    time.sleep(1)
    while not race_node.is_enabled():
        if not printed:
            print("waiting for auto mode")
            printed = True
    print("auto mode enabled")
    
    try:


        # waypoints

        eldo_start =    [33.822141, -118.0893215]
        eldo_mid =      [33.8221759,-118.0891911]
        eldo_cone1 =    [33.8220967, -118.0891856]
        wp_hall_mid =   [33.802161699111736, -118.12336089799598,-90]
        wp_hall     =   [33.80215158730133, -118.1233545389763, -70]
        wp_couch_back = [33.802170,  -118.123332]

        # routes
        # front door

        route_inside_house = [wp_hall_mid, wp_hall];
        route_through_front_door = [
            [-118.12335692041168, 33.802147026711815],
            [-118.12335630151807, 33.80211912648005],
            [-118.12331916790136, 33.802124293189635],
            [-118.12327893981654, 33.80212790988635],
            [-118.12325975411457, 33.80212790988635],
            [-118.12325418407208, 33.802111893086625],
            [-118.12325294628485, 33.80209380960308],
            [-118.123251089604, 33.802076242790484],
        ]

        route_eldo = [eldo_mid, eldo_cone1]

        route = route_inside_house

        route_geoposes = [latLonYaw2Geopose(wp[0], wp[1], math.pi / 180. * wp[2] ) for wp in route]



        navigator.followGpsWaypoints(route_geoposes)
        print('navigating waypoints')
        while not navigator.isTaskComplete() and race_node.is_enabled():
            print(navigator.getFeedback())
        print("waypoint navigation done")
    except KeyboardInterrupt:
        print("Interrupted")
    navigator.cancelTask()


    cone_follower = ActionClient(race_node, FollowCone, "follow_cone")
    cone_follower.wait_for_server()
    goal_msg = FollowCone.Goal()
    send_goal_future = cone_follower.send_goal_async(goal_msg, race_node._feedbackCallback)
    rclpy.spin_until_future_complete(race_node, send_goal_future)
    goal_handle = send_goal_future.result()
    printed = False
    if not goal_handle.accepted:
        print("Cone goal rejected")
        return
    result_future = goal_handle.get_result_async() 

    while not result_future.result():
        if not race_node.is_enabled():
            print("auto mode disabled, cancelling goal")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(race_node, cancel_future)
            print("goal cancelled")
            break

        if not printed:
            print("waiting for follow cone action to complete")
            printed = True
        
    print("follow cone complete")

    print("backup")
    navigator.backup(backup_dist=1.0, backup_speed = 0.5, time_allowance=10)
    print("backup complete");


    rclpy.shutdown()

if __name__ == "__main__":
    main()
