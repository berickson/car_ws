#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, Quaternion, Twist
from utils.gps_utils import latLonYaw2Geopose
from car_msgs.msg import Update
from car_msgs.action import FollowCone
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu
from vision_msgs.msg import Detection2DArray
import time
import math
import signal
import sys

cancel = False

def signal_handler(sig, frame):
    global cancel
    cancel = True
    print('Ctrl-C detected, setting cancel to True')

# Set the signal handler
signal.signal(signal.SIGINT, signal_handler)


# returns degrees heading between two points in ENU degrees
def get_bearing(lat1, lon1, lat2, lon2):
    dLon = (lon2 - lon1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) \
        - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    bearing = math.atan2(y,x)
    bearing = math.degrees(bearing)

    return bearing


# adds yaws to all waypoints in route that don't have one yet
def add_yaws_to_route(route):
    # start and end points point to connected point
    if len(route[0]) < 3:
        lat1, lon1 = route[0][0], route[0][1]
        lat2, lon2 = route[1][0], route[1][1]
        bearing = get_bearing(lat1, lon1, lat2, lon2)
        route[0].append(bearing)
    if len(route[-1]) < 3:
        lat1, lon1 = route[-2][0], route[-2][1]
        lat2, lon2 = route[-1][0], route[-1][1]
        bearing = get_bearing(lat1, lon1, lat2, lon2)
        route[-1].append(bearing)

    # other points use bearing from previous to next point
    for i in range(2, len(route)-1):
        if len(route[i]) == 3:
            continue
        lat1, lon1 = route[ADD-1][0], route[i-1][1]
        lat2, lon2 = route[i+1][0], route[i+1][1]
        bearing = get_bearing(lat1, lon1, lat2, lon2)
        route[i].append(bearing)


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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cone_detection_sub = self.create_subscription(Detection2DArray, '/car/oakd/color/cone_detections', self.cone_detection_cb, qos_profile=qos_profile_sensor_data)

        self.enabled = False
    
    def back_up(self, velocity, seconds):
        print("backing up")
        global cancel
        if cancel: return

        twist = Twist()
        twist.linear.x = -velocity
        twist.angular.z = 0.0
        waited = 0.0
        while waited < seconds:
            if cancel: break
            self.cmd_vel_pub.publish(twist)
            #rclpy.spin_once(self)
            time.sleep(0.1)
            waited += 0.1
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        #rclpy.spin_once(self)
        

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

    def cone_detection_cb(self, msg: Detection2DArray):
        try:
            x_fov_degrees = 69;
            x_resolution = 320;
            if len(msg.detections) == 0:
                self.cone_in_sight = False
                return
            
            # find the index of the closest (widest) detection
            max_width = 0
            max_width_index = 0
            for i in range(len(msg.detections)):
                width = msg.detections[i].bbox.size_x
                if width > max_width:
                    max_width = width
                    max_width_index = i
            

            x =  msg.detections[max_width_index].bbox.center.position.x
            if x > x_resolution * 0.2 and x < x_resolution * 0.8:
                self.cone_in_sight = True
        except Exception as e:
            print(e)
            self.cone_in_sight = False
            print("error in cone detection callback")
        


    def car_update_cb(self, msg: Update):
        self.enabled = msg.mode == "auto"
    
    def is_enabled(self):
        rclpy.spin_once(self)
        return self.enabled

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')        
        self.feedback = msg.feedback
        return


def test_bearing():
    wp = [33,-118]
    west = [33, -118.1]
    east = [33, -117.9]
    north = [33.1, -118]
    south = [32.9, -118]
    print(f"expect -90, actual {get_bearing(wp[0], wp[1], south[0], south[1])}")
    print(f"expect 90, actual {get_bearing(wp[0], wp[1], north[0], north[1])}")
    print(f"expect 0, actual {get_bearing(wp[0], wp[1], east[0], east[1])}")
    print(f"expect 180, actual {get_bearing(wp[0], wp[1], west[0], west[1])}")


def main():
    global cancel
    print("race code started")
    rclpy.init()
    race_node = RaceNode()
    navigator = BasicNavigator("basic_navigator")
    navigator.waitUntilNav2Active(localizer='robot_localization')

    wp_start    =   [33.802174844465256, -118.123360897995988,-90 ]

    # # send fake gps point to nav2
    print("setting starting location")
    for _ in range(5):
        race_node.publish_gps(wp_start[0], wp_start[1])
        race_node.publish_compass_degrees(math.pi / 180. * wp_start[2])
        time.sleep(0.1)
    

    printed = False
    navigator.clearAllCostmaps()
    time.sleep(1)
    try:
        while not cancel and not race_node.is_enabled():
            if not printed:
                print("waiting for auto mode")
                printed = True
    except KeyboardInterrupt:
        pass
    
    if cancel: 
        print("cancelling")
        return

    print("auto mode enabled")
    

    # waypoints

    eldo_start =    [33.822141, -118.0893215]
    eldo_mid =      [33.8221759,-118.0891911]
    eldo_cone1 =    [33.8220967, -118.0891856]
    wp_hall_mid =   [33.80217, -118.12336089799598]
    wp_hall     =   [33.802165, -118.12336089799598]
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

    add_yaws_to_route(route)

    # print route
    print("route: ")
    for wp in route:
        print(wp)

    route_geoposes = [latLonYaw2Geopose(wp[0], wp[1], math.pi / 180. * wp[2] ) for wp in route]



    try:
        navigator.followGpsWaypoints(route_geoposes)
    except KeyboardInterrupt:
        pass
    print('navigating waypoints')
    while not cancel and not navigator.isTaskComplete() and race_node.is_enabled():
        if race_node.cone_in_sight:
            print("cone in sight, cancelling waypoint navigation")
            break
        print(navigator.getFeedback())
    print("waypoint navigation done")
    navigator.cancelTask()

    if cancel: return

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

    try:
        while not cancel and not result_future.result():
            if not race_node.is_enabled():
                print("auto mode disabled, cancelling goal")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(race_node, cancel_future)
                print("goal cancelled")
                break

            if not printed:
                print("waiting for follow cone action to complete")
                printed = True
    except KeyboardInterrupt:
        pass
    
    # cancel goal if not complete
    if not result_future.result():
        print("cancelling goal")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future

    print("follow cone complete")

    if not cancel:
        race_node.back_up(velocity=0.5, seconds=3.0)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
