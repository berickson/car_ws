#!/usr/bin/env python3

# Standard library imports
import math
import signal
import time

# ROS related imports
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# ROS message imports
from car_msgs.msg import Update
from car_msgs.action import FollowCone
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PointStamped, Quaternion, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray

# ROS tf and geodesy imports
import tf2_ros
from geodesy.utm import fromLatLong
from tf2_geometry_msgs import PointStamped

# Local module imports
from nav2_simple_commander.robot_navigator import BasicNavigator
from utils.gps_utils import latLonYaw2Geopose


def great_circle_distance(lat1, lon1, lat2, lon2):
    R = 6371e3
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) * math.sin(delta_phi / 2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

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
    for i in range(1, len(route)-1):
        if len(route[i]) == 3:
            continue
        lat1, lon1 = route[i-1][0], route[i-1][1]
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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cone_in_sight = False
        self.enabled = False

    def prepare_nav(self):
        self.navigator = BasicNavigator("basic_navigator")
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        self.wait_for_utm_frame()
    
    def circle_to_find_cone(self, timeout_seconds = 20.0, speed = 1.5):
        try:
            print("circling to find cone")
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = 1.0 * speed / 1.5
            start_time = time.time()
            while not self.cone_in_sight and time.time() - start_time < timeout_seconds:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self)
                time.sleep(0.01)
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        if self.cone_in_sight:
            print("cone found by circling") 
            return True
        else:
            print("cone not found by circling")
            return False

    def set_fake_location(self, wp):
        # # send fake gps point to nav2
        print("setting starting location")
        for _ in range(5):
            self.publish_gps(wp[0], wp[1])
            self.publish_compass_degrees(math.pi / 180. * wp[2])
            time.sleep(0.1)


    def wait_for_utm_frame(self):
        # Wait for the utm frame to become available
        while not self.tf_buffer.can_transform('utm', 'base_link', rclpy.time.Time()):
            print('Waiting for utm frame...')
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def lat_lon_to_frame(self, lat, lon, frame_id):
        rclpy.spin_once(self)
        geo_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        utm_point = fromLatLong(geo_point.latitude, geo_point.longitude)

        try:
            utm_point_stamped = PointStamped()
            utm_point_stamped.header.stamp = self.get_clock().now().to_msg()
            #utm_point_stamped.header.stamp = self.tf_buffer.get_latest_common_time('utm', frame_id)
            utm_point_stamped.header.frame_id = 'utm'
            utm_point_stamped.point = Point(x=utm_point.easting, y=utm_point.northing, z=0.0)

            point = self.tf_buffer.transform(utm_point_stamped, frame_id)
        except Exception as e:
            print(e)
            print(f"couldn't transform point to {frame_id} frame")
            return None
        return point



    def back_up(self, velocity, seconds):
        print("backing up")
        try:
            twist = Twist()
            twist.linear.x = -velocity
            twist.angular.z = 0.0
            waited = 0.0
            while waited < seconds:
                self.cmd_vel_pub.publish(twist)
                #rclpy.spin_once(self)
                time.sleep(0.1)
                waited += 0.1
        finally:
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)

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
    
    def follow_cone(self, timeout_seconds = 15.0):
        start_time = time.time()

        cone_follower = ActionClient(self, FollowCone, "follow_cone")
        cone_follower.wait_for_server()
        goal_msg = FollowCone.Goal()
        send_goal_future = cone_follower.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        printed = False
        if not goal_handle.accepted:
            print("Cone goal rejected")
            return
        result_future = goal_handle.get_result_async() 

        timed_out = False
        try:
            while not timed_out and not result_future.result():
                if not self.is_enabled():
                    print("auto mode disabled, cancelling goal")
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future)
                    print("goal cancelled")
                    break

                if not printed:
                    print("waiting for follow cone action to complete")
                    printed = True

                if time.time() - start_time > timeout_seconds:
                    timed_out = True
                    print("timed out waiting for follow cone action to complete")
        finally:
            if not result_future.result():
                print("cancelling goal")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                print("goal cancelled")

        print("follow cone complete")

        return not timed_out

    # route is an array of [lat, lon, yaw_degrees] waypoints
    def follow_route_to_cone(self, route):
        add_yaws_to_route(route)

        # print route
        print("route: ")
        for wp in route:
            print(wp)

        route_geoposes = [latLonYaw2Geopose(wp[0], wp[1], math.pi / 180. * wp[2] ) for wp in route]

        try:
            self.navigator.followGpsWaypoints(route_geoposes)
            print('navigating waypoints')
            while not self.navigator.isTaskComplete() and self.is_enabled():
                rclpy.spin_once(self);
                p = self.lat_lon_to_frame(route[-1][0], route[-1][1], 'map')
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                p_car = transform.transform.translation

                # calculate distance from car to goal
                distance = math.sqrt((p.point.x - p_car.x)**2 + (p.point.y - p_car.y)**2)
                print(f"distance to goal: {distance:.2f} meters")        

                if self.cone_in_sight and distance < 5.0:
                    print("cone in sight, cancelling waypoint navigation")
                    break

        finally:
            print("waypoint navigation done")
            self.navigator.cancelTask()


        if not self.cone_in_sight:
            print("cone not in sight, circling to find cone")
            if not self.circle_to_find_cone():
                print("cone not found by circling, exiting")
                return
            print("cone found by circling")
        self.follow_cone();



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

def test_great_circle_distance():
    wp_start    =   [33.802174844465256, -118.123360897995988,-90 ]
    wp_hall     =   [33.802165, -118.12336089799598]
    print(f"expect 0, actual {great_circle_distance(wp_start[0], wp_start[1], wp_start[0], wp_start[1])}")
    print(f"expect few meters, actual {great_circle_distance(wp_start[0], wp_start[1], wp_hall[0], wp_hall[1])}")

def main():

    print("race code started")
    rclpy.init()

    race_node = RaceNode()
    print("race node initialized")

    printed = False
    while not race_node.is_enabled():
        if not printed:
            print("waiting for auto mode")
            printed = True

    race_node.follow_cone();
    return

    # uncomment to test out circling to find cone
    # print("circling to find cone")
    # if race_node.circle_to_find_cone():
    #     print("circle to find cone succeeded")
    #     if race_node.follow_cone():
    #         print("follow cone succeeded")
    #     else:
    #         print("follow cone failed")
    # else:
    #     print("circle to find cone failed")
    # return

    # uncomment to test out follow cone
    # if race_node.follow_cone():
    #     print("follow cone succeeded")
    # else:
    #     print("follow cone failed")
    # return

    wp_start    =   [33.802174844465256, -118.123360897995988,-90 ]
    # uncomment to set fake location 
    # race_node.set_fake_location(wp_start)

    race_node.navigator.clearAllCostmaps()
    time.sleep(1)
    while not race_node.is_enabled():
        if not printed:
            print("waiting for auto mode")
            printed = True

    print("auto mode enabled")
    race_node.navigator.clearAllCostmaps()
    

    # waypoints

    eldo_start =    [33.822141, -118.0893215]
    eldo_mid =      [33.8221759,-118.0891911]
    eldo_cone1 =    [33.8220967, -118.0891856]
    wp_hall_mid =   [33.80217, -118.12336089799598]
    wp_hall     =   [33.802165, -118.12336089799598]
    wp_couch_back = [33.802170,  -118.123332]

    # routes
    # front door
    viola_ave_ne_to_sw = [
        [37.329317561278145, -121.88691031341239 ],
        [37.3291697212461, -121.88711584619394 ],
        [37.32904693884661, -121.88728686201983 ],
        [37.328955478487806, -121.88742492984255 ],
        [37.32887529406364, -121.8875331875672 ],
        [37.32880513269251, -121.887624186814 ],
        [37.32873371843974, -121.88771675501333 ],
        [37.32866856859511, -121.88780147844997 ],
        [37.328587131289325, -121.88791444303223 ],
        [37.328466854653094, -121.88808232095303 ],
        [37.32836912988614, -121.88822195772828 ],
        [37.32829270410687, -121.88832707754787 ],
        [37.32829270410687, -121.88832707754787 ],
        [37.32821627831174, -121.88841964574837 ],
        [37.32810351896528, -121.88858124785905 ],
        [37.328020828777866, -121.88869107453624 ],
    ]

    route_inside_house = [wp_hall_mid, wp_hall];
    route_back_to_desk = [wp_hall_mid, wp_start]
    route_through_front_door = [
        [33.802147026711815, -118.12335692041168],
        [33.80211912648005, -118.12335630151807],
        [33.802124293189635, -118.12331916790136],
        [33.80212790988635, -118.12327893981654],
        [33.80212790988635, -118.12325975411457],
        [33.802111893086625, -118.12325418407208],
        [33.80209380960308, -118.12325294628485],
        [33.802076242790484, -118.123251089604],
    ]

    route_bike_symbol = [
        [37.32900311343151, -121.88735484778778 ],
        [37.329078315121365, -121.88725672110606 ],]

    route_back_bike_symbol = [
        [37.32895099215355, -121.8874171698899 ],
        [37.32888673313748, -121.88751296445777 ],
        [37.32883962545032, -121.88758428538665 ],
        [37.32879590740213, -121.8876420276632 ],
        [37.32875491696356, -121.8876914692842 ],
        [37.32873143012843, -121.88772132607468 ],
    ]

    route_race_direct = [
        [37.327067833426696, -121.8913562990751 ],
        [37.32713947113167, -121.8914061374403 ],
        [37.32717396335999, -121.89144434685359 ],
        [37.327212435460815, -121.89149086266112 ],
        [37.327252234185806, -121.89154734614165 ],
        [37.327288053038295, -121.89158555555497 ],
        [37.32732387189078, -121.89163207136248 ],
        [37.327380916729936, -121.89168689356416 ],
        [37.327427348575746, -121.89177826390035 ],
        [37.32745653430741, -121.89185302144813 ],
    ]

    route_bonus1 = [
        [37.32691659827281, -121.8913396862894 ],
        [37.32692588464197, -121.89137623442387 ],
        [37.32693517101114, -121.89141278255833 ],
    ]

    route_race_upper = [
        [37.32695639699673, -121.89139284720957 ],
        [37.32699221584922, -121.89148089498808 ],
        [37.32707181329919, -121.89154900742051 ],
        [37.32712753151418, -121.89160549090104 ],
        [37.32718324972915, -121.89163539392014 ],
        [37.327240294568306, -121.89166529693925 ],
        [37.32728937966246, -121.89169519995838 ],
        [37.32734111800602, -121.89172676425632 ],
        [37.32738622322767, -121.89177494134267 ],
        [37.3274008160935, -121.89180318308293 ],
        [37.32742602195266, -121.89185136016928 ],
        [37.32745122781182, -121.89191448876518 ],
    ]

    race_node.follow_cone();
    return

    route_eldo = [eldo_mid, eldo_cone1]

    race_node.follow_route_to_cone(route_bonus1)
    race_node.follow_route_to_cone(route_race_upper)
    # race_node.follow_route_to_cone(route_back_bike_symbol)
    race_node.back_up(velocity= 2.0, seconds=4.0)
    race_node.circle_to_find_cone()
    race_node.follow_cone()

    race_node.back_up(velocity=0.5, seconds=3.0)
    race_node.follow_route_to_cone(route_back_to_desk)
    
    print("done");

    rclpy.shutdown()

if __name__ == "__main__":
    main()
