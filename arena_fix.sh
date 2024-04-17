#!/bin/bash
for i in in {1..5} 
do
  ros2 topic pub /car/gps/fix sensor_msgs/msg/NavSatFix "{header: {stamp: 'now', frame_id: 'base_link'}, status: {status: 0, service: 1}, latitude: 37.32906, longitude: -121.88699, altitude: 0.0, position_covariance: [1.0E-6, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 1.0E-6], position_covariance_type: 1}" --times 1
  ros2 topic pub /car/compass sensor_msgs/msg/Imu "{header: {stamp: 'now', frame_id: 'base_link'}, orientation: {z: 0.7071, w: -0.7071}, orientation_covariance: [1.0E-6, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 1.0E-6]}" --times 1
done
