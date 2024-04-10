#!/bin/bash

while true; do
  SEC=$(date +%s)
  NSEC=$(date +%N)
  ros2 topic pub /car/gps/fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: $SEC, nanosec: $NSEC}, frame_id: 'map'}, status: {status: 0, service: 1}, latitude: 33.802169, longitude: -118.123311, altitude: 0.0, position_covariance: [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001], position_covariance_type: 1}" --once
  sleep 0.1
done
