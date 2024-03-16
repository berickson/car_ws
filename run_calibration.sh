#!/bin/bash
ros2 run camera_calibration cameracalibrator --fix-principal-point --fix-aspect-ratio --size 6x8 --square 0.0247 --no-service-check --ros-args -r image:=/car/oakd/color/image

