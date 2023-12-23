#!/bin/bash
ros2 run detection_visualizer detection_visualizer --ros-args -r detection_visualizer/images:=car/oakd/color/image -r detection_visualizer/detections:=car/oakd/color/cone_detections -r detection_visualizer/dbg_images:=car/oakd/color/annotated_image
