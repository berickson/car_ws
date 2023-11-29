This contains the microcontroller code using micro-ros under PlatformiIO

See: https://github.com/micro-ROS/micro_ros_platformio

To add new message types, you mest add the .msg files to extra_packages folder and force a full rebuild using
```
pio run --target clean_microros 
```