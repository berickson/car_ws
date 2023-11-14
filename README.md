

## Installing on Desktop

- Install ROS Iron
```
sudo apt install ros-iron-desktop-full
```

- Install Colcon
```
sudo apt install python3-colcon-common-extensions
```
- Clone this repo including submodules

```bash
git clone https://github.com/berickson/car_ws.git --recurse-submodules
```

- Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- Prepare your udev rules for your lidar

Copy this text into /etc/udev/rules.d/99-rplidar-udev.rules to expose lidar on /dev/rplidar
```text
# expose lidar on /dev/rplidar
#
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
```
logout and back in to apply rules, or
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Building
need to fix this, but for now...
```
colcon build --packages-skip nav2_simple_commander nav2_util nav2_behavior_tree
```

## Run the simulation
```
source install/setup.bash
ros2 launch sim_car bringup.launch.py
```
Wait for the costmaps to show in RVIZ, then use the [2D Goal Pose] button in RVIZ to navigate.

## Todo
- Fix Nav2 build errors, or better yet make it so Nav2 isn't required as a submodule. I believe I had to add it to get header files for nav2_line_following_controller to work.
- Dockerize. Ros builds and nodes should run from Docker containers. This is most important for the running on the cars where everything is headless, but less important for the simulator, RViz etc.
- Strategy for single codebase and multiple different cars. Most of the code should say the same with a simple way to swap parameters for specific cars. The cars will likely have different IMUs, Sensors, etc., that will need to be dealt with.