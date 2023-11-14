

## Installing

- Install ROS Iron

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
