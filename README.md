# Self-Driving RC Car - Ros2 Workspace

This shared Ros2 workspace is a team effort to create self driving RC cars for indoor and outdoor navigations. Our next major goal is to participate in  the [RoboMagellan Competition](http://robogames.net/rules/magellan.php) at [RoboGames 2024](http://robogames.net/index.php).

## Installing on Desktop

1. **Install ROS Iron**
    ```bash
    sudo apt install ros-iron-desktop-full
    ```

2. **Install Colcon**
    ```
    sudo apt install python3-colcon-common-extensions
    ```
3. **Clone the Repository**
    ```bash
    git clone https://github.com/berickson/car_ws.git --recurse-submodules
    ```

4. **Install Dependencies**
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

5. **Prepare udev rules for Lidar**

    Copy the following text into `/etc/udev/rules.d/99-rplidar-udev.rules` to expose lidar on `/dev/rplidar`:
    ```text
    # expose lidar on /dev/rplidar
    #
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
    ```
    To apply the rules, log out and back in, or run:
    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
6. ** Prepare udev rules for Oak-D**
    ```bash
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
    Reboot or unplug and plug camera for rules to take effect

7. Get machine ready for Platformio. This avoids *"Can not find working Python 3.6+ Interpreter. Please install the latest Python 3 and restart VSCode"*
   ```bash
   sudo apt install python3-venv
   ```


## Building the Project
```
colcon build
```


## Equipment
GPS: https://www.amazon.com/Compass-Precision-Receiver-Navigation-Compatible/dp/B08NY9JSZ3

## To-Do List

- **Resolve Nav2 Build Errors**
    Ideally, Nav2 should not be required as a submodule. The current setup was necessary to access header files for the `nav2_line_following_controller`.

- **Implement Docker**
    ROS builds and nodes should run from Docker containers. This is particularly important for headless car operations, but less so for the simulator and RViz.

- **Develop a Single Codebase Strategy**
    Most of the code should remain the same, with a simple method to swap parameters for specific cars. Different cars will likely have varying IMUs, Sensors, etc., which will need to be accommodated.

