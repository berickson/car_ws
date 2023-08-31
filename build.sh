#!/bin/bash
set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --symlink-install \
        --packages-skip car_gazebo_plugin tf2_web_republisher rosbridge_cpp nav2_system_tests \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic

colcon test --packages-select nav2_line_following_controller --event-handlers console_direct+

# colcon test-result --verbose
