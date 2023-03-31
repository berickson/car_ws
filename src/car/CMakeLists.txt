cmake_minimum_required(VERSION 3.8)
project(car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(car_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# car
add_executable(
  car 
  src/ackermann.cpp
  src/car.cpp
  src/geometry.cpp
  src/kalman.cpp
  src/logger.cpp
  src/lookup_table.cpp
  src/pid.cpp
  src/speedometer.cpp
  src/string_utils.cpp

)

ament_target_dependencies(car rclcpp geometry_msgs std_msgs std_srvs car_msgs tf2 tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  car
  DESTINATION lib/${PROJECT_NAME})

ament_package()
