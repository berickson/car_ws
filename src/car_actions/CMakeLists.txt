cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(car_actions VERSION 2.8.2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()



find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(car_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  car_msgs
  std_msgs
  nav2_msgs
  vision_msgs
  sensor_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)




add_executable(car_action_server src/cone_follower.cpp)
ament_target_dependencies(car_action_server car_msgs ${dependencies})

add_executable(follow_cone src/follow_cone.cpp)
ament_target_dependencies(follow_cone car_msgs ${dependencies})

add_executable(race src/race.cpp)
ament_target_dependencies(race car_msgs ${dependencies})


add_executable(bt src/bt.cpp)
ament_target_dependencies(bt car_msgs ${dependencies} nav2_msgs nav2_behavior_tree)


#install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
#FILES_MATCHING PATTERN "*.py")

install(TARGETS
        bt
        race
        car_action_server
        follow_cone
        DESTINATION lib/${PROJECT_NAME})
ament_package()

