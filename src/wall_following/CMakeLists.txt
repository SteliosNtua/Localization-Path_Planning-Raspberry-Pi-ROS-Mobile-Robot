cmake_minimum_required(VERSION 2.8.3)
project(wall_following)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## Declare a catkin package
catkin_package()

## Build follower
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(follower src/follower.cpp)
target_link_libraries(follower ${catkin_LIBRARIES})
#add_dependencies(follower wall_following_generate_messages_cpp)
add_dependencies(follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
