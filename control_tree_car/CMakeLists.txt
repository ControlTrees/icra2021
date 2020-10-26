cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Release)

enable_testing()

project(control_tree_car)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs nav_msgs sensor_msgs tf)

## Declare a catkin package
catkin_package()

#add_subdirectory(solver)
add_subdirectory(applications)

