cmake_minimum_required(VERSION 3.5)

enable_testing()

project(control_tree_car)

add_subdirectory(solver)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs nav_msgs sensor_msgs tf)

## Declare a catkin package
catkin_package()

add_subdirectory(applications)

