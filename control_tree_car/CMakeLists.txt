cmake_minimum_required(VERSION 3.5)

enable_testing()

project(control_tree_car)

catkin_package()

# MLR
set(MLR_LIBRARIES_DIR "externals/rai/lib")
set(MLR_INCLUDE_DIR "externals/rai/rai")

link_directories(${MLR_LIBRARIES_DIR})
include_directories(${MLR_INCLUDE_DIR})

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# ROS
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs nav_msgs tf)
include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(externals)
add_subdirectory(applications)
