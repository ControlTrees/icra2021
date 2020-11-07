cmake_minimum_required(VERSION 3.5)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

##################################
# Locate Depedencies
##################################

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)
find_package(CGAL)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs nav_msgs visualization_msgs tf)

include_directories(
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
)

#################################
# Define executable
#################################
add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++14)

# Trajectory controller
add_executable(trajectory_controller_node nodes/trajectory_controller_node.cpp)
target_link_libraries(trajectory_controller_node
${catkin_LIBRARIES}
common
ros_common_lib
)

