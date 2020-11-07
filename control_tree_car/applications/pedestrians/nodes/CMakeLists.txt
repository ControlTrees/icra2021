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
add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++11)

# Ros common utilities
file(GLOB ros_common_src "nodes/common.h" "nodes/common.cpp" "nodes/pedestrian_common.h" "nodes/pedestrian_common.cpp")
add_library(ros_common_lib STATIC ${ros_common_src})
target_include_directories(ros_common_lib PUBLIC ".")
target_link_libraries(ros_common_lib
${catkin_LIBRARIES}
common
)

# Pedestrian
add_executable(pedestrian_tree_qp_node nodes/pedestrian_tree_qp_node.cpp)
target_link_libraries(pedestrian_tree_qp_node
qp
ros_common_lib
)

