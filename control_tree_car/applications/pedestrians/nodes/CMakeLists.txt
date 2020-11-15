cmake_minimum_required(VERSION 3.5)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

##################################
# Locate depedencies
##################################

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)
find_package(CGAL)
find_package(osqp REQUIRED)

#################################
# Define executable
#################################
add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++14)

# Ros common utilities
file(GLOB ros_common_src "nodes/pedestrian_common.h" "nodes/pedestrian_common.cpp")
add_library(ros_common_lib STATIC ${ros_common_src})
target_include_directories(ros_common_lib PUBLIC ".")
target_link_libraries(ros_common_lib
${catkin_LIBRARIES}
)

# Pedestrian
add_executable(pedestrian_tree_qp_node nodes/pedestrian_tree_qp_node.cpp)
target_link_libraries(pedestrian_tree_qp_node
qp
common
ros_common_lib
)
