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

# common utilities
file(GLOB obstacle_common_src "nodes/obstacle_common.h" "nodes/obstacle_common.cpp")
add_library(obstacle_common_lib STATIC ${obstacle_common_src})
target_include_directories(obstacle_common_lib PUBLIC ".")
target_link_libraries(obstacle_common_lib
${catkin_LIBRARIES}
common
)

# Obstacles
add_executable(obstacle_avoidance_tree_node nodes/obstacle_avoidance_tree_node.cpp)
target_link_libraries(obstacle_avoidance_tree_node
komo
obstacle_common_lib
)

add_executable(obstacle_popping_control_node nodes/obstacle_popping_control_node.cpp)
target_link_libraries(obstacle_popping_control_node
komo
obstacle_common_lib
)

add_executable(obstacle_avoidance_evaluation_node nodes/obstacle_avoidance_evaluation_node.cpp)
target_link_libraries(obstacle_avoidance_evaluation_node
komo
obstacle_common_lib
)
