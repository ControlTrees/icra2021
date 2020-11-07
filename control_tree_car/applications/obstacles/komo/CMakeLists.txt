cmake_minimum_required(VERSION 3.5)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})
##################################
# Locate MLR libs
##################################

#find Eigen
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs nav_msgs tf)
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
#################################
# Define executable
#################################

add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++11)

file(GLOB_RECURSE SOURCES *.cpp *.h)

add_library(${project_name} STATIC ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(${project_name}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
common
pthread
)


