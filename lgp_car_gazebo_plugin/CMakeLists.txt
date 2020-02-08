cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lgp_car_gazebo_plugin)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS gazebo_ros
                                        gazebo_plugins
                                        geometry_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf)

include_directories(${GAZEBO_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS}  ${catkin_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(lgp_car_gazebo_plugin SHARED src/lgp_car.cpp)
target_link_libraries(lgp_car_gazebo_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(lgp_pedestrian_gazebo_plugin SHARED src/lgp_pedestrian.cpp)
target_link_libraries(lgp_pedestrian_gazebo_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
