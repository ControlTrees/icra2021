cmake_minimum_required(VERSION 3.5)
project(control_tree_car)

set(CMAKE_BUILD_TYPE Release)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs nav_msgs sensor_msgs tf)

## Declare a catkin package
catkin_package()

#################################
# Locate MLR libs
#################################

#find Eigen
find_package(Eigen3 REQUIRED)

# Boost
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

include_directories(
${EIGEN3_INCLUDE_DIR}
"/home/camille/Dev/mlr/rai/rai"
"/home/camille/Dev/mlr/share/projects/17-camille-obsTask/libs/MotionPlanning"
"/home/camille/Dev/mlr/share/projects/17-camille-obsTask/libs/GeometricUtilityTasks"
"/home/camille/Dev/mlr/share/projects/17-camille-obsTask/libs/MathUtility"
"/home/camille/Dev/mlr/share/projects/17-camille-obsTask/libs/CollisionAvoidance"
"/home/camille/Dev/mlr/share/projects/17-camille-obsTask/libs/Optimization/include"
) 
link_directories(
"/home/camille/Dev/mlr/rai/lib"
"/home/camille/Dev/mlr_build/libs/MotionPlanning"
"/home/camille/Dev/mlr_build/libs/GeometricUtilityTasks"
"/home/camille/Dev/mlr_build/libs/MathUtility"
"/home/camille/Dev/mlr_build/libs/CollisionAvoidance"
"/home/camille/Dev/mlr_build/libs/Optimization"
)

#CGAL
find_package(CGAL)

#OSQP
find_package(osqp REQUIRED)

## Build traj planner
add_compile_options(-std=c++11)

include_directories(include ${catkin_INCLUDE_DIRS})

## CORE
file(GLOB core_lib_src "include/control_tree/core/*.h" "src/core/*.cpp")
add_library(core_lib STATIC ${core_lib_src})
target_link_libraries(core_lib
${catkin_LIBRARIES}
 pthread
 )

## QP
file(GLOB qp_lib_src "include/control_tree/qp/*.h" "src/qp/*.cpp")
add_library(qp_lib STATIC ${qp_lib_src})
target_link_libraries(qp_lib
${catkin_LIBRARIES}
core_lib
CGAL::CGAL
osqp::osqp
Core
Optim
Optimization
${Boost_LIBRARIES}
 )

## KOMO
file(GLOB komo_lib_src "include/control_tree/komo/*.h" "src/komo/*.cpp")
add_library(komo_lib STATIC ${komo_lib_src})
target_link_libraries(komo_lib
${catkin_LIBRARIES}
 core_lib
 Core
 Geo
 Kin
 KOMO
 GeometricUtilityTasks
 MathUtility
 CollisionAvoidance
 MotionPlanning
 pthread
 )

## Pedestrian
add_executable(pedestrian_tree_qp_node src/ros/pedestrian_tree_qp_node.cpp src/ros/pedestrian_common.cpp include/control_tree/ros/pedestrian_common.h)
target_link_libraries(pedestrian_tree_qp_node
qp_lib
 )

add_executable(pedestrian_popping_control_node src/ros/pedestrian_popping_control_node.cpp)
target_link_libraries(pedestrian_popping_control_node
core_lib
 )

## Obstacle avoidance
add_executable(obstacle_avoidance_tree_node src/ros/obstacle_avoidance_tree_node.cpp src/ros/obstacle_common.cpp include/control_tree/ros/obstacle_common.h)
target_link_libraries(obstacle_avoidance_tree_node
komo_lib
 )

add_executable(obstacle_avoidance_linear_node src/ros/obstacle_avoidance_linear_node.cpp src/ros/obstacle_common.cpp include/control_tree/ros/obstacle_common.h)
target_link_libraries(obstacle_avoidance_linear_node
komo_lib
 )

add_executable(obstacle_popping_control_node src/ros/obstacle_popping_control_node.cpp src/ros/obstacle_common.cpp include/control_tree/ros/obstacle_common.h)
target_link_libraries(obstacle_popping_control_node
core_lib
 )

## Trajectory controller
add_executable(trajectory_controller_node src/ros/trajectory_controller_node.cpp)
target_link_libraries(trajectory_controller_node
core_lib
 )

add_subdirectory(test)

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

