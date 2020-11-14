cmake_minimum_required(VERSION 3.5)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

##################################
# Locate depedencies
##################################
find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

#################################
# Define executable
#################################
add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++14)

file(GLOB_RECURSE SOURCES *.cpp *.h)

add_library(${project_name} STATIC ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(${project_name}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
pthread
)
