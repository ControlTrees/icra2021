cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate MLR libs
#################################

include_directories( ${MLR_INCLUDE_DIR} )
link_directories(    ${MLR_LIBRARIES_DIR} )

#find Boost
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)

#################################
# Define executable
#################################

file(GLOB_RECURSE SOURCES *.cpp *.h *.tpp)

add_library(${project_name} STATIC ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(
 ${project_name}
 ${Boost_LIBRARIES}
 Core
 Optim
)

add_subdirectory(${project_name}/test)
