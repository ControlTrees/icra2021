cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

#add_definitions(-fprofile-arcs -ftest-coverage -fPIC -O0  -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -std=c++0x)
add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++0x)

enable_testing()

add_subdirectory("Core")
add_subdirectory("Optim")
add_subdirectory("OptimDec")

