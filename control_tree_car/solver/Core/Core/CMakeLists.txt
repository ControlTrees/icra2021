cmake_minimum_required (VERSION 2.8.12)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)
add_definitions(-DRAI_EIGEN=1)

#################################
# Depedencies
#################################
find_package(Eigen3 REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

#################################
# Define executable
#################################

file(GLOB_RECURSE SOURCES *.cpp *.h *.tpp)

add_library(${project_name} ${SOURCES})
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(${project_name}
${BLAS_blas_LIBRARY}
${LAPACK_lapack_LIBRARY}
#gcov
)

#add_subdirectory(test)

