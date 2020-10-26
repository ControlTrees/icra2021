cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" project_name ${project_name})

# test utility lib
file(GLOB_RECURSE utility_lib_src common/*.cpp)

# list all tests
file(GLOB_RECURSE tests test_*.cpp) 

#find GTest
find_package(GTest REQUIRED)

#find Eigen
find_package (Eigen3 REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# utility test lib
add_library(utility_lib STATIC ${utility_lib_src})
target_link_libraries(utility_lib
                      qp_lib
                      pthread 
                     )

# specify target for each test
foreach(test ${tests})
    get_filename_component(test_name ${test} NAME_WE)
    add_executable(${test_name} ${test})
    target_link_libraries(${test_name}
                          qp_lib
                          core_lib
                          utility_lib
                          komo_lib
			  ros_common_lib
                          gtest
                          pthread
                          )
endforeach()

# copy directory data
#file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/data" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

