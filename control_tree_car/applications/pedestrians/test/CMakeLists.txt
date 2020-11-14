cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" project_name ${project_name})

include(CTest)

##################################
# Locate depedencies
##################################
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

##################################
# Define executable
##################################
# flags
add_definitions(-O3 -Wall -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD -Wno-terminate -fPIC -std=c++14)

# utility test lib
file(GLOB_RECURSE utility_lib_src common/*.cpp)
add_library(utility_lib STATIC ${utility_lib_src})
target_link_libraries(utility_lib
                      qp
                      common
                      pthread
                     )

# list all tests
file(GLOB_RECURSE tests test_*.cpp)

# specify target for each test
foreach(test ${tests})
    get_filename_component(test_name ${test} NAME_WE)
    add_executable(${test_name} ${test})
    target_link_libraries(${test_name}
                          qp
                          common
                          utility_lib
                          #komo_lib
			  ros_common_lib
                          gtest
                          pthread
                          )
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
