cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" project_name ${project_name})

include(CTest)

# list all tests
file(GLOB_RECURSE tests *.cpp)

#find GTest
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

# specify target for each test
foreach(test ${tests})
    get_filename_component(test_name ${test} NAME_WE)
    add_executable(${test_name} ${test})
    target_link_libraries(${test_name}
                          ${GTEST_LIBRARIES}
                          OptimDec
                          pthread
                          gtest
                          )
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
