cmake_minimum_required(VERSION 3.20)

project(cpp_trajectory_tools LANGUAGES CXX)

include(CTest)

add_library(trajectory_tools src/trajectory.cpp)
target_include_directories(trajectory_tools PUBLIC include)
target_compile_features(trajectory_tools PUBLIC cxx_std_17)

add_executable(trajectory_demo src/main.cpp)
target_link_libraries(trajectory_demo PRIVATE trajectory_tools)

add_executable(trajectory_tools_tests tests/test_trajectory.cpp)
target_link_libraries(trajectory_tools_tests PRIVATE trajectory_tools)

add_test(NAME trajectory_tools_tests COMMAND trajectory_tools_tests)
