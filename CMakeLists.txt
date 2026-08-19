cmake_minimum_required(VERSION 3.20)

project(cpp_trajectory_tools LANGUAGES CXX)

include(CTest)

function(enable_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(trajectory_tools src/trajectory.cpp src/io.cpp)
target_include_directories(trajectory_tools PUBLIC include)
target_compile_features(trajectory_tools PUBLIC cxx_std_17)
enable_project_warnings(trajectory_tools)

add_executable(trajectory_demo src/main.cpp)
target_link_libraries(trajectory_demo PRIVATE trajectory_tools)
enable_project_warnings(trajectory_demo)

add_executable(trajectory_tools_tests tests/test_trajectory.cpp)
target_link_libraries(trajectory_tools_tests PRIVATE trajectory_tools)
enable_project_warnings(trajectory_tools_tests)

add_test(NAME trajectory_tools_tests COMMAND trajectory_tools_tests)
