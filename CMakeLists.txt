cmake_minimum_required(VERSION 3.20)

project(cpp_trajectory_tools VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

function(enable_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(trajectory_tools src/trajectory.cpp src/io.cpp)
target_include_directories(trajectory_tools
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(trajectory_tools PUBLIC cxx_std_17)
enable_project_warnings(trajectory_tools)

install(TARGETS trajectory_tools
    EXPORT trajectory_toolsTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(EXPORT trajectory_toolsTargets
    FILE trajectory_toolsTargets.cmake
    NAMESPACE trajectory_tools::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/trajectory_tools")

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/trajectory_toolsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/trajectory_toolsConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/trajectory_tools")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/trajectory_toolsConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/trajectory_toolsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/trajectory_toolsConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/trajectory_tools")

add_executable(trajectory_demo src/main.cpp)
target_link_libraries(trajectory_demo PRIVATE trajectory_tools)
enable_project_warnings(trajectory_demo)

install(TARGETS trajectory_demo
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

add_executable(trajectory_tools_tests tests/test_trajectory.cpp)
target_link_libraries(trajectory_tools_tests PRIVATE trajectory_tools)
enable_project_warnings(trajectory_tools_tests)

add_test(NAME trajectory_tools_tests COMMAND trajectory_tools_tests)
