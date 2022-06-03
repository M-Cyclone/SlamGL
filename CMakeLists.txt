cmake_minimum_required(VERSION 3.16)
project(MonoTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")


find_package(OpenCV 4 REQUIRED)

find_package(Threads REQUIRED)

find_package(OpenGL REQUIRED)


set(project_dir ${CMAKE_CURRENT_SOURCE_DIR})
set(external_dir ${CMAKE_CURRENT_SOURCE_DIR}/external)
set(orbslam3_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_subdirectory(./external)
add_subdirectory(./slam)


add_subdirectory(./test_mono_imu)
add_subdirectory(./test_ui)