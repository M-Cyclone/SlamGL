cmake_minimum_required(VERSION 3.16)
project(MonoTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message("Build type: " ${CMAKE_BUILD_TYPE})

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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