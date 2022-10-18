cmake_minimum_required(VERSION 3.16)

add_subdirectory(./DBoW2)

add_subdirectory(./g2o)

add_subdirectory(./glad)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(./glfw)

add_subdirectory(./imgui)

set(SPDLOG_BUILD_SHARED OFF)
add_subdirectory(./spdlog)

add_subdirectory(./glm)

add_subdirectory(./stb)