cmake_minimum_required(VERSION 3.16)
project("imgui")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB imgui_files CONFIGURE_DEPENDS ./imgui/*.cpp ./imgui/*.h)

set(imgui_backends_files
    ./imgui/backends/imgui_impl_opengl3_loader.h
    ./imgui/backends/imgui_impl_opengl3.h
    ./imgui/backends/imgui_impl_opengl3.cpp
    ./imgui/backends/imgui_impl_glfw.h
    ./imgui/backends/imgui_impl_glfw.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${imgui_files}
    ${imgui_backends_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC .
    PUBLIC ./imgui
    PUBLIC ../glfw/include
)
target_link_libraries(${PROJECT_NAME}
    OpenGL::GL
    glfw
    ${CMAKE_DL_LIBS}
)