cmake_minimum_required(VERSION 3.16)
project(test_ui)

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

file(GLOB core_files CONFIGURE_DEPENDS
    ./src/core/*.cpp
    ./src/core/*.h
)
file(GLOB render_files CONFIGURE_DEPENDS
    ./src/render/*.cpp
    ./src/render/*.h
)
file(GLOB utils_files CONFIGURE_DEPENDS
    ./src/utils/*.cpp
    ./src/utils/*.h
)

add_executable(${PROJECT_NAME}
    ./src/main.cpp
    ${core_files}
    ${render_files}
    ${utils_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ./src
)
target_link_libraries(${PROJECT_NAME}
    Orbslam3
    glad
    glfw
    imgui
    spdlog::spdlog
    glm::glm
    stb
)
