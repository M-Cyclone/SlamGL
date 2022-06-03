cmake_minimum_required(VERSION 3.16)
project("stb")

file(GLOB stb_files CONFIGURE_DEPENDS ./include/stb/*.h ./src/*.cpp)

add_library(${PROJECT_NAME} SHARED
    ${stb_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ./include
)