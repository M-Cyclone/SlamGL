cmake_minimum_required(VERSION 3.16)
project("glad")

file(GLOB glad_files CONFIGURE_DEPENDS ./src/*.c ./include/glad/*.h ./include/KHR/*.h)

source_group(glad FILES ${glad_files})

add_library(${PROJECT_NAME} SHARED
    ${glad_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ./include
)