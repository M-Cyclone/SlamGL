cmake_minimum_required(VERSION 3.16)
project("glad")

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

file(GLOB glad_files CONFIGURE_DEPENDS ./src/*.c ./include/glad/*.h ./include/KHR/*.h)

source_group(glad FILES ${glad_files})

add_library(${PROJECT_NAME} STATIC
    ${glad_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ./include
)