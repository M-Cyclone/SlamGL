cmake_minimum_required(VERSION 3.16)
project("stb")

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

file(GLOB stb_files CONFIGURE_DEPENDS ./include/stb/*.h ./src/*.cpp)

add_library(${PROJECT_NAME} STATIC
    ${stb_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ./include
)