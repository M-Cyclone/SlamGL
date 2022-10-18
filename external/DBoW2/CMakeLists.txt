cmake_minimum_required(VERSION 3.16)
project(DBoW2)

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

find_package(OpenCV 4 REQUIRED)


file(GLOB_RECURSE dbow2_files CONFIGURE_DEPENDS *.h *.cpp *.hpp)
add_library(DBoW2 STATIC ${dbow2_files})
target_include_directories(DBoW2 PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})