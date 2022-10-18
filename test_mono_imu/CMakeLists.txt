cmake_minimum_required(VERSION 3.16)
project(mono_inertial_euroc)

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

add_executable(${PROJECT_NAME}
    ./main.cpp
)
target_link_libraries(${PROJECT_NAME}
    Orbslam3
    ${OpenCV_LIBS}
)
target_compile_definitions(${PROJECT_NAME}
    PUBLIC -DCOMPILEDWITHC11
)