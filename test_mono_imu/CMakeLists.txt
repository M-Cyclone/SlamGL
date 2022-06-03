cmake_minimum_required(VERSION 3.16)
project(mono_inertial_euroc)

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