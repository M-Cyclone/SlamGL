cmake_minimum_required(VERSION 3.16)
project(Orbslam3)

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj")
endif()

add_definitions(-DCOMPILEDWITHC11)

find_package(OpenCV 4 REQUIRED)

find_package(Threads REQUIRED)

    file(GLOB slam_camera_files  CONFIGURE_DEPENDS ./camera_models/*.h ./camera_models/*.cpp)
    file(GLOB slam_core_files    CONFIGURE_DEPENDS ./core/*.h ./core/*.cpp)
    file(GLOB slam_feature_files CONFIGURE_DEPENDS ./feature/*.h ./feature/*.cpp)
    file(GLOB slam_frame_files   CONFIGURE_DEPENDS ./frame/*.h ./frame/*.cpp)
    file(GLOB slam_map_files     CONFIGURE_DEPENDS ./map/*.h ./map/*.cpp)
    file(GLOB slam_solver_files  CONFIGURE_DEPENDS ./solver/*.h ./solver/*.cpp)
    file(GLOB slam_threads_files CONFIGURE_DEPENDS ./threads/*.h ./threads/*.cpp)
    file(GLOB slam_utils_files   CONFIGURE_DEPENDS ./utils/*.h ./utils/*.cpp)

    add_library(${PROJECT_NAME} STATIC
        ${slam_camera_files}
        ${slam_core_files}
        ${slam_feature_files}
        ${slam_frame_files}
        ${slam_map_files}
        ${slam_solver_files}
        ${slam_threads_files}
        ${slam_utils_files}
    )
target_include_directories(Orbslam3
    # PUBLIC camera_models
    # PUBLIC core
    # PUBLIC feature
    # PUBLIC frame
    # PUBLIC map
    # PUBLIC solver
    # PUBLIC threads
    # PUBLIC utils
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    # PUBLIC ${project_dir}
    PUBLIC ${external_dir}/Sophus
    PUBLIC ${external_dir}/eigen3
    PUBLIC ${external_dir}/g2o
    PUBLIC ${external_dir}/DBoW2
)
target_link_libraries(Orbslam3
    ${OpenCV_LIBS}
    DBoW2
    g2o
    Threads::Threads
)