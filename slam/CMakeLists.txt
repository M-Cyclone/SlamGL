cmake_minimum_required(VERSION 3.16)
project(Orbslam3)

set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

add_definitions(-DCOMPILEDWITHC11)

find_package(OpenCV 4 REQUIRED)

find_package(Threads REQUIRED)

add_library(Orbslam3 STATIC
    Core/System.cc

    Thread/Tracking.cc
    Thread/LocalMapping.cc
    Thread/LoopClosing.cc

    Feature/ORBextractor.cc
    Feature/ORBmatcher.cc

    Map/Atlas.cc
    Map/Map.cc
    Map/MapPoint.cc

    Frame/Frame.cc
    Frame/KeyFrame.cc
    Frame/KeyFrameDatabase.cc

    Solver/G2oTypes.cc
    Solver/Optimizer.cc
    Solver/OptimizableTypes.cpp
    Solver/Sim3Solver.cc
    Solver/MLPnPsolver.cpp
    Solver/TwoViewReconstruction.cc

    CameraModels/Pinhole.cpp
    CameraModels/KannalaBrandt8.cpp

    Utils/ImuTypes.cc
    Utils/GeometricTools.cc
    Utils/Config.cc
    Utils/Converter.cc
    Utils/Settings.cc
)
target_include_directories(Orbslam3
    # PUBLIC CameraModels
    # PUBLIC Core
    # PUBLIC Feature
    # PUBLIC Frame
    # PUBLIC Map
    # PUBLIC Solver
    # PUBLIC Thread
    # PUBLIC Utils
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