cmake_minimum_required(VERSION 3.18)
project("g2o")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj")
endif()

file(GLOB g2o_core_files    CONFIGURE_DEPENDS ./g2o/core/*.cpp ./g2o/core/*.hpp ./g2o/core/*.h)
file(GLOB g2o_solvers_files CONFIGURE_DEPENDS ./g2o/solvers/*.h)
file(GLOB g2o_stuff_files   CONFIGURE_DEPENDS ./g2o/stuff/*.h ./g2o/stuff/*.cpp ./g2o/stuff/*.c)
file(GLOB g2o_types_files   CONFIGURE_DEPENDS ./g2o/types/*.h ./g2o/types/*.cpp ./g2o/types/*.hpp)

source_group(core   FILES ${g2o_core_files})
source_group(solver FILES ${g2o_solvers_files})
source_group(stuff  FILES ${g2o_stuff_files})
source_group(types  FILES ${g2o_types_files})

add_library(${PROJECT_NAME} STATIC
    ${g2o_core_files}
    ${g2o_solvers_files}
    ${g2o_stuff_files}
    ${g2o_types_files}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ./g2o/core
    PUBLIC ./g2o/types
    PUBLIC ./g2o/stuff
    PUBLIC ../eigen3
)
if(MSVC)
    target_compile_definitions(${PROJECT_NAME}
        PUBLIC -DWINDOWS
        PUBLIC -D_WINDOWS
    )
endif()