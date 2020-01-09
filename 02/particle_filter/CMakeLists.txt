cmake_minimum_required(VERSION 2.8)
PROJECT(ParticleFilter)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
add_definitions("-std=c++11")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories ("${PROJECT_SOURCE_DIR}")

add_subdirectory(dataLoader)
set(LIBS LaserDataLoader)

add_subdirectory(gui)
set(LIBS ${LIBS} Gui)

add_subdirectory(laserSimulator)
set(LIBS ${LIBS} lasersimulator)

add_executable(ParticleFilter pf_main_init.cpp)
add_executable(ParticleFilterInit pf_main_init.cpp)
target_link_libraries(ParticleFilter ${LIBS} ${VTK_LIBRARIES})

