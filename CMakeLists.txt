cmake_minimum_required(VERSION 2.8.3)
project(lib_ego_planner)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( 
    include 
    ${EIGEN3_INCLUDE_DIR} 
    ${OpenCV_INCLUDE_DIRS}
)

if((Eigen3_FOUND) AND (DEFINED EIGEN3_INCLUDE_DIR))
 message(STATUS "Found Eigen3: " ${EIGEN3_INCLUDE_DIR})
 INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
else()
 message(FATAL_ERROR "EIGEN3 not found.")
endif()

add_library(lib_ego_planner 
    src/planner_manager.cpp
    src/poly_traj_optimizer.cpp
    src/grid_map.cpp
    src/dyn_a_star.cpp
    )

add_executable(optimize_test
    src/optimize_test.cpp)

target_link_libraries(optimize_test
    lib_ego_planner
    ${OpenCV_LIBS}
    )  

add_executable(main
src/main.cpp)