cmake_minimum_required(VERSION 3.18.4)

project(DepthCamEKF VERSION 1.0.0 LANGUAGES CXX)

set (TARGET ${PROJECT_NAME})

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SDL_CFLAGS} -ggdb -Wall -std=c++0x -Wno-narrowing -std=c++14")

message(STATUS "REALSENSE_ROOT = $ENV{REALSENSE_ROOT}")

find_package(Boost COMPONENTS thread filesystem system REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(glfw3 REQUIRED)





include_directories(src/main $ENV{REALSENSE_ROOT}/include $ENV{REALSENSE_ROOT}/third-party/imgui $ENV{REALSENSE_ROOT} ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(main src/main/Main.cpp)