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



include_directories(src/main $ENV{REALSENSE_ROOT}/include ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main/Log.cpp
        src/main/Main.cpp src/main/DepthCamSLAM.cpp
        src/main/framegrabber/UsbCamGrabber.cpp
        src/main/framegrabber/FrameGrabber.cpp
        src/main/improc/improc.cpp
)

target_link_directories(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARY_DIRS} ${OpenCV_LIB_DIR} ${Boost_LIBRARY_DIRS} $ENV{REALSENSE_ROOT}/build/Release )
target_link_libraries(${PROJECT_NAME} realsense2 ${GLUT_LIBRARIES} ${Pangolin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} )

add_executable(realsense_test src/test/realsense_test.cpp)
target_link_directories(realsense_test PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR})
target_link_libraries(realsense_test realsense2 ${OpenCV_LIBS})

add_executable(realsense_intrinsic_params src/test/realsense_intrinsic_params.cpp)
target_link_directories(realsense_intrinsic_params PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR})
target_link_libraries(realsense_intrinsic_params realsense2)