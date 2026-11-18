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

add_executable(${PROJECT_NAME} src/main/Log.cpp
        src/main/Main.cpp src/main/DepthCamSLAM.cpp
        src/main/framegrabber/FileGrabber.cpp
        src/main/framegrabber/UsbCamGrabber.cpp
        src/main/framegrabber/FrameGrabber.cpp
        src/main/improc/improc.cpp
        src/main/improc/SearchMultipleOverlappingEllipses.cpp
        src/main/MotionModel.cpp
        src/main/FeatureModel.cpp
        src/main/PartFeatureModel.cpp
        src/main/FullFeatureModel.cpp
        src/main/Feature.cpp
        src/main/FeatureInitInfo.cpp
        src/main/Camera.cpp
        src/main/graphic/GraphicTool.cpp
        src/main/support/math_util.cpp
        src/main/support/eigen_util.cpp
        src/main/support/pangolin_util.cpp
        src/main/DepthCamFeatureModel.cpp

)

file(GLOB IMGUI_SOURCES "$ENV{REALSENSE_ROOT}/third-party/imgui/*.cpp")

target_link_directories(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARY_DIRS} ${OpenCV_LIB_DIR} ${Boost_LIBRARY_DIRS} $ENV{REALSENSE_ROOT}/build/Release )
target_link_libraries(${PROJECT_NAME} realsense2 ${GLUT_LIBRARIES} ${Pangolin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} OpenGL::GLU )

add_executable(realsense_test src/test/realsense_test.cpp)
target_link_directories(realsense_test PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR})
target_link_libraries(realsense_test realsense2 ${OpenCV_LIBS})

add_executable(create_video src/test/create_video.cpp)
target_link_directories(create_video PUBLIC ${OpenCV_LIB_DIR})
target_link_libraries(create_video ${OpenCV_LIBS})

add_executable(show_pgm src/test/show_pmg.cpp)
target_link_directories(show_pgm PUBLIC ${OpenCV_LIB_DIR})
target_link_libraries(show_pgm ${OpenCV_LIBS})

add_executable(convert_to_pgm src/test/convert_to_pgm.cpp)
target_link_directories(convert_to_pgm PUBLIC ${OpenCV_LIB_DIR})
target_link_libraries(convert_to_pgm ${OpenCV_LIBS})

add_executable(realsense_intrinsic_params src/test/realsense_intrinsic_params.cpp)
target_link_directories(realsense_intrinsic_params PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR})
target_link_libraries(realsense_intrinsic_params realsense2)

add_executable(realsense_to_pgm src/test/realsense_to_pgm.cpp)
target_link_directories(realsense_to_pgm PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR})
target_link_libraries(realsense_to_pgm realsense2 ${OpenCV_LIBS})

add_executable(rs-imshow src/test/rs-imshow.cpp)
target_link_directories(rs-imshow PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR})
target_link_libraries(rs-imshow realsense2 ${OpenCV_LIBS})

add_executable(realtime_data_collector src/test/realtime_data_collector.cpp ${IMGUI_SOURCES})
target_link_directories(realtime_data_collector PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR} ${Boost_LIBRARY_DIRS})
target_link_libraries(realtime_data_collector realsense2 ${OpenCV_LIBS} ${GLUT_LIBRARIES} ${Boost_LIBRARIES} glfw OpenGL::GL OpenGL::GLU)

add_executable(check_saved_depth_info src/test/check_saved_depth_info.cpp ${IMGUI_SOURCES})
target_link_directories(check_saved_depth_info PUBLIC $ENV{REALSENSE_ROOT}/build/Release ${OpenCV_LIB_DIR} ${Boost_LIBRARY_DIRS})
target_link_libraries(check_saved_depth_info realsense2 ${OpenCV_LIBS} ${GLUT_LIBRARIES} ${Boost_LIBRARIES} glfw OpenGL::GL OpenGL::GLU)