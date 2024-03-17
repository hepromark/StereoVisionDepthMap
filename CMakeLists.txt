cmake_minimum_required(VERSION 3.25)
project(Stereo_Vision_Depth_Map)

set(CMAKE_CXX_STANDARD 17)

# OpenCV
set(ENV{OPENCV_DIR} "C:\\Users\\aleon\\Desktop\\opencv\\build\\x64\\vc16\\lib")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Stereo_Vision_Depth_Map Distortion.cpp Distortion.h)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)
target_link_libraries(Stereo_Vision_Depth_Map ${OpenCV_LIBS})
