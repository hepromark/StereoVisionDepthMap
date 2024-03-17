cmake_minimum_required(VERSION 3.25)
project(StereoVisionDepthMap)

set(CMAKE_CXX_STANDARD 17)

# OpenCV
set(ENV{OPENCV_DIR} "C:\\opencv\\build\\x64\\vc16\\lib")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(StereoVisionDepthMap PointSelection.cpp PointSelection.h IntrinsicSolver.cpp IntrinsicSolver.h Triangulate.cpp Triangulate.cpp FundamentalSolver.cpp FundamentalSolver.h Distortion.cpp Distortion.h)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)
target_link_libraries(StereoVisionDepthMap ${OpenCV_LIBS})