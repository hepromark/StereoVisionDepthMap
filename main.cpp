#include "IntrinsicSolver.h"
#include "opencv2/core.hpp"

int main () {
    const cv::Size PATTERN = {7, 5};
    const double CHESS_SQUARE = 26;  // mm
    const cv::Size IMAGE_SIZE = {1920, 1080};

    IntrinsicSolver intrinsic_solver = IntrinsicSolver(PATTERN, CHESS_SQUARE, IMAGE_SIZE);

    intrinsic_solver.calibrate("C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\calibrated_txt",
                               "C:\\Users\\Humperdink2\\Documents\\github\\StereoVisionDepthMap\\K2.txt");
}
