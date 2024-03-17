#include "CameraSolver.h"
#include "opencv2/core.hpp"
#include <cmath>

void solve_camera(const cv::Mat &fund,const cv::Mat &K1, const cv::Mat &K2, cv::Mat &camera) {
    cv::Mat inter = cv::Mat();
    cv::multiply(K2,fund,inter);
    cv::multiply(inter,K1,inter);

    //solve for translation and rotation matrices
    double ax = -1 * inter.at<double>(1,2);
    double ay = inter.at<double>(0,2);

    double theta = std::asin(inter.at<double>(2,1) / (ax + ay));
    double az = inter.at<double>(0,0) / std::sin(theta);

    //building camera matrix
    double data[3][4] = {{std::cos(theta), -1 * std::sin(theta), 0, ax},
                         {std::sin(theta), std::cos(theta),0,ay},
                         {0,0,1,az}};
    camera = cv::Mat(3,4,CV_64F,data);
}
