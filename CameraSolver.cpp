#include "CameraSolver.h"
#include "opencv2/core.hpp"
#include <cmath>
#include <iostream>

void solve_camera2(const cv::Mat &fund, const cv::Mat &K1, const cv::Mat &K2, cv::Mat &camera) {
    cv::Mat inter = cv::Mat();
    inter = K2.t() * fund * K1;
    std::cout <<"K1: " <<  K1 << std::endl << "K2: "<< K2 << std::endl;
    std::cout << inter << std::endl;

    //solve for translation and rotation matrices
    double ax = -1 * inter.at<double>(1,2);
    double ay = inter.at<double>(0,2);

    double theta = std::asin(inter.at<double>(2,1) / (ax + ay));
    double az = inter.at<double>(0,0) / std::sin(theta);

    std::cout << "ax: " << ax << std::endl;
    std::cout << "ay: " << ay << std::endl;
    std::cout << "az: " << az << std::endl;
    std::cout << "theta: " << theta << std::endl;

    //building camera matrix
    double data[3][4] = {{std::cos(theta), std::sin(theta), 0, -ax * std::cos(theta) + ay * std::sin(theta)},
                         {-1 * std::sin(theta), std::cos(theta),0,-ax *std::sin(theta) - ay * std::cos(theta)},
                         {0,0,1,-az}};
    camera = cv::Mat(3,4,CV_64F,data);
}

int get_rank(const cv::Mat& mat) {
    int rank = 0;
    cv::Mat U, S, Vt;
    cv::SVDecomp(mat, S, U, Vt);
    rank = cv::countNonZero(S > 0.0001);
    std::cout << "RANK: " << rank << std::endl;
    return rank;
}
