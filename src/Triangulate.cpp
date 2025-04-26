#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

const float BASELINE = 130.0;

cv::Mat calc_P(const cv::Mat &camera1, const cv::Mat& camera2, std::vector<cv::Point> points) {

    cv::Mat system = (points[0].x * camera1.row(2) - camera1.row(0));
    system.push_back((points[0].y * camera1.row(2) - camera1.row(1)));
    system.push_back((points[1].x * camera2.row(2) - camera2.row(0)));
    system.push_back((points[1].y * camera2.row(2) - camera2.row(1)));

    std::cout << system << std::endl;

    cv::Mat output = cv::Mat();

    cv::SVD::solveZ(system,output);

    output = output / output.at<double>(3,0);

    return output;
}

double triangulate(cv::Mat point1, cv::Mat point2) {
    double x = std::abs(point1.at<double>(0,0) - point2.at<double>(0,0));
    double y = std::abs(point1.at<double>(1,0) - point2.at<double>(1,0));
    double z = std::abs(point1.at<double>(2,0) - point2.at<double>(2,0));
    return std::pow(x*x + y*y + z*z,0.5);
}
