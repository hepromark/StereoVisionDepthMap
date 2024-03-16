#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

float BASELINE = 130.0;

cv::Mat calc_P(const cv::Mat &fund1, const cv::Mat& fund2, std::vector<cv::Point> points) {

    cv::Mat system = (points[0].x * fund1.row(2).t() - fund1.row(0).t()).t();
    system.push_back((points[0].y * fund1.row(2).t() - fund1.row(1).t()).t());
    system.push_back((points[1].x * fund2.row(2).t() - fund2.row(0).t()).t());
    system.push_back((points[1].y * fund2.row(2).t() - fund2.row(0).t()).t());

    cv::Mat output = cv::Mat();

    cv::SVD::solveZ(system,output);
    return output;
}


