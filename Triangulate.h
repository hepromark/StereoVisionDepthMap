#include "opencv2/core/core.hpp"

#ifndef STEREOVISIONDEPTHMAP_TRIANGULATE_H
#define STEREOVISIONDEPTHMAP_TRIANGULATE_H

cv::Mat calc_P(const cv::Mat &camera1, const cv::Mat& camera2, std::vector<cv::Point> points);
double triangulate(cv::Mat point1, cv::Mat point2);

#endif //STEREOVISIONDEPTHMAP_TRIANGULATE_H
