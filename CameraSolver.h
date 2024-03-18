//
// Created by Humperdink2 on 2024-03-17.
//
#include "opencv2/core.hpp"
#ifndef STEREOVISIONDEPTHMAP_CAMERASOLVER_H
#define STEREOVISIONDEPTHMAP_CAMERASOLVER_H

cv::Mat solve_camera2(cv::Mat & fund,  cv::Mat & K1, cv::Mat & K2);

int get_rank(const cv::Mat& mat);

void rowReduce(cv::Mat& matrix);
#endif //STEREOVISIONDEPTHMAP_CAMERASOLVER_H
