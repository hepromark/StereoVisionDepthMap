//
// Created by Humperdink2 on 2024-03-17.
//
#include "opencv2/core.hpp"
#ifndef STEREOVISIONDEPTHMAP_CAMERASOLVER_H
#define STEREOVISIONDEPTHMAP_CAMERASOLVER_H

void solve_camera2(const cv::Mat &fund, const cv::Mat &K1, const cv::Mat &K2, cv::Mat &camera);

#endif //STEREOVISIONDEPTHMAP_CAMERASOLVER_H
