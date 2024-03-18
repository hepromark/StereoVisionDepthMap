#include "CameraSolver.h"
#include "opencv2/core.hpp"
#include <cmath>
#include <iostream>

cv::Mat solve_camera2(cv::Mat & fund,  cv::Mat & K1,  cv::Mat & K2) {
    cv::Mat E = K2.t() * fund * K1;
    std::cout <<"K1: " <<  K1 << std::endl << "K2: "<< K2 << std::endl;
    std::cout << "E: " << E << std::endl;

    //SVD decomposition
    cv::SVD decomp = cv::SVD(E);
    cv::Mat U = decomp.u;
    cv::Mat V = decomp.vt.t();

    cv::Mat EEt = E * E.t();
    cv::Mat S(3, 3, CV_64F, cv::Scalar(0));
    S.at<double>(0, 0) = std::pow(EEt.at<double>(0,0),0.5);
    S.at<double>(1, 1) = std::pow(EEt.at<double>(1,1),0.5);
    std::cout << "EEt: " << std::endl << EEt << std::endl;
    std::cout << "S: " << std::endl << S << std::endl;

    cv::Mat W(3, 3, CV_64F, cv::Scalar(0));
    W.at<double>(0, 1) = -1;
    W.at<double>(1, 0) = 1;
    W.at<double>(2, 2) = 1;

    cv::Mat R = U * W.t() * V.t();
    cv::Mat T = U * W * S * U.t();

    std::cout << "Computed rotation:"<< std::endl << R << std::endl;
    std::cout << "Computed translation:" << std::endl << T << std::endl;

    //digesting translation and rotation matrices
    double ax = (std::abs(T.at<double>(0,1)) + std::abs(T.at<double>(1,0)))/2;
    double ay = (std::abs(T.at<double>(0,2)) + std::abs(T.at<double>(2,0)))/2;
    double az = (std::abs(T.at<double>(2,1)) + std::abs(T.at<double>(1,2)))/2;

    std::cout << "ax: " << ax << std::endl;
    std::cout << "ay: " << ay << std::endl;
    std::cout << "az: " << az << std::endl;

    //building camera matrix
    double data[3][4] = {{R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2), -1 * (ax * R.at<double>(0,0) + ay * R.at<double>(0,1) + az * R.at<double>(0,2))},
                         {R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2), -1 * (ax * R.at<double>(1,0) + ay * R.at<double>(1,1) + az * R.at<double>(1,2))},
                         {R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2), -1 * (ax * R.at<double>(2,0) + ay * R.at<double>(2,1) + az * R.at<double>(2,2))}};
    cv::Mat camera = cv::Mat(3,4,CV_64F,data);
    std::cout << "M': " << std::endl << camera << std::endl;
    return camera;
}

int get_rank(const cv::Mat& mat) {
    int rank = 0;
    cv::Mat U, S, Vt;
    cv::SVDecomp(mat, S, U, Vt);
    rank = cv::countNonZero(S > 0.0001);
    std::cout << "RANK: " << rank << std::endl;
    return rank;
}
