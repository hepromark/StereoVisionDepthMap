//
// Created by markd on 2024-03-16.
//

#include "FundamentalSolver.h"

#include <iostream>

FundamentalSolver::FundamentalSolver() {}

cv::Mat FundamentalSolver::calc_Fundamental(const std::vector<cv::Point2i> &points1, const std::vector<cv::Point2i> &points2) {
    // Algorithm requires at least 8 points
    const int num_points = points1.size();
    if (points1.size() != points2.size() || num_points < 8) {
        throw std::invalid_argument("Error: At least 8 points per each input vector, "
                                    "both vectors should have same length.");
    }

    // points1 and points2 have shape m x 2
    // Resulting W matrix has m columns (m >= 8)
    std::vector<std::vector<int>> W;

    for (int row = 0; row < num_points; row++) {
        // Every row is has:
        // p1x * p2x, p2y * p1x, p1x, p2x * p1y, p1y * p2y, p1y, p2x, p2y, 1
        std::vector<int> W_i = {points1[row].x * points2[row].x,
                                points2[row].y * points1[row].x,
                                points1[row].x,
                                points2[row].x * points1[row].y,
                                points1[row].y * points2[row].y,
                                points1[row].y,
                                points2[row].y,
                                1};
        W.push_back(W_i);
    }
    // Convert the 2D vector to cv::Mat
    cv::Mat mat(W.size(), W[0].size(), CV_32F);
    for (size_t i = 0; i < W.size(); ++i) {
        for (size_t j = 0; j < W[i].size(); ++j) {
            mat.at<float>(i, j) = W[i][j];
        }
    }

    std::cout << mat.size() << std::endl;
    std::cout << mat << std::endl;

    cv::Mat U, diag_S, Vt;
    cv::SVDecomp(mat, diag_S, U, Vt, cv::SVD::FULL_UV);  // makes the u and Vt square matrices
    cv::Mat S = cv::Mat::diag(diag_S);

    std::cout << "============" << std::endl;
    std::cout << U << std::endl;
    std::cout << U.size() << std::endl;
    std::cout << "============" << std::endl;
    std::cout << S << std::endl;
    std::cout << S.size() << std::endl;
    std::cout << "============" << std::endl;
    std::cout << Vt << std::endl;
    std::cout << Vt.size() << std::endl;
    std::cout << "============" << std::endl;

    // F_hat is the smallest singular value vector, last column in V
    cv::Mat F_hat = Vt.row(Vt.rows - 1);

    std::cout << F_hat << std::endl;
    std::cout << F_hat.size() << std::endl;
    std::cout << "============" << std::endl;

    return F_hat;
}
