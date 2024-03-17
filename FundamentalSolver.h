//
// Created by markd on 2024-03-16.
//

#ifndef STEREOVISIONDEPTHMAP_FUNDAMENTALSOLVER_H
#define STEREOVISIONDEPTHMAP_FUNDAMENTALSOLVER_H

#include <opencv2/core.hpp>

class FundamentalSolver {
public:
    FundamentalSolver();

public:
    cv::Mat manual_match_points(std::string input_imgage_dir, std::string output_txt_dir);
    cv::Mat FundamentalSolver::calc_Fundamental(const std::vector<cv::Point2i> &points1,
                                                const std::vector<cv::Point2i> &points2);
};


#endif //STEREOVISIONDEPTHMAP_FUNDAMENTALSOLVER_H
