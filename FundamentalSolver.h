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
    void manual_match_points(std::string input_imgage_dir, std::string output_txt_dir);
    cv::Mat FundamentalSolver::calc_fundamental(std::string cam1_pts, std::string cam2_pts);
    std::vector<cv::Point2i> FundamentalSolver::read_corners_from_txt(std::string filepath);
};


#endif //STEREOVISIONDEPTHMAP_FUNDAMENTALSOLVER_H
