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
    void FundamentalSolver::manual_match_points(std::string input_image_dir,
                                                std::string input_K_dir,
                                                std::string output_txt_file);
    void FundamentalSolver::calc_fundamental(std::string cam1_pts,
                                                std::string cam2_pts,
                                                std::string output_txt_dir);
    std::vector<cv::Point2f> FundamentalSolver::read_corners_from_txt(std::string filepath);
    cv::Mat FundamentalSolver::normalize_points(std::vector<cv::Point2f>& points1);
};


#endif //STEREOVISIONDEPTHMAP_FUNDAMENTALSOLVER_H
