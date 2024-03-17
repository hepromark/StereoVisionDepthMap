
#include "FundamentalSolver.h"

#include <iostream>
#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>

#include "PointSelection.h"

FundamentalSolver::FundamentalSolver() {}

void FundamentalSolver::manual_match_points(std::string input_image_dir, std::string output_txt_dir) {
    std::filesystem::path directory = input_image_dir;
    for (const auto& image : std::filesystem::directory_iterator(directory)) {
        std::string raw_filename = image.path().string();
        std::string filename = raw_filename.substr(input_image_dir.size() + 1, raw_filename.size() - 4);
        std::cout << "file: " << filename << std::endl;

        // Check file ok
        if (!std::filesystem::is_regular_file(image.path())) {
            std::cout << "Couldnt open file?" << std::endl;
            return;
        }

        std::ofstream fout(output_txt_dir + "\\" + filename + ".txt");

        // Load an image
        cv::Mat img = cv::imread(
                raw_filename,
                cv::IMREAD_GRAYSCALE);

        const int num_points = 8;
        std::vector<cv::Point> user_points = PointSelection::getPoints(img, num_points);

        for (auto point : user_points) {
            fout << point.x << " " << point.y << std::endl;
            std::cout << point << std::endl;
        }
        std::cout << output_txt_dir + "\\" + filename + ".txt" << std::endl;
        fout.close();
    }
}

std::vector<cv::Point2i> FundamentalSolver::read_corners_from_txt(std::string filepath) {
    std::ifstream fin(filepath);
    int x = 0;
    int y = 0;
    std::vector<cv::Point2i> output;
    while (fin >> x) {
        fin >> y;
        output.push_back(cv::Point2i(x, y));
    }

    return output;
}

cv::Mat FundamentalSolver::calc_fundamental(std::string cam1_pts, std::string cam2_pts) {
    std::vector<cv::Point2i> points1 = read_corners_from_txt(cam1_pts);
    std::vector<cv::Point2i> points2 = read_corners_from_txt(cam2_pts);

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
