#include "FundamentalSolver.h"

#include <iostream>
#include <filesystem>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <cmath>

#include "PointSelection.h"
#include "Distortion.h"

FundamentalSolver::FundamentalSolver() {}

void FundamentalSolver::manual_match_points(int num_points,
                                            std::string input_image_dir,
                                            std::string input_K_dir,
                                            std::string output_txt_file) {
    std::filesystem::path directory = input_image_dir;
    int cam_idx = 1;
    for (const auto& image : std::filesystem::directory_iterator(directory)) {
        std::string raw_filename = image.path().string();
        std::string filename = raw_filename.substr(input_image_dir.size() + 1, raw_filename.size() - 4);
        std::cout << "file: " << filename << std::endl;

        // Check file ok
        if (!std::filesystem::is_regular_file(image.path())) {
            std::cout << "Couldnt open file?" << std::endl;
            return;
        }

        std::ofstream fout(output_txt_file + "\\" + filename + ".txt");

        // Load an image
        cv::Mat distorted_img = cv::imread(
                raw_filename,
                cv::IMREAD_GRAYSCALE);

        // Load intrinsic parameters
        cv::Mat K = cv::Mat(3, 3, CV_64F);
        std::fstream fin(input_K_dir + "\\K" + std::to_string(cam_idx) + ".txt");
        std::cout << input_K_dir + "\\K" + std::to_string(cam_idx) + ".txt" << std::endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                fin >> K.at<double>(i, j);
            }
        }

        cv::Mat distortion_coeff;
        for (int d = 0; d < 5; d++) {
            double temp;
            fin >> temp;
            distortion_coeff.push_back(temp);
        }

        // Undistort image before calibrating
        std::cout << K << std::endl;
        std::cout << distortion_coeff << std::endl;
        cv::Mat img = Distortion::correct_distortion(distorted_img, K, distortion_coeff);

        std::vector<cv::Point> user_points = PointSelection::getPoints(img, num_points);

        for (auto point : user_points) {
            fout << point.x << " " << point.y << std::endl;
            std::cout << point << std::endl;
        }
        std::cout << output_txt_file + "\\" + filename + ".txt" << std::endl;
        fout.close();

        // Increment cam
        cam_idx++;
    }
}

std::vector<cv::Point2f> FundamentalSolver::read_corners_from_txt(std::string filepath) {
    std::ifstream fin(filepath);
    int x = 0;
    int y = 0;
    std::vector<cv::Point2f> output;
    while (fin >> x) {
        fin >> y;
        output.push_back(cv::Point2i(x, y));
    }

    return output;
}

cv::Mat FundamentalSolver::normalize_points(std::vector<cv::Point2f>& points) {
    // Math source:
    // https://www5.cs.fau.de/fileadmin/lectures/2014s/Lecture.2014s.IMIP/exercises/4/exercise4.pdf

    // Get x and y centroids for Point 1
    double x_centroid = 0;
    double y_centroid = 0;
    for (const cv::Point2f& point : points) {
        x_centroid += point.x;
        y_centroid += point.y;
    }
    x_centroid /= points.size();
    y_centroid /= points.size();
    std::cout << "Centroid: " << x_centroid << " " << y_centroid << std::endl;

    // Translate points
    for (cv::Point2f& point : points) {
        point.x -= x_centroid;
        point.y -= y_centroid;
    }
    std::cout << "Translated: " << points << std::endl;

    // Get scaling factor s
    double total_dist = 0;
    for (cv::Point2f& point : points) {
        total_dist += std::pow(point.x, 2) + std::pow(point.y, 2);
    }
    total_dist = std::pow(total_dist, 0.5);
    std::cout << "Total Distance: " << total_dist << std::endl;

    double scale_factor = 1 / (total_dist / points.size() * std::pow(2, 0.5));
    std::cout << "Scale factor: " << scale_factor << std::endl;

    // Scale points array
    for (cv::Point2f& point : points) {
        point *= scale_factor;
    }

    // Output transform matrix T
    cv::Mat T = cv::Mat::zeros(3, 3, CV_64F);
    T.at<double>(0, 0) = scale_factor;
    T.at<double>(1, 1) = scale_factor;
    T.at<double>(0, 2) = -1 * scale_factor * x_centroid;
    T.at<double>(1, 2) = -1 * scale_factor * y_centroid;
    T.at<double>(2, 2) = 1;

    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << T << std::endl;
    return T;
}

void FundamentalSolver::calc_fundamental_2(std::string cam1_pts, std::string cam2_pts, std::string output_txt_path) {
    std::vector<cv::Point2f> points1 = read_corners_from_txt(cam1_pts);
//    std::vector<std::vector<cv::Point2f>> p1;
//    p1.push_back(points1);
    std::vector<cv::Point2f> points2 = read_corners_from_txt(cam2_pts);
//    std::vector<std::vector<cv::Point2f>> p2;
//    p2.push_back(points2);

    std::cout << points1 << std::endl;
    std::cout << points2 << std::endl;

    cv::Mat F;
    F = cv::findFundamentalMat(points1, points2, F);

    // Output to file
    std::cout << "============" << std::endl;
    std::cout << "F" << std::endl;
    std::cout << F << std::endl;
    std::cout << F.size() << std::endl;
    std::cout << "============" << std::endl;

    std::ofstream fout(output_txt_path);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            fout << F.at<double>(i, j) << " ";
        }
        fout << std::endl;
    }
}

void FundamentalSolver::calc_fundamental(std::string cam1_pts, std::string cam2_pts, std::string output_txt_path) {
    std::vector<cv::Point2f> points1 = read_corners_from_txt(cam1_pts);
    std::vector<cv::Point2f> points2 = read_corners_from_txt(cam2_pts);

    // Normalize points and get T
    cv::Mat T1 = normalize_points(points1);
    cv::Mat T2 = normalize_points(points2);

    // Print normalized points
    std::cout << "=========" << std::endl;
    std::cout << points1 << std::endl;
    std::cout << "=========" << std::endl;
    std::cout << "=========" << std::endl;
    std::cout << points2 << std::endl;
    std::cout << "=========" << std::endl;

    // Algorithm requires at least 8 points
    const int num_points = points1.size();
    if (points1.size() != points2.size() || num_points < 8) {
        throw std::invalid_argument("Error: At least 8 points per each input vector, "
                                    "both vectors should have same length.");
    }

    // points1 and points2 have shape m x 2
    // Resulting W matrix has m columns (m >= 8)
    std::vector<std::vector<double>> W;

    for (int row = 0; row < num_points; row++) {
        // Every row is has:
        // p1x * p2x, p2y * p1x, p1x, p2x * p1y, p1y * p2y, p1y, p2x, p2y, 1
        std::vector<double> W_i = {points1[row].x * points2[row].x,
                                   points2[row].y * points1[row].x,
                                   points1[row].x,
                                   points2[row].x * points1[row].y,
                                   points1[row].y * points2[row].y,
                                   points1[row].y,
                                   points2[row].x,
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

    // First SVD Decomp
    cv::Mat U, diag_S, Vt;
    cv::SVDecomp(mat, diag_S, U, Vt, cv::SVD::FULL_UV);  // makes the u and Vt square matrices
    cv::Mat S = cv::Mat::diag(diag_S);

    // F_hat is the smallest singular value vector, last column in V
    cv::Mat F_hat = Vt.row(Vt.rows - 1);

    // Turn the vector F_hat into (3x3) matrix
    F_hat = F_hat.reshape(1, 3);
    F_hat.convertTo(F_hat, CV_64F);

    // Do SVD again to get a rank 2 matrix
    cv::Mat U2, diag_S2, Vt2;
    cv::SVDecomp(F_hat, diag_S2, U2, Vt2, cv::SVD::FULL_UV);  // makes the u and Vt square matrices

    // Construct rank 2 F
    cv::Mat S2 = cv::Mat::zeros(3, 3, CV_64F);
    S2.at<double>(0, 0) = diag_S2.at<double>(0);
    S2.at<double>(1, 1) = diag_S2.at<double>(1);

    cv::Mat F = U2 * S2 * Vt2;

    // Remove the previous transform
    // T_right_cam * F * T_left_cam
    F = T1.t() * F * T2;
    F = F / F.at<double>(2, 2);

    // Output to file
    std::cout << "============" << std::endl;
    std::cout << "F" << std::endl;
    std::cout << F << std::endl;
    std::cout << F.size() << std::endl;
    std::cout << "============" << std::endl;

    std::ofstream fout(output_txt_path);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            fout << F.at<double>(i, j) << " ";
        }
        fout << std::endl;
    }
}


cv::Mat FundamentalSolver::solve_camera2(cv::Mat & fund,  cv::Mat & K1,  cv::Mat & K2, std::string matrix_output_path,
                                         cv::Mat R, cv::Mat T) {
//    cv::Mat E = K2.t() * fund * K1;
//    std::cout <<"K1: " <<  K1 << std::endl << "K2: "<< K2 << std::endl;
//    std::cout << "E: " << E << std::endl;
//
//    //SVD decomposition
//    cv::SVD decomp = cv::SVD(E);
//    cv::Mat U = decomp.u;
//    cv::Mat V = decomp.vt.t();
//
//    cv::Mat EEt = E * E.t();
//    cv::Mat S(3, 3, CV_64F, cv::Scalar(0));
//    S.at<double>(0, 0) = std::pow(EEt.at<double>(0,0),0.5);
//    S.at<double>(1, 1) = std::pow(EEt.at<double>(1,1),0.5);
//    std::cout << "EEt: " << std::endl << EEt << std::endl;
//    std::cout << "S: " << std::endl << S << std::endl;
//
//    cv::Mat W(3, 3, CV_64F, cv::Scalar(0));
//    W.at<double>(0, 1) = -1;
//    W.at<double>(1, 0) = 1;
//    W.at<double>(2, 2) = 1;
//
//    cv::Mat R = U * W.t() * V.t();
//    cv::Mat T = U * W * S * U.t();
//
//    std::cout << "Computed rotation:"<< std::endl << R << std::endl;
//    std::cout << "Computed translation:" << std::endl << T << std::endl;
//
//    //digesting translation and rotation matrices
//    double ax = (std::abs(T.at<double>(0,1)) + std::abs(T.at<double>(1,0)))/2;
//    double ay = (std::abs(T.at<double>(0,2)) + std::abs(T.at<double>(2,0)))/2;
//    double az = (std::abs(T.at<double>(2,1)) + std::abs(T.at<double>(1,2)))/2;

    double ax = T.at<double>(0, 0);
    double ay = T.at<double>(1, 0);
    double az = T.at<double>(2, 0);

    std::cout << "ax: " << ax << std::endl;
    std::cout << "ay: " << ay << std::endl;
    std::cout << "az: " << az << std::endl;

    cv::Mat Rt = R.t();

    //building camera matrix
    double data[3][4] = {{Rt.at<double>(0,0),Rt.at<double>(0,1),Rt.at<double>(0,2), -1 * (ax * Rt.at<double>(0,0) + ay * Rt.at<double>(0,1) + az * Rt.at<double>(0,2))},
                         {Rt.at<double>(1,0),Rt.at<double>(1,1),R.at<double>(1,2), -1 * (ax * Rt.at<double>(1,0) + ay * Rt.at<double>(1,1) + az * Rt.at<double>(1,2))},
                         {Rt.at<double>(2,0),Rt.at<double>(2,1),Rt.at<double>(2,2), -1 * (ax * Rt.at<double>(2,0) + ay * Rt.at<double>(2,1) + az * Rt.at<double>(2,2))}};
    cv::Mat camera = K2 * cv::Mat(3,4,CV_64F,data);
    std::cout << "M': " << std::endl << camera << std::endl;

    std::ofstream fout(matrix_output_path);
    for (int i =0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j)
            fout << camera.at<double>(i,j) << " ";
        fout << std::endl;
    }

    return camera;
}

int FundamentalSolver::get_rank(const cv::Mat &mat) {
    int rank = 0;
    cv::Mat U, S, Vt;
    cv::SVDecomp(mat, S, U, Vt);
    rank = cv::countNonZero(S > 0.0001);
    std::cout << "RANK: " << rank << std::endl;
    return rank;
}