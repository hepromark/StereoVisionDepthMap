//
// Created by markd on 2024-03-16.
//
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "PointSelection.h"
#include "IntrinsicSolver.h"

// Constructor
IntrinsicSolver::IntrinsicSolver(cv::Size pattern, double chess_square,
                                 cv::Size image_dim) {
    PATTERN = pattern;
    CHESS_SQUARE_DIM = chess_square;
    IMAGE_DIM = image_dim;
}

void IntrinsicSolver::find_pattern_size(const cv::Mat& img, int row_max, int col_max) {
    std::cout << "finding pattern size" << std::endl;
    cv::Mat temp;
    for (int i = 3; i < row_max; i++) {
        for (int j = 3; j < col_max; j++) {
            bool board_found = cv::findChessboardCorners(img,
                                                         cv::Size(i, j),
                                                         temp);
            if (board_found) {
                std::cout << "Size: " << i << " " << j << std::endl;
            }
        }
    }
}

void IntrinsicSolver::find_pattern_sizes_in_directory(std::string filepath) {
    std::filesystem::path directory = filepath;
    for (const auto& image : std::filesystem::directory_iterator(directory)) {
        std::string filename = image.path().filename().string();
        if (!std::filesystem::is_regular_file(image.path())) {
            std::cout << "Couldnt open file?" << std::endl;
            return;
        }
        std::cout << "file: " << filename << std::endl;
        std::string image_path = filepath + "\\" + filename;
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        find_pattern_size(img, 6, 8);
        std::cout << "===============" << std::endl;
        std::cout << std::endl;
    }
}

void IntrinsicSolver::check_image(std::string filepath, int row_count, int col_count) {
    // Read the image
    std::string image_path = filepath;
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cout << "Can't read image" << std::endl;
    }
    cv::imshow("Displaying image", img);
    int k = cv::waitKey(0); // Wait for a keystroke in the window

    // Check corners
    find_pattern_size(img, row_count, col_count);
}

void IntrinsicSolver::IntrinsicSolver::manual_annotate(std::string image_dir_path, std::string txt_path) {
    std::filesystem::path directory = image_dir_path;
    for (const auto& image : std::filesystem::directory_iterator(directory)) {
        std::string raw_filename = image.path().string();
        std::string filename = raw_filename.substr(image_dir_path.size() + 1, raw_filename.size() - 4);
        std::cout << "file: " << filename << std::endl;

        // Check file ok
        if (!std::filesystem::is_regular_file(image.path())) {
            std::cout << "Couldnt open file?" << std::endl;
            continue;
        }

        std::ofstream fout(txt_path + "\\" + filename + ".txt");

        // Load an image
        cv::Mat img = cv::imread(
                raw_filename,
                cv::IMREAD_GRAYSCALE);

        const int num_points = (PATTERN.width + 1) * (PATTERN.height + 1);
        std::vector<cv::Point> user_points = PointSelection::getPoints(img, num_points);

        for (auto point : user_points) {
            fout << point.x << " " << point.y << std::endl;
            std::cout << point << std::endl;
        }
        std::cout << txt_path + "\\" + filename + ".txt" << std::endl;
        fout.close();
    }
}

std::vector<cv::Point2f> read_corners_from_txt(std::string filepath) {
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

std::vector<cv::Point2f> IntrinsicSolver::read_corners_from_txt(std::string filepath) {
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

void IntrinsicSolver::get_2d_3d_coords(std::string txtdir,
                      std::vector<std::vector<cv::Point2f>>& img_coords,
                      std::vector<std::vector<cv::Point3f>>& world_coords) {

    std::filesystem::path directory = txtdir;
    int calibration_count = 0;  // tracks how many images have been calibrated so far
    for (const auto& image : std::filesystem::directory_iterator(directory)) {
        std::string filename = image.path().filename().string();
        std::cout << "Reading " << filename << std::endl;

        // Read corners array into input array
        std::vector<cv::Point2f> img_corners = read_corners_from_txt(image.path().string());
        img_coords.push_back(img_corners);

        // Real world coords into input array
        // Corners go from top left of image to bottom right
        world_coords.push_back({});
        for (int i = 0; i <= PATTERN.height; i++) {
            for (int j = 0; j <= PATTERN.width; j++) {
                world_coords[calibration_count].push_back(cv::Point3f(j * CHESS_SQUARE_DIM, i * CHESS_SQUARE_DIM, 0));
            }
        }
        calibration_count++;
    }
}

int IntrinsicSolver::calibrate(std::string calibration_txt_dir) {
    // Get input matrices
    std::vector<std::vector<cv::Point3f>> world_coords;
    std::vector<std::vector<cv::Point2f>> corner_pixel_coords;
    get_2d_3d_coords(calibration_txt_dir, corner_pixel_coords, world_coords);

    // Declare pass by ref output matrices
    cv::Mat cam_matrix;
    cv::Mat dist_coeff;
    std::vector<cv::Mat> rotation;
    std::vector<cv::Mat> translation;
    cv::Mat std_dev_intrinsics;
    cv::Mat std_dev_extrinsics;
    cv::Mat errors;

    // Print Parameters
    std::cout << "=============" << std::endl;
    std::cout << "Starting calibration with:" << std::endl;
    std::cout << "Calibration coordinate source file: " << calibration_txt_dir << std::endl;
    std::cout << "Image count: " << corner_pixel_coords.size() << std::endl;
    std::cout << "Image Dimension: " << IMAGE_DIM << std::endl;
    std::cout << "Chess Square Dimension: " << CHESS_SQUARE_DIM << std::endl;
    std::cout << "Pattern Size: " << PATTERN << std::endl;
    std::cout << "=============" << std::endl;

    cv::calibrateCamera(world_coords, corner_pixel_coords, IMAGE_DIM,
                        cam_matrix, dist_coeff,
                        rotation, translation,
                        std_dev_intrinsics, std_dev_extrinsics,
                        errors, cv::CALIB_ZERO_TANGENT_DIST);

    // Print results
    std::cout << "Camera Matrix" << std::endl;
    std::cout << cam_matrix << std::endl;
    std::cout << "=============" << std::endl;
    std::cout << "Distortion Coefficient" << std::endl;
    std::cout << dist_coeff << std::endl;
    std::cout << "=============" << std::endl;
    std::cout << "Intrinsic Standard Deviation" << std::endl;
    std::cout << std_dev_intrinsics << std::endl;
    std::cout << "=============" << std::endl;
    std::cout << "Errors" << std::endl;
    std::cout << errors << std::endl;

    // Write matrices to file
    std::ofstream fout("C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\cam_matrix");
    fout << cam_matrix << std::endl;
    fout.close();

    return 0;
}