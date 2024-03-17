//
// Created by markd on 2024-03-16.
//

#ifndef STEREOVISIONDEPTHMAP_INTRINSICSOLVER_H
#define STEREOVISIONDEPTHMAP_INTRINSICSOLVER_H

#include <opencv2/core.hpp>

class IntrinsicSolver {
public:
    // Constructor
    IntrinsicSolver(cv::Size pattern, double chess_square, cv::Size image_size);

    // Public Members
    cv::Size PATTERN;
    double CHESS_SQUARE_DIM;
    cv::Size IMAGE_DIM;

public:
    // Public methods
    static void find_pattern_size(const cv::Mat& img, int row_max, int col_max);
    static void find_pattern_sizes_in_directory(std::string filepath);
    static void check_image(std::string filepath, int row_count, int col_count);
    void manual_annotate(std::string image_dir_path, std::string output_txt_path);
    int calibrate(std::string pixel_coords_output_txt, std::string output_matrix_path);

private:
    // Private Methods
    std::vector<cv::Point2f> read_corners_from_txt(std::string filepath);
    void get_2d_3d_coords(std::string pixel_coords_output_txt,
                          std::vector<std::vector<cv::Point2f>>& img_coords,
                          std::vector<std::vector<cv::Point3f>>& world_coords);
};


#endif //STEREOVISIONDEPTHMAP_INTRINSICSOLVER_H
