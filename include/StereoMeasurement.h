//
// Created by aleon on 2024-03-17.
//

//#include <opencv2/core.hpp>

#include <opencv2/opencv.hpp>

#ifndef STEREO_VISION_DEPTH_MAP_STEREOMEASUREMENT_H
#define STEREO_VISION_DEPTH_MAP_STEREOMEASUREMENT_H

class StereoMeasurement {

public:
    StereoMeasurement();
    void start();

    static void read_intrinsics(cv::Mat& camera_matrix, cv::Mat& distortion_coefficients, std::string path);
    static void StereoMeasurement::read_m_by_n(cv::Mat& output_matrix, int m, int n, std::string path);
    void k_then_opencv();

private:
    const int LEFT_CAM_INDEX = 0;
    const int RIGHT_CAM_INDEX = 2;
    const int POINTS_PER_PHOTO = 2;
    const int DESIRED_WIDTH = 1920, DESIRED_HEIGHT = 1080;

    //Matricies to store images
    cv::Mat left_image_distorted, right_image_distorted, left_image_undistorted, right_image_undistorted;

    //Matricies to store camera parameters
    cv::Mat left_cam_intrinsics, left_cam_distortion, right_cam_intrinsics, right_cam_distortion;

    // Final matrices for triangulation
    cv::Mat left_cam_M, right_cam_M;

    // Fundamental Matrix
    cv::Mat fundamental;

    //User selected points
    std::vector<cv::Point> left_points, right_points;

    void take_photos();
    void calculate_M();

};


#endif //STEREO_VISION_DEPTH_MAP_STEREOMEASUREMENT_H
