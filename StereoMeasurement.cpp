//
// Created by aleon on 2024-03-17.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Distortion.h"
#include "PointSelection.h"
#include "Triangulate.h"
#include <fstream>

#include "StereoMeasurement.h"


//std::vector<cv::Mat>

/**
 * Constructor - Creates an object with required members to measure distance with stereo cameras.
 *
 * Opens txt files and reads stored camera parameters and distortion coefficients.
 */
StereoMeasurement::StereoMeasurement() {
    //Get intrinsics for each camera
    // K1
    read_intrinsics(left_cam_intrinsics,left_cam_distortion,
                    "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\K1.txt");

    //K2
    read_intrinsics(right_cam_intrinsics,right_cam_distortion,
                    "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\K2.txt");

    // Read fundamental matrix
    read_m_by_n(fundamental, 3, 3, "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\F.txt");

    calculate_M();
}

/**
 * Function containing the procedure to take a distance measurement.
 */
void StereoMeasurement::start() {

    //Take photos and undistort them
//    take_photos();

    // Temporary no camera testing
    left_image_distorted = cv::imread("C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\Fundamental\\imgs\\cam1.jpg");

    right_image_distorted = cv::imread("C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\Fundamental\\imgs\\cam2.jpg");

    left_image_undistorted = Distortion::correct_distortion(
            left_image_distorted,left_cam_intrinsics, left_cam_distortion);
    right_image_undistorted = Distortion::correct_distortion(
            right_image_distorted, right_cam_intrinsics, right_cam_distortion);

    //Get user to select points in undistorted photos
    left_points = PointSelection::getPoints(left_image_undistorted, POINTS_PER_PHOTO);
    right_points = PointSelection::getPoints(right_image_undistorted, POINTS_PER_PHOTO);

    //OPENCV SHIT
    cv::Mat cam_1_proj_point_1 = cv::Mat(2, 1, CV_64F);
    cam_1_proj_point_1.at<double>(0, 0) = left_points[0].x;
    cam_1_proj_point_1.at<double>(1, 0) = left_points[0].y;

    cv::Mat cam_1_proj_point_2 = cv::Mat(2, 1, CV_64F);
    cam_1_proj_point_2.at<double>(0, 0) = right_points[0].x;
    cam_1_proj_point_2.at<double>(1, 0) = right_points[0].y;

    cv::Mat cam_2_proj_point_1 = cv::Mat(2, 1, CV_64F);
    cam_2_proj_point_1.at<double>(0, 0) = left_points[1].x;
    cam_2_proj_point_1.at<double>(1, 0) = left_points[1].y;

    cv::Mat cam_2_proj_point_2 = cv::Mat(2, 1, CV_64F);
    cam_2_proj_point_2.at<double>(0, 0) = right_points[1].x;
    cam_2_proj_point_2.at<double>(1, 0) = right_points[1].y;


    std::cout << cam_1_proj_point_2 << std::endl;
    std::cout << cam_1_proj_point_2 << std::endl;

//    std::vector<cv::Point2i> pixel_pair_1 = {cv::Point(left_points[0]), cv::Point(right_points[0])};
//    std::vector<cv::Point2i> pixel_pair_2 = {cv::Point(left_points[1]), cv::Point(right_points[1])};

    // Do Triangulation to get output points
    cv::Mat P1;
    cv::Mat P2;
    cv::triangulatePoints(left_cam_M, right_cam_M, cam_1_proj_point_1, cam_1_proj_point_2, P1);
    cv::triangulatePoints(left_cam_M, right_cam_M, cam_2_proj_point_1, cam_2_proj_point_2, P2);

    P1 = P1 / P1.at<double>(3, 0);
    P2 = P2 / P2.at<double>(3, 0);

    std::cout << P1 << std::endl;
    std::cout << P2 << std::endl;

    std::cout << "DISTANCE: " << std::endl;
    std::cout << triangulate(P1, P2);


//    cv::Mat P1 = calc_P(left_cam_M, right_cam_M, pixel_pair_1);
//    std::cout << "POINT1: " << std::endl;
//    std::cout << P1 << std::endl;
//
//    cv::Mat P2 = calc_P(left_cam_M, right_cam_M, pixel_pair_2);
//    std::cout << "POINT2: " << std::endl;
//    std::cout << P2 << std::endl;
}

void StereoMeasurement::calculate_M() {
    // M1 = K[I 0]
    left_cam_M = cv::Mat::zeros(3, 4, CV_64F);
    left_cam_intrinsics.copyTo(
            left_cam_M(cv::Rect(0, 0, left_cam_intrinsics.cols, left_cam_intrinsics.rows))
    );

    std::cout << left_cam_M << std::endl;

    // M2 = K[Rt -Rt * T] which is pre-calculated in a .txt
    read_m_by_n(right_cam_M, 3, 4, "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\M'.txt");
}

/**
 * Read an m x n stored in a text file and save into a cv::Mat object.
 *
 * @param output_matrix Mat object passed as ref used to return the matrix stored in txt file.
 * @param m Number of rows in matrix.
 * @param n Number of columns in matrix.
 * @param path Path to txt file
 */
void StereoMeasurement::read_m_by_n(cv::Mat& output_matrix, int m, int n, std::string path) {

    //Initialize matrix
    output_matrix = cv::Mat(m,n,CV_64FC1);

    //Open file, then check if opened properly
    std::ifstream fin(path);
    if(!fin){
        std::cout << "Unable to open file";
        return;
    }

    //Reading from file
    for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++) {
            double in;
            fin >> in;
            output_matrix.at<double>(i, j) = in;
        }
    }

    fin.close();
}

/**
 * Reads intrinsic parameter and distortion coefficients stored in txt file.
 *      txt file must be formatted with first 9 values being the camera matrix, then
 *      the next 5 as distortion coefficients.
 *
 * @param camera_matrix Mat object passed as ref used to return the matrix stored in txt file.
 * @param distortion_coefficients Mat object passed as ref used to return distortion coefficients.
 * @param path Path to txt file
 */
void StereoMeasurement::read_intrinsics(cv::Mat& camera_matrix, cv::Mat& distortion_coefficients, std::string path){
    std::ifstream fin(path);

    camera_matrix = cv::Mat(3,3,CV_64FC1);
    distortion_coefficients = cv::Mat(5,1,CV_64FC1);

    if(!fin){
        std::cout << "Unable to open file";
        return;
    }

    //Reading intrinsic camera parameters
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++) {
            double in;
            fin >> in;
            camera_matrix.at<double>(i, j) = in;
        }
    }

    std::cout << "Camera parameters: " << camera_matrix << std::endl;

    //Reading camera distortion coefficients
    for(int i = 0; i < 5; i++) {
        fin >> distortion_coefficients.at<double>(i,0);
    }
    std::cout << "Disortion coefficients: " << distortion_coefficients << std::endl;

    fin.close();
}

/**
 * Takes photos from stereo camera setup
 */
void StereoMeasurement::take_photos() {

    //Assign and open cameras
    cv::VideoCapture left_cam(LEFT_CAM_INDEX);
    cv::VideoCapture right_cam(RIGHT_CAM_INDEX);

    if(!left_cam.isOpened() || !right_cam.isOpened()) {
        std::cout << "Unable to open cameras";
    }

    //Setup proper aspect ratio for each camera
    left_cam.set(cv::CAP_PROP_FRAME_WIDTH, DESIRED_WIDTH);
    left_cam.set(cv::CAP_PROP_FRAME_HEIGHT, DESIRED_HEIGHT);
    right_cam.set(cv::CAP_PROP_FRAME_WIDTH, DESIRED_WIDTH);
    right_cam.set(cv::CAP_PROP_FRAME_HEIGHT, DESIRED_HEIGHT);

    //Take photos then release the cameras
    left_cam >> left_image_distorted;
    right_cam >> right_image_distorted;
    left_cam.release();
    right_cam.release();
}