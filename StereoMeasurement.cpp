//
// Created by aleon on 2024-03-17.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Distortion.h"
#include "PointSelection.h"
#include <fstream>

#include "StereoMeasurement.h"


//std::vector<cv::Mat>

StereoMeasurement::StereoMeasurement() {


    //Read in camera intrinsic parameters (including distortion)
    std::ifstream fin("C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Calibration\\K1.txt");

    if(!fin) {
        std::cout << "Could not open file to read intrinsics";
    }


    //Reading intrinsic parameters
    std::vector<std::vector<double>> data;
    std::string line;

    //Get 1 line at a time
    while (std::getline(fin, line)) {
        std::vector<double> row;
        std::string value;
        for (char c : line) {

            //Skip opening brackets
            if (c == '(' || c == '[') {
                continue;
            } else if (c == ',' || c == ';' || c == ')' || c == ']') {
                if (!value.empty()) {
                    row.push_back(std::stod(value));
                    value.clear();
                }
            } else {
                value.push_back(c);
            }
        }
        if (!row.empty()) {
            data.push_back(row);
        }
    }

    // Convert parsed values into matrix
    left_cam_intrinsics = cv::Mat(data.size(), data[0].size(), CV_64F);
    for (int i = 0; i < left_cam_intrinsics.rows; ++i) {
        for (int j = 0; j < left_cam_intrinsics.cols; ++j) {
            left_cam_intrinsics.at<double>(i, j) = data[i][j];
        }
    }

    std::cout<<left_cam_intrinsics;

}


void StereoMeasurement::start() {

    //Take photos and undistort
    take_photos();
    left_image_undistorted = Distortion::correct_distortion
            (left_image_distorted,left_cam_intrinsics, left_cam_distortion);
    right_image_undistorted = Distortion::correct_distortion(
            right_image_distorted, right_cam_intrinsics, right_cam_distortion);

    //Get user to select points
    left_points = PointSelection::getPoints(left_image_undistorted, POINTS_PER_PHOTO);
    right_points = PointSelection::getPoints(right_image_undistorted, POINTS_PER_PHOTO);

    cv::imshow("Left image", left_image_distorted);
    cv::imshow("Right image", right_image_distorted);
    while(cv::waitKey(1) != 'q'){}


    //Take photos
    //Undistort photos
    //Show left and right photos to user and get matching points
    //Pass matching points to 3D coordinate finder x2
    //Pass real world points to 3D distance solver




}


void StereoMeasurement::take_photos() {
    cv::VideoCapture left_cam(LEFT_CAM_INDEX);
    cv::VideoCapture right_cam(RIGHT_CAM_INDEX);

    if(!left_cam.isOpened() || !right_cam.isOpened()) {
        std::cout << "Unable to open cameras";
    }

    left_cam >> left_image_distorted;
    right_cam >> right_image_distorted;

    left_cam.release();
    right_cam.release();
}