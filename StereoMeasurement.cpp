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
    //Get intrinsics for each camera
    read_intrinsics(left_cam_intrinsics,left_cam_distortion,
                    "C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Calibration\\K1.txt");
    read_intrinsics(right_cam_intrinsics,right_cam_distortion,
                    "C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Calibration\\K2.txt");

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

void StereoMeasurement::read_intrinsics(cv::Mat camera_matrix, cv::Mat distortion_coefficients, std::string path){
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

    //Reading camera distortion coefficients
    for(int i = 0; i < 5; i++) {
        fin >> distortion_coefficients.at<double>(i,0);
    }

    std::cout << "Left intrinsics: " << camera_matrix << std::endl <<
              "Left distortion" << distortion_coefficients << std::endl;

    fin.close();
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