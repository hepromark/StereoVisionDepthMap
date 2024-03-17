//
// Created by aleon on 2024-03-17.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Distortion.h"

#include "StereoMeasurement.h"


//std::vector<cv::Mat>

StereoMeasurement::StereoMeasurement() {

}


void StereoMeasurement::start() {

    //Take photos and undistort
    take_photos();


    cv::imshow("Left image", left_image);
    cv::imshow("Right image", right_image);
    while(cv::waitKey(1) != 'q'){}


    //Take photos
    //Undistort photos
    //Show left and right photos to user and get matching points
    //Pass matching points to 3D coordinate finder x2
    //Pass real world points to 3D distance solver

    //Connect to cameras
    // StereoMeasurement distance_test_1;





}


void StereoMeasurement::take_photos() {
    cv::VideoCapture left_cam(LEFT_CAM_INDEX);
    cv::VideoCapture right_cam(RIGHT_CAM_INDEX);

    if(!left_cam.isOpened() || !right_cam.isOpened()) {
        std::cout << "Unable to open cameras";
    }

    left_cam >> left_image;
    right_cam >> right_image;

    left_cam.release();
    right_cam.release();
}