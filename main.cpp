//
// Created by aleon on 2024-03-17.
//
#include <iostream>
#include "StereoMeasurement.h"
#include "FundamentalSolver.h"
#include "Distortion.h"
#include "MonoMeasurement.h"

int main() {
//    FundamentalSolver fs;
//    fs.manual_match_points(
//        24,
//        "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\Fundamental\\tmp_imgs",
//        "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration",
//        "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\Fundamental\\tmp_txt");
//
//    fs.calc_fundamental_2(
//            "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\Fundamental\\tmp_txt\\hallway1.jpg.txt",
//            "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\Fundamental\\tmp_txt\\hallway2.jpg.txt",
//            "C:\\Users\\markd\\Documents\\GitHub\\StereoVisionDepthMap\\Calibration\\F.txt");

//    StereoMeasurement firstMeasurement;
//    firstMeasurement.start();
//    firstMeasurement.k_then_opencv();


    std::string txt_file_dir = "C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Mono_Calibration",
        image_path = "C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Mono_Calibration\\calibration_photo.jpg";

    MonoMeasurement first_mono_measure;
    first_mono_measure.calibrate(txt_file_dir, image_path);

    std::cout << "Calibration done";



    return 0;
}