//
// Created by aleon on 2024-03-17.
//
#include <iostream>
#include "StereoMeasurement.h"
#include "FundamentalSolver.h"
#include "Distortion.h"

int main() {
    FundamentalSolver fs;
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

    StereoMeasurement firstMeasurement;
    firstMeasurement.start();
//    firstMeasurement.k_then_opencv();

    return 0;
}