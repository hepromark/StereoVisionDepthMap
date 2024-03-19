//
// Created by aleon on 2024-03-17.
//
#include <iostream>
#include "StereoMeasurement.h"
#include "FundamentalSolver.h"
#include "Distortion.h"
#include "MonoMeasurement.h"
#include <opencv2/opencv.hpp>

int main() {

    MonoMeasurement first_mono_measure;

    //uncomment when calibrating
    //    std::string txt_file_dir = "C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Mono_Calibration",
    //        image_path = "C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\Mono_Calibration\\calibration_photo_2.jpg";
    //    first_mono_measure.calibrate(txt_file_dir, image_path);

    double pixel_dist = first_mono_measure.measure();
    double mm_dist = first_mono_measure.convert_pix_to_mm(pixel_dist);


    // Setup backround and text for displaying measurement
    cv::Mat display_window = cv::Mat::zeros(800, 600, CV_8UC3);
    display_window.setTo(cv::Scalar(255, 192, 203));

    // Define the text to be displayed
    std::string pixel_text = "Distance in pixels: " + std::to_string(pixel_dist);
    std::string mm_text = "Distance in mm: " + std::to_string(mm_dist);

    // Calculate the position to center the text
    int font_style = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int thickness = 1;
    int baseline = 0;
    int x1, x2, y1, y2;

    // Calculate the size of the text
    cv::Size textSize1 = cv::getTextSize(pixel_text, font_style, font_scale, thickness, &baseline);
    cv::Size textSize2 = cv::getTextSize(mm_text, font_style, font_scale, thickness, &baseline);

    x1 = (display_window.cols - textSize1.width) / 2;
    y1 = (display_window.rows + textSize1.height) / 2;
    x2 = (display_window.cols - textSize2.width) / 2;
    y2 = y1 + textSize1.height + 10;


// Put the text on the window
    cv::putText(display_window, pixel_text, cv::Point(x1, y1), font_style,
                font_scale, cv::Scalar(0, 0, 0), thickness);
    cv::putText(display_window, mm_text, cv::Point(x2, y2), font_style,
                font_scale, cv::Scalar(0, 0, 0), thickness);

    // Display the window
    cv::imshow("Pink Window", display_window);

    // Wait for a key press to close the window
    cv::waitKey(0);

    cv::destroyAllWindows();

    return 0;
}