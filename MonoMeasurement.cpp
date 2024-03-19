//
// Created by aleon on 2024-03-19.
//
//Need to register first point click at 0mm and save this point coordinate
//  Then, do 30 clicks, each should be 10mm further than the previous
//Save each distance to a text file
//Repeat process, but start at 5mm and save to separate txt file

//Will need a function to find distance
//Will need a function that handles point taking
//  This function will need to call the PointSelection::getPoints() function

#include "MonoMeasurement.h"
#include "PointSelection.h"
#include "cmath"
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

MonoMeasurement::MonoMeasurement() {}

double MonoMeasurement::get_distance(cv::Point datum, cv::Point selected_point) {
    return sqrt(pow(selected_point.x-datum.x,2) + pow(selected_point.y-datum.y,2));
}

void MonoMeasurement::calibrate(std::string output_dir, std::string image_path) {
    //Create files and open w/ file streams

    std::ofstream datum_zero_fout(output_dir + "zero_datum_measurements");
    std::ofstream datum_five_fout(output_dir  + "five_datum_measurements");
    cv::Mat calibration_image = cv::imread(image_path);

    if(!datum_five_fout || !datum_zero_fout || calibration_image.empty()) {
        std::cout << "Unable to open files";
        return;
    }

    datum_zero_fout << "Distance pixels distances for points with datum at 0mm."
                       " Ground truth increments of 10mm between points."<< std::endl;
    datum_five_fout << "Distance pixels distances for points with datum at 5mm."
                       " Ground truth increments of 10mm between points."<< std::endl;

    //Get all Points
    std::vector<cv::Point> zero_datum_points = PointSelection::getPoints(calibration_image, NUM_POINTS);
    std::vector<cv::Point> five_datum_points = PointSelection::getPoints(calibration_image, NUM_POINTS);

    std::cout<<"Zero datum points are: " << zero_datum_points << "\nAnd five datum points are: " << five_datum_points
        << std::endl;


    //Find distances and write to file


    datum_zero_fout.close();
    datum_five_fout.close();

}