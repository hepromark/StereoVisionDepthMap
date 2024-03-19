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

    std::ofstream datum_zero_fout(output_dir + "\\zero_datum_measurements_trial2");
    std::ofstream datum_five_fout(output_dir  + "\\five_datum_measurements_trial2");
    cv::Mat calibration_image = cv::imread(image_path);

    if(!datum_five_fout || !datum_zero_fout || calibration_image.empty()) {
        std::cout << "Unable to open files";
        return;
    }

    datum_zero_fout << "Distance between points and datum in pixels."
                       " Ground truth increments of 10mm between points and datum at 0mm."<< std::endl;
    datum_five_fout << "Distance between points and datum in pixels."
                       " Ground truth increments of 10mm between points and datum at 5mm"<< std::endl;

    //Get all Points (+1 for datum)
    std::vector<cv::Point> zero_datum_points = PointSelection::getPoints(calibration_image, NUM_POINTS+1);
    std::vector<cv::Point> five_datum_points = PointSelection::getPoints(calibration_image, NUM_POINTS+1);

    std::cout<<"Zero datum points are: " << zero_datum_points << "\nAnd five datum points are: " << five_datum_points
        << std::endl;


    //Find distances and write to file
    double distance;

    for(int i = 1; i <= NUM_POINTS; i++) {
        distance = get_distance(zero_datum_points[0], zero_datum_points[i]);
        datum_zero_fout << distance << " ";

        distance = get_distance(five_datum_points[0], five_datum_points[i]);
        datum_five_fout << distance << " ";
    }


    datum_zero_fout.close();
    datum_five_fout.close();

}