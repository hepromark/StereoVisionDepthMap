//
// Created by aleon on 2024-03-19.
//
#include <opencv2/core.hpp>
#ifndef STEREOVISIONDEPTHMAP_MONOMEASUREMENT_H
#define STEREOVISIONDEPTHMAP_MONOMEASUREMENT_H

class MonoMeasurement {

public:
    MonoMeasurement();
    void calibrate(std::string output_dir, std::string image_path);

private:
    const int NUM_POINTS = 30;
    static double get_distance(cv::Point datum, cv::Point selected_point);



};

#endif //STEREOVISIONDEPTHMAP_MONOMEASUREMENT_H
