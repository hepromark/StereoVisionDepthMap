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
    double measure();
    double convert_pix_to_mm(double pixel_dist);

private:
    const int NUM_POINTS = 29;
    const int CAM_INDEX = 0;
    const int POINTS_PER_PHOTO = 2;
    const int DESIRED_WIDTH = 1920, DESIRED_HEIGHT = 1080;
    const double CUBE_CONST = 3.5174 * pow(10,-8),
                    SQUARE_CONST = -7.4493 * pow(10,-5), LINEAR_CONST = 2.2467 * pow(10,-1);

    cv::Mat image;
    std::vector<cv::Point> selected_points;

    static double get_distance(cv::Point datum, cv::Point selected_point);
    void take_photo();

};

#endif //STEREOVISIONDEPTHMAP_MONOMEASUREMENT_H
