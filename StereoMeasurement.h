//
// Created by aleon on 2024-03-17.
//

//#include <opencv2/core.hpp>

#include <opencv2/opencv.hpp>

#ifndef STEREO_VISION_DEPTH_MAP_STEREOMEASUREMENT_H
#define STEREO_VISION_DEPTH_MAP_STEREOMEASUREMENT_H

class StereoMeasurement {

public:
    StereoMeasurement();
    void start();



private:
    const int LEFT_CAM_INDEX = 2;
    const int RIGHT_CAM_INDEX = 0;
    cv::Mat left_image, right_image;

    void take_photos();


};


#endif //STEREO_VISION_DEPTH_MAP_STEREOMEASUREMENT_H
