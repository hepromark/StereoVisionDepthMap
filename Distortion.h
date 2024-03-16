//
// Created by aleon on 2024-03-16.
//

#ifndef STEREO_VISION_DEPTH_MAP_DISTORTION_H
#define STEREO_VISION_DEPTH_MAP_DISTORTION_H

class Distortion {
public:
    static cv::Mat correct_distortion(cv::Mat image_distorted, cv::Mat camera_matrix, cv::Mat distortion_coefficients);
};

#endif //STEREO_VISION_DEPTH_MAP_DISTORTION_H
