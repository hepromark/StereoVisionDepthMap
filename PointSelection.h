//
// Created by markd on 2024-03-14.
//

#ifndef STEREOVISIONDEPTHMAP_POINTSELECTION_H
#define STEREOVISIONDEPTHMAP_POINTSELECTION_H


class PointSelection {
public:
    static void mouse_handler(int event, int x, int y, int flags, void* pointData);
    static std::vector<cv::Point> getPoints(cv::Mat image, const int NUM_POINTS);
};


#endif //STEREOVISIONDEPTHMAP_POINTSELECTION_H
