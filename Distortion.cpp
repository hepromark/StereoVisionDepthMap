#include "Distortion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

/**
 * Correct the distortion in an image.
 *
 * @param imaage_distorted Input distorted image.
 * @param camera_matrix Intrinsic parameters of camera that took photo.
 * @param distortion_coefficients Input array of distortion coefficients
 * @return Undistorted image of same size as input image.
 */
cv::Mat Distortion::correct_distortion(cv::Mat image_distorted, cv::Mat camera_matrix,
                                       cv::Mat distortion_coefficients) {
    //Pass by ref undistorted image
    cv::Mat image_undistorted;
    cv::Size image_size = image_distorted.size();

    //Pass by ref matix storing process that maps each pixel from distorted to undistorted image
    cv::Mat map1, map2;

    //Calculate distortion maps, then map undistorted image
    cv::initUndistortRectifyMap(camera_matrix,distortion_coefficients,
                                cv::Mat::eye(3,3, CV_64F), camera_matrix,
                                image_size, CV_32FC1, map1, map2);
    cv::remap(image_distorted,image_undistorted,map1,map2,cv::INTER_NEAREST,
              cv::BORDER_TRANSPARENT);

    return image_undistorted;
}
