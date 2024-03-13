#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

// Mouse callback function - called anytime mouse input detected
void mouse_handler(int event, int x, int y, int flags, void* pointData) {

    if (event == cv::EVENT_LBUTTONDOWN) {
        //Access point data by casting generic reference into specific type
        std::vector<cv::Point>* points = (std::vector<cv::Point>*) (pointData);

        //Save user selected points
        points->push_back(cv::Point(x, y));
    }
}

/**
 * Save user selected points into a vector of points.
 *
 * @param image Image user selects points from.
 * @param NUM_POINTS Number of points user will select.
 * @return Vector of points selected by user, or vector containing ((-1,-1)) if image cannot be loaded.
 */
std::vector<cv::Point> getPoints(cv::Mat image, const int NUM_POINTS) {

    //check if image is empty, return null pointer if it is
    if(image.empty()) {
        std::cout<<"Error: Image not loaded" << std::endl;
        return std::vector<cv::Point>(1,cv::Point(-1,-1));
    }
    //Create vector of points
    std::vector<cv::Point> points;

    //Create window
    cv::namedWindow("Select Points");

    // Set mouse callback function
    cv::setMouseCallback("Select Points", mouse_handler, &points);

    // Display the image
    imshow("Select Points", image);

    // Wait until all points are selected or user closes the window
    while (cv::waitKey(1) != 'q' && points.size() < NUM_POINTS) {}

    return points;
}

int main() {

    // Load an image
    cv::Mat image = cv::imread("C:\\Users\\aleon\\CLionProjects\\Stereo Vision Depth Map\\calib_radial.jpg",
                         cv::IMREAD_GRAYSCALE);
    const int num_points = 3;
    std::vector<cv::Point> user_points = getPoints(image, num_points);
    std::cout << "Selected points: " << user_points;
    return 0;
}

