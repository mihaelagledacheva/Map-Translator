#ifndef CONTOURING_H
#define CONTOURING_H

#include <opencv2/opencv.hpp>

class contouring {
public:
    static void contours(const cv::Mat *image, cv::Mat *output_image);
    static void borders(const cv::Mat *image, std::vector<std::vector<cv::Point>> *contours);
};

#endif // CONTOURING_H
