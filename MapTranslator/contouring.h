#ifndef CONTOURING_H
#define CONTOURING_H

#include <opencv2/opencv.hpp>

class contouring {
public:
    void contour(cv::Mat *image, std::vector<std::vector<cv::Point>> *contours);
};

#endif // CONTOURING_H
