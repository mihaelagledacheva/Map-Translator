#ifndef CONTOURING_H
#define CONTOURING_H

#include <opencv2/opencv.hpp>

class contouring {
public:
    void regions(const cv::Mat *input_image, std::vector<std::vector<cv::Point>> *contours, std::vector<cv::Mat> *masks);
    void borders(const cv::Mat *input_image, std::vector<std::vector<cv::Point>> *contours, std::vector<cv::Mat> *masks);
    void contours(const cv::Mat *input_image, cv::Mat *output_image);

private:
    void contour_mask(const cv::Mat *input_image, cv::Mat *output_image);
};

#endif // CONTOURING_H
