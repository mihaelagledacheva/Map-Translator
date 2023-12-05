#ifndef PATTERNS_H
#define PATTERNS_H

#include <opencv2/opencv.hpp>

class patterns {
public:
    void create_patterns(const cv::Mat *input_image, cv::Mat *output_image);

private:
    void generateBlueNoise(const float targetDensity, cv::Mat *output);
    cv::Point pointInContour(const std::vector<cv::Point>& contour);
};

#endif // PATTERNS_H
