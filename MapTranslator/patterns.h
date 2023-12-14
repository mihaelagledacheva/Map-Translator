#ifndef PATTERNS_H
#define PATTERNS_H

#include <opencv2/opencv.hpp>

class PVector {
public:
    float x, y;
    PVector(float _x, float _y) : x(_x), y(_y) {}
};

class patterns {
public:
    void create_patterns(const cv::Mat *input_image, cv::Mat *output_image);

private:
    std::vector<PVector> poisson_disk_sampling(int k, int r, int size);
};

#endif // PATTERNS_H
