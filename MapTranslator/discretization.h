#ifndef DISCRETIZATION_H
#define DISCRETIZATION_H

#include "sampling.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/geometry.hpp>
#include <boost/polygon/voronoi.hpp>

class discretization {
public:
    void discretize_custom_sample(const std::vector<cv::Point> *contour, const cv::Mat *input_image, cv::Mat *output_image);
    void discretize_random_sample(const std::vector<cv::Point> *contour, const cv::Mat *input_image, cv::Mat *output_image);
    void discretize_uniform_sample(const std::vector<cv::Point> *contour, const cv::Mat *input_image, cv::Mat *output_image);
    void discretize_grid_sample(const std::vector<cv::Point> *contour, const cv::Mat *input_image, cv::Mat *output_image);
    void discretize_nn(const cv::Mat *input_image,cv::Mat *output_image);

private:
    void create_image(std::vector<Point> *points, const std::vector<Segment> *segments, const std::vector<cv::Point> *border, const cv::Mat *input_image, cv::Mat *output_image);
};

#endif // DISCRETIZATION_H
