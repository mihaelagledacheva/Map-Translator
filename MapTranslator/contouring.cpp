#include "contouring.h"
#include "parameters.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

void contouring::contours(const cv::Mat *image, cv::Mat *output_image)
{
    cv::Mat blurredImage;
    cv::GaussianBlur((*image), blurredImage, cv::Size(5, 5), 1);

    cv::Mat cannyImage;
    cv::Canny(blurredImage, cannyImage, 0, 10);
    std::vector<std::vector<cv::Point>> canny_contours;
    cv::findContours(cannyImage, canny_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat binaryImage = cv::Mat::zeros(cannyImage.size(), CV_8UC1);
    cv::drawContours(binaryImage, canny_contours, -1, cv::Scalar(255), 8, cv::LINE_AA);

    std::vector<std::vector<cv::Point>> raw_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryImage, raw_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours(raw_contours.size());
    for (size_t i = 0; i < raw_contours.size(); ++i) {
        cv::approxPolyDP(raw_contours[i], contours[i], 3, true);
    }

    for (int i = 0; i < contours.size(); ++i) {
        if (hierarchy[i][3] == -1) {
            cv::fillPoly((*output_image), contours[i], cv::Scalar(0));
        }
    }
    for (int i = 0; i < contours.size(); ++i) {
        if (hierarchy[i][3] != -1) {
            cv::fillPoly((*output_image), contours[i], cv::Scalar(255));
        }
    }
}

void contouring::borders(const cv::Mat *image, std::vector<std::vector<cv::Point>> *contours)
{
    cv::Mat cannyImage;
    cv::Canny((*image), cannyImage, 0, 10);
    std::vector<std::vector<cv::Point>> canny_contours;
    cv::findContours(cannyImage, canny_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat binaryImage = cv::Mat::zeros(cannyImage.size(), CV_8UC1);
    cv::drawContours(binaryImage, canny_contours, -1, cv::Scalar(255), cv::LINE_4);

    cv::Mat blurredImage;
    cv::GaussianBlur(binaryImage, blurredImage, cv::Size(5, 5), 1);

    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(blurredImage, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < all_contours.size(); i++) {
        cv::RotatedRect boundingBox = cv::minAreaRect(all_contours[i]);
        if (static_cast<int>(boundingBox.size.width) > MIN_WIDTH && 
            static_cast<int>(boundingBox.size.height) > MIN_HEIGHT &&
            cv::contourArea(all_contours[i]) > MIN_AREA) {
            contours->emplace_back(all_contours[i]);
        }
    }
}
