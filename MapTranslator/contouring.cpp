#include "contouring.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

const int MIN_AREA = 100;

void contouring::contour_mask(const cv::Mat *input_image, cv::Mat *output_image)
{
    cv::Mat binaryImage;
    cv::threshold((*input_image), binaryImage, 20, 255, cv::THRESH_BINARY);
    cv::bitwise_not(binaryImage, binaryImage);

    cv::Mat elementDilation = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));  
    cv::dilate(binaryImage, (*output_image), elementDilation);
}

void contouring::regions(const cv::Mat *input_image, std::vector<std::vector<cv::Point>> *contours, std::vector<cv::Mat> *masks)
{
    cv::Mat contourImage = cv::Mat::zeros(input_image->size(), CV_8UC1);
    contour_mask(input_image, &contourImage);

    std::vector<std::vector<cv::Point>> all_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(contourImage, all_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < all_contours.size(); i++) {
        if (hierarchy[i][3] != -1 && cv::contourArea(all_contours[i]) > MIN_AREA) {
            cv::Mat mask = cv::Mat::ones(input_image->size(), CV_8UC1) * 255;
            cv::fillPoly(mask, all_contours[i], cv::Scalar(0));
            masks->emplace_back(mask);
            contours->emplace_back(all_contours[i]);
        }
    }
}

void contouring::borders(const cv::Mat *input_image, std::vector<std::vector<cv::Point>> *contours, std::vector<cv::Mat> *masks)
{
    cv::Mat contourImage;
    contour_mask(input_image, &contourImage);

    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(contourImage, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (int i = 0; i < all_contours.size(); i++) {
        if (cv::contourArea(all_contours[i]) > MIN_AREA) {
            cv::Mat mask = cv::Mat::ones(input_image->size(), CV_8UC1) * 255;
            cv::fillPoly(mask, all_contours[i], cv::Scalar(0));
            masks->emplace_back(mask);
            contours->emplace_back(all_contours[i]);
        }
    }
}

void contouring::contours(const cv::Mat *input_image, cv::Mat *output_image)
{
    cv::Mat contourImage;
    contour_mask(input_image, &contourImage);

    cv::Mat elementErosion = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::erode(contourImage, contourImage, elementErosion);

    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(contourImage, all_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours;
    for (int i = 0; i < all_contours.size(); i++) {
        if (cv::contourArea(all_contours[i]) > MIN_AREA) {
            contours.emplace_back(all_contours[i]);
        }
    }
    cv::drawContours((*output_image), contours, -1, cv::Scalar(0), 2, cv::LINE_AA);
}
