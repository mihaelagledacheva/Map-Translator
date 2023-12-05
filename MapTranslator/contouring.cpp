#include "contouring.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

void contouring::contour_mask(const cv::Mat *input_image, cv::Mat *output_image)
{
    cv::Mat gradX, gradY;
    cv::Sobel((*input_image), gradX, CV_32F, 1, 0);
    cv::Sobel((*input_image), gradY, CV_32F, 0, 1);
    
    cv::Mat gradientMagnitude;
    cv::magnitude(gradX, gradY, gradientMagnitude);
    cv::normalize(gradientMagnitude, gradientMagnitude, 0, 255, cv::NORM_MINMAX);
    gradientMagnitude.convertTo(gradientMagnitude, CV_8U);
    
    cv::Mat blurredImage, binaryImage;
    cv::GaussianBlur(gradientMagnitude, blurredImage, cv::Size(5, 5), 0);
    cv::threshold(blurredImage, binaryImage, 50, 255, cv::THRESH_BINARY);

    cv::Mat elementDilation = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));  
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
        if (hierarchy[i][3] != -1) {
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

    cv::findContours(contourImage, (*contours), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (int i = 0; i < contours->size(); i++) {
        cv::Mat mask = cv::Mat::ones(input_image->size(), CV_8UC1) * 255;
        cv::fillPoly(mask, (*contours)[i], cv::Scalar(0));
        masks->emplace_back(mask);    
    }
}

void contouring::contours(const cv::Mat *input_image, cv::Mat *output_image)
{
    cv::Mat contourImage;
    contour_mask(input_image, &contourImage);

    cv::Mat elementErosion = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::erode(contourImage, contourImage, elementErosion);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours((*output_image), contours, -1, cv::Scalar(0), 2, cv::LINE_AA);
}
