#include "discretization.h"
#include "contouring.h"
#include "parameters.h"

int main() {
    cv::Mat map = cv::imread(IMAGE_PATH, cv::IMREAD_GRAYSCALE);                                // read it into an opencv matrix in grayscale
    if (map.empty()) {                                                                         // error handling
        std::cerr << "Error: Could not read the image." << std::endl;
        return 1;
    }
    cv::Mat result = cv::Mat::ones(map.size(), CV_8UC1) * 255;
    
    if (TRANSLATION == 1) {
        std::vector<std::vector<cv::Point>> borders;
        contouring::borders(&map, &borders);
        cv::fillPoly(result, borders, cv::Scalar(0));
        for (auto& border : borders) {
            cv::Mat intermediate = cv::Mat::ones(map.size(), CV_8UC1) * 255;
            discretization::discretize(&map, &intermediate, &border);
            cv::bitwise_xor(intermediate, result, result);
        }
        cv::drawContours(result, borders, -1, cv::Scalar(0), 2, cv::LINE_AA);
    }

    if (TRANSLATION == 2) {
        std::vector<std::vector<cv::Point>> contours;
        contouring::contours(&map, &result);
    }

    if (TRANSLATION == 3) {
        std::vector<std::vector<cv::Point>> borders;
        contouring::borders(&map, &borders);
        cv::fillPoly(result, borders, cv::Scalar(0));
        for (auto& border : borders) {
            cv::Mat intermediate = cv::Mat::ones(map.size(), CV_8UC1) * 255;
            discretization::discretize(&map, &intermediate, &border);
            cv::bitwise_xor(intermediate, result, result);
        }
        cv::Mat contours = cv::Mat::ones(map.size(), CV_8UC1) * 255;
        contouring::contours(&map, &contours);
        bitwise_not(contours, contours);
        bitwise_not(result, result);
        cv::bitwise_or(result, contours, result);
        bitwise_not(result, result);
    }
    
    cv::imwrite("result.jpg", result);                                                         // save the resulting image
    return 0;
}
