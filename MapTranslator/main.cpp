#include "discretization.h"
#include "contouring.h"
#include "patterns.h"

contouring c;
discretization d;
patterns p;

void evaluate_similarity(const cv::Mat *img1, const cv::Mat *img2) {
    int totalPixels = img1->rows * img1->cols;
    int identicalPixels = 0;
    for (int i = 0; i < img1->rows; ++i) {
        for (int j = 0; j < img1->cols; ++j) {
            if (std::abs(img1->at<uchar>(i, j) - img2->at<uchar>(i, j)) <= 1) {
                identicalPixels++;
            }
        }
    }
    double percentageIdentical = (static_cast<double>(identicalPixels) / totalPixels) * 100.0;
    std::cout << "Similarity: " << percentageIdentical << std::endl;

    double mse = 0;
    for (int i = 0; i < img1->rows; ++i) {
        for (int j = 0; j < img1->cols; ++j) {
            int pixel1 = static_cast<int>(img1->at<uchar>(i, j));
            int pixel2 = static_cast<int>(img2->at<uchar>(i, j));
            mse += ((pixel1 - pixel2) * (pixel1 - pixel2));
        }
    }
    mse = mse / totalPixels;
    std::cout << "MSE: " << mse << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <image_path> <translation>" << std::endl;
        return -1;
    }
    const std::string IMAGE_PATH = argv[1];
    const int TRANSLATION = std::stoi(argv[2]);

    cv::Mat map = cv::imread(IMAGE_PATH, cv::IMREAD_GRAYSCALE);                                // read it into an opencv matrix in grayscale
    if (map.empty()) {                                                                         // error handling
        std::cerr << "Error: Could not read the image." << std::endl;
        return 1;
    }
    cv::Mat result = cv::Mat::ones(map.size(), CV_8UC1) * 254;

    if (TRANSLATION == 0) {
        c.contours(&map, &result);
    } else {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Mat> masks;
        if (TRANSLATION == 1) {        
            c.borders(&map, &contours, &masks);
        } else {
            c.regions(&map, &contours, &masks);
        }
        for (int i = 0; i < contours.size(); i++) {
            cv::Mat blank = cv::Mat::ones(map.size(), CV_8UC1) * 254;
            d.discretize_random_sample(&contours[i], &map, &blank);
            cv::bitwise_or(blank, masks[i], masks[i]);
        }
        cv::Mat intermediate = cv::Mat::ones(map.size(), CV_8UC1) * 254;
        for (int i = 0; i < masks.size(); i++) {
            cv::bitwise_not(masks[i], masks[i]);
            cv::bitwise_xor(result, masks[i], intermediate);
        }
        // d.discretize_nn(&map, &intermediate);
        // evaluate_similarity(&map, &intermediate);
        // cv::imwrite("discretization.png", intermediate);
        cv::Mat blank = cv::Mat::ones(map.size(), CV_8UC1) * 254;
        p.create_patterns(&intermediate, &result);
        if (TRANSLATION == 2) {        
            c.contours(&map, &result);
        }
    }
    
    cv::imwrite("result.png", result);
    return 0;
}
