#include "discretization.h"
#include "contouring.h"
#include "patterns.h"

const std::string IMAGE_PATH = "map_0.jpg";
const int TRANSLATION = 1;

contouring c;
discretization d;
patterns p;

int main() {
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
            cv::Mat intermediate = cv::Mat::ones(map.size(), CV_8UC1) * 254;
            d.discretize_voronoi(&contours[i], &map, &intermediate);
            cv::bitwise_or(intermediate, masks[i], masks[i]);
            intermediate = cv::Mat::ones(map.size(), CV_8UC1) * 254;
            // create patterns
        }
        for (int i = 0; i < masks.size(); i++) {
            cv::bitwise_not(masks[i], masks[i]);
            cv::bitwise_xor(result, masks[i], result);
        }
        if (TRANSLATION == 2) {        
            c.contours(&map, &result);
        }
    }
    
    cv::imwrite("result.jpg", result);                                                         // save the resulting image
    return 0;
}
