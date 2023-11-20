#include "discretization.h"
#include "contouring.h"

std::string IMAGE_PATH = "map.png";
int CONTOURS = 0;

//From an input image of a map, generate and visualize the Voronoi diagrams in 5 output images voronoi_cells_n
int main() {
    cv::Mat map = cv::imread(IMAGE_PATH, cv::IMREAD_GRAYSCALE);                                // read it into an opencv matrix in grayscale
    if (map.empty()) {                                                                         // error handling
        std::cerr << "Error: Could not read the image." << std::endl;
        return 1;
    }
    cv::Mat translation = map.clone();
    std::vector<std::vector<cv::Point>> contours;
    if (CONTOURS) {
        std::vector<std::vector<cv::Point>> contours;
        contouring c;
        c.contour(&map, &contours);
        cv::drawContours(translation, contours, -1, cv::Scalar(255,0,0), 2, cv::LINE_AA);      //    draw contours on the image copy _ to modify parameters : (copy, contours, contourIndex, color, thickness, linetype)                                                                                       //    contourIndex : e.g -1 for "draw all contours" ; color: cv::Scalar(B,G,R) ; tickness : n pixels ; linetype : e.g  anti-aliased lines
        
    } else {
        std::vector<cv::Point> contour;
        contour.emplace_back(cv::Point(0, 0));
        contour.emplace_back(cv::Point(map.cols, 0));
        contour.emplace_back(cv::Point(map.cols, map.rows));
        contour.emplace_back(cv::Point(0, map.rows));
        discretization d;
        d.discretize(&map, &translation, &contour);
        cv::imwrite("translation.jpg", translation);                                           // save the resulting image
    }
    return 0;
}
