#include "discretization.h"
#include <fstream>
#include <cstdlib>

sampling s;

void discretization::create_image(
    std::vector<Point> *points,
    const std::vector<Segment> *segments,
    const std::vector<cv::Point> *border,
    const cv::Mat *input_image,
    cv::Mat *output_image)
{
    VoronoiDiagram vd;
    boost::polygon::construct_voronoi(points->begin(), points->end(), &vd);
    for (int i = 0; i < vd.num_cells(); i++) {
        VoronoiCell cell = vd.cells()[i];
        std::vector<Point> vertices;
        s.get_vertices(&cell, &vertices);
        if (vertices.empty()) {
            continue;
        }
        std::vector<cv::Point> contour;
        for (const auto& vertix : vertices) {
            contour.emplace_back(cv::Point(vertix.x, vertix.y));
        }
        std::vector<std::vector<cv::Point>> contours;
        contours.emplace_back(contour);
        Point center = (*points)[cell.source_index()];
        uchar intensity = input_image->at<uchar>(center.y, center.x);
        if (intensity == 255) {
            intensity = 254;
        }
        cell.color(intensity);
        cv::drawContours((*output_image), contours, -1, cv::Scalar(intensity), cv::FILLED);
    }
}

void discretization::discretize_custom_sample(
    const std::vector<cv::Point> *border,
    const cv::Mat *input_image,
    cv::Mat *output_image)
{
    std::vector<Point> points;
    std::vector<Segment> segments;
    s.custom_sample(input_image, border, &points, &segments);
    create_image(&points, &segments, border, input_image, output_image);
}

void discretization::discretize_random_sample(
    const std::vector<cv::Point> *border,
    const cv::Mat *input_image,
    cv::Mat *output_image)
{
    std::vector<Point> points;
    std::vector<Segment> segments;
    s.random_sample(input_image, border, &points, &segments);
    create_image(&points, &segments, border, input_image, output_image);
}

void discretization::discretize_uniform_sample(
    const std::vector<cv::Point> *border,
    const cv::Mat *input_image,
    cv::Mat *output_image)
{
    std::vector<Point> points;
    std::vector<Segment> segments;
    s.uniform_sample(input_image, border, &points, &segments);
    create_image(&points, &segments, border, input_image, output_image);
}

void discretization::discretize_grid_sample(
    const std::vector<cv::Point> *border,
    const cv::Mat *input_image,
    cv::Mat *output_image)
{
    std::vector<Point> points;
    std::vector<Segment> segments;
    s.grid_sample(input_image, border, &points, &segments);
    create_image(&points, &segments, border, input_image, output_image);
}

void discretization::discretize_nn(
    const cv::Mat *input_image,
    cv::Mat *output_image
) {
    std::vector<cv::Point> border;
    border.emplace_back(cv::Point(0, 0));
    border.emplace_back(cv::Point(0, input_image->rows));
    border.emplace_back(cv::Point(input_image->cols, input_image->rows));
    border.emplace_back(cv::Point(input_image->cols, 0));
    std::vector<Point> points;
    std::vector<Segment> segments;
    s.uniform_sample(input_image, &border, &points, &segments);

    std::ofstream outFile("sample.txt");
    for (const auto& point : points) {
        outFile << point.x << " " << point.y << " " << static_cast<int>(input_image->at<uchar>(point.y, point.x)) << "\n";
    }
    outFile.close();

    std::ofstream file("grid.txt");
    for (int i = 0; i < input_image->cols; i++) {
        for (int j = 0; j < input_image->rows; j++) {
            file << i << " " << j << "\n";
        }
    }
    file.close();

    std::string string = "nn-c/nn/nnbathy -i sample.txt -o grid.txt -n " 
                        + std::to_string(input_image->rows) + "x" + std::to_string(input_image->cols) 
                        + " > nn_interpolation.txt";
    system(string.c_str());

    std::ifstream infile("nn_interpolation.txt");
    std::string line;
    while (std::getline(infile, line)) {
        double x, y, i;
        std::istringstream iss(line);
        iss >> x >> y >> i;
        output_image->at<uchar>(y, x) = i;
    }
    infile.close();
}
