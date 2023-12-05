#include "patterns.h"
#include "contouring.h"
#include <cmath>
#include <iostream>
#include <random>
#include <queue>

contouring contour;

void patterns::generateBlueNoise(
    const float targetDensity, 
    cv::Mat *output) 
{
    // needs rework
    std::default_random_engine rng(std::random_device{}());
    std::uniform_real_distribution<float> distribution(0, 1);

    const int width = output->cols;
    const int height = output->rows;
    const float minDistance = std::sqrt(1.0f / targetDensity);
    const float cellSize = minDistance / std::sqrt(2.0f);
    const int gridSizeX = static_cast<int>(std::ceil(width / cellSize));
    const int gridSizeY = static_cast<int>(std::ceil(height / cellSize));

    std::vector<cv::Point2i> grid(gridSizeX * gridSizeY, cv::Point2i(-1, -1));
    std::vector<cv::Point2f> points;
    std::queue<cv::Point2i> activeQueue;

    cv::Point2f initialPoint(distribution(rng) * width, distribution(rng) * height);
    activeQueue.push(cv::Point2i(static_cast<int>(initialPoint.x / cellSize),
                                  static_cast<int>(initialPoint.y / cellSize)));
    points.push_back(initialPoint);

    while (!activeQueue.empty()) {
        cv::Point2i currentCell = activeQueue.front();
        activeQueue.pop();

        for (int i = 0; i < 30; ++i) {
            float angle = distribution(rng) * 2.0f * static_cast<float>(CV_PI);
            float distance = distribution(rng) * minDistance + minDistance;
            cv::Point2f candidatePos = points.back() + cv::Point2f(std::cos(angle), std::sin(angle)) * distance;

            int candidateCellX = static_cast<int>(candidatePos.x / cellSize);
            int candidateCellY = static_cast<int>(candidatePos.y / cellSize);

            if (candidateCellX >= 0 && candidateCellX < gridSizeX &&
                candidateCellY >= 0 && candidateCellY < gridSizeY &&
                grid[candidateCellY * gridSizeX + candidateCellX] == cv::Point2i(-1, -1)) {
                activeQueue.push(cv::Point2i(candidateCellX, candidateCellY));
                grid[candidateCellY * gridSizeX + candidateCellX] = cv::Point2i(static_cast<int>(points.size()) - 1, 0);
                points.push_back(candidatePos);
            }
        }
    }

    for (const auto& point : points) {
        cv::circle((*output), cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)), 10, cv::Scalar(1), -1);
    }
}

cv::Point patterns::pointInContour(const std::vector<cv::Point>& contour) {
    cv::Rect boundingRect = cv::boundingRect(contour);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> randomX(boundingRect.x + 1, boundingRect.x + boundingRect.width - 2);
    std::uniform_int_distribution<int> randomY(boundingRect.y + 1, boundingRect.y + boundingRect.height - 2);

    cv::Point randomPoint;
    do {
        randomPoint.x = randomX(gen);
        randomPoint.y = randomY(gen);
    } while (cv::pointPolygonTest(contour, randomPoint, false) <= 0);

    return randomPoint;
}

void patterns::create_patterns(
    const cv::Mat *input_image,
    cv::Mat *output_image)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Mat> masks;
    contour.regions(input_image, &contours, &masks);
    for (int i = 0; i < contours.size(); i++) {
        cv::Point point = pointInContour(contours[i]);
        uchar intensity = input_image->at<uchar>(point.y, point.x);
        float density = intensity / 255;
        cv::Mat intermediate = cv::Mat::ones(input_image->size(), CV_8UC1) * 254;
        generateBlueNoise(density, &intermediate);
        cv::bitwise_or(intermediate, masks[i], masks[i]);
        cv::imwrite("imh.jpg", masks[i]);
    }
    for (int i = 0; i < masks.size(); i++) {
        cv::bitwise_not(masks[i], masks[i]);
        cv::bitwise_xor((*output_image), masks[i], (*output_image));
    }
    contour.contours(input_image, output_image);
}
