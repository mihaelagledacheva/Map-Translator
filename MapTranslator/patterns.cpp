// source of poisson_disk_sampling: https://stackoverflow.com/questions/32979413/infinite-blue-noise

#include "patterns.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

const int MIN_AREA = 500;

std::vector<PVector> patterns::poisson_disk_sampling(int k, int r, int size) {
    std::vector<PVector> samples;
    std::vector<PVector> active_list;
    active_list.emplace_back(static_cast<float>(rand() % size), static_cast<float>(rand() % size));

    while (!active_list.empty()) {
        int len = static_cast<int>(active_list.size());
        int index = rand() % len;
        std::swap(active_list[len - 1], active_list[index]);
        PVector sample = active_list[len - 1];
        bool found = false;

        for (int i = 0; i < k; ++i) {
            float angle = 2 * M_PI * (rand() / static_cast<float>(RAND_MAX));
            float radius = static_cast<float>(rand() % r) + r;
            PVector dv(radius * cos(angle), radius * sin(angle));
            PVector new_sample = {sample.x + dv.x, sample.y + dv.y};

            bool ok = true;
            for (const auto& existing_sample : samples) {
                float distance = std::sqrt((new_sample.x - existing_sample.x) * (new_sample.x - existing_sample.x) +
                                           (new_sample.y - existing_sample.y) * (new_sample.y - existing_sample.y));
                if (distance <= r) {
                    ok = false;
                    break;
                }
            }

            if (ok && new_sample.x >= 0 && new_sample.x < size && new_sample.y >= 0 && new_sample.y < size) {
                samples.push_back(new_sample);
                active_list.push_back(new_sample);
                len++;
                found = true;
            }
        }

        if (!found)
            active_list.pop_back();
    }

    return samples;
}

void patterns::create_patterns(const cv::Mat *input_image, cv::Mat *output_image) {
    std::vector<cv::Mat> masks;
    std::vector<std::vector<cv::Point>> all_contours;
    for (int i = 1; i < 10; i++) {
        cv::Mat binaryMask = ((*input_image) >= 25*i+1) & ((*input_image) <= 25*(i+1));
        cv::Mat mask = cv::Mat::ones(input_image->size(), CV_8UC1) * 255;
        input_image->copyTo(mask, binaryMask);
        mask.setTo(0, (mask != 255));

        if (cv::countNonZero(mask != 255) > 0) {
            cv::bitwise_not(mask, mask);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            mask = cv::Mat::ones(input_image->size(), CV_8UC1) * 255;
            for (int i = 0; i < contours.size(); i++) {
                if (cv::contourArea(contours[i]) > MIN_AREA) {
                    cv::fillPoly(mask, contours[i], cv::Scalar(0));
                    all_contours.emplace_back(contours[i]);
                }
            }

            int SIZE = std::max(input_image->rows, input_image->cols);
            int k = 30;
            int r = 5 + 2 * i;
            std::vector<PVector> samples = poisson_disk_sampling(k, r, SIZE);

            cv::Mat pattern(SIZE, SIZE, CV_8UC1, cv::Scalar(254));
            for (const auto& sample : samples) {
                cv::Point point(static_cast<int>(sample.x), static_cast<int>(sample.y));
                cv::circle(pattern, point, 2, cv::Scalar(1), -1);
            }
            cv::Rect roiRect(0, 0, input_image->cols, input_image->rows);
            pattern = pattern(roiRect).clone();
            cv::bitwise_or(pattern, mask, mask);
            masks.emplace_back(mask);
        }
    }

    for (int i = 0; i < masks.size(); i++) {
        cv::bitwise_not(masks[i], masks[i]);
        cv::bitwise_xor((*output_image), masks[i], (*output_image));
    }
    cv::drawContours((*output_image), all_contours, -1, cv::Scalar(1), 1, cv::LINE_AA);
}
