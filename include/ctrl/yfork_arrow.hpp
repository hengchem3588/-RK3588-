#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

struct YforkArrowObservation
{
    cv::Rect box;
    int direction = 0;
    double correlation = 0.0;
};

class YforkArrow
{
public:
    static YforkArrowObservation inspect(const cv::Mat &image, const cv::Rect &proposal, bool tracking = false)
    {
        YforkArrowObservation result;
        if (image.empty() || image.type() != CV_8UC3 || proposal.area() <= 0)
            return result;
        const int padding = std::max(12, std::max(proposal.width, proposal.height) * 2 / 3);
        const cv::Rect region = cv::Rect(proposal.x - padding, proposal.y - padding,
            proposal.width + padding * 2, proposal.height + padding * 3) &
            cv::Rect(0, 0, image.cols, image.rows);
        if (region.area() < 36)
            return result;

        cv::Mat hsv, blue;
        cv::cvtColor(image(region), hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(85, 65, 35), cv::Scalar(140, 255, 255), blue);
        cv::morphologyEx(blue, blue, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blue, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double bestCost = 1e9;
        cv::Rect chosen;
        for (const auto &contour : contours)
        {
            const cv::Rect local = cv::boundingRect(contour);
            if (local.width < 10 || local.height < 10 || local.width > 200 || local.height > 200)
                continue;
            const double aspect = double(local.width) / local.height;
            const double area = cv::contourArea(contour);
            std::vector<cv::Point> outline;
            if (tracking)
                cv::convexHull(contour, outline);
            else
                outline = contour;
            const double shapeArea = cv::contourArea(outline);
            const double perimeter = cv::arcLength(outline, true);
            if (aspect < (tracking ? 0.45 : 0.6) || aspect > (tracking ? 3.0 : 1.55) ||
                area < local.area() * (tracking ? 0.30 : 0.45) || perimeter <= 0.0 ||
                4.0 * CV_PI * shapeArea / (perimeter * perimeter) < (tracking ? 0.45 : 0.50))
                continue;
            if (tracking && (local.width < proposal.width * 0.50 || local.height < proposal.height * 0.25))
                continue;
            const cv::Rect global(local.x + region.x, local.y + region.y, local.width, local.height);
            if (!tracking && (global.x <= 1 || global.x + global.width >= image.cols - 1))
                continue;
            if (!tracking && (global & proposal).area() < std::min(global.area(), proposal.area()) * 0.25)
                continue;
            const double distance = std::hypot(global.x + global.width * 0.5 -
                proposal.x - proposal.width * 0.5, global.y + global.height * 0.5 -
                proposal.y - proposal.height * 0.5);
            if (distance < bestCost)
            {
                bestCost = distance;
                chosen = local;
                result.box = global;
            }
        }
        if (chosen.area() == 0)
            return result;
        if (tracking)
            return result;

        cv::Mat white, inside = cv::Mat::zeros(chosen.size(), CV_8UC1);
        cv::inRange(hsv(chosen), cv::Scalar(0, 0, 125), cv::Scalar(179, 115, 255), white);
        cv::ellipse(inside, cv::Point(chosen.width / 2, chosen.height / 2),
            cv::Size(std::max(1, int(chosen.width * 0.40)),
                     std::max(1, int(chosen.height * 0.40))), 0, 0, 360, cv::Scalar(255), -1);
        cv::bitwise_and(white, inside, white);
        cv::Mat labels, stats, centroids;
        const int components = cv::connectedComponentsWithStats(white, labels, stats, centroids);
        int largest = -1;
        for (int label = 1; label < components; ++label)
            if (largest < 0 || stats.at<int>(label, cv::CC_STAT_AREA) >
                stats.at<int>(largest, cv::CC_STAT_AREA))
                largest = label;
        if (largest < 0 || stats.at<int>(largest, cv::CC_STAT_AREA) < 12 ||
            stats.at<int>(largest, cv::CC_STAT_HEIGHT) < chosen.height * 0.30 ||
            stats.at<int>(largest, cv::CC_STAT_WIDTH) < chosen.width * 0.22)
            return result;

        const double meanCol = centroids.at<double>(largest, 0);
        const double meanRow = centroids.at<double>(largest, 1);
        double covariance = 0.0;
        double varianceCol = 0.0;
        double varianceRow = 0.0;
        for (int row = 0; row < labels.rows; ++row)
            for (int col = 0; col < labels.cols; ++col)
                if (labels.at<int>(row, col) == largest)
                {
                    const double deltaCol = col - meanCol;
                    const double deltaRow = row - meanRow;
                    covariance += deltaCol * deltaRow;
                    varianceCol += deltaCol * deltaCol;
                    varianceRow += deltaRow * deltaRow;
                }
        result.correlation = covariance / std::max(1.0, std::sqrt(varianceCol * varianceRow));
        const double slope = covariance / std::max(1.0, varianceRow);
        if (std::abs(slope) >= 0.24 && std::abs(result.correlation) >= 0.38)
            result.direction = covariance < 0.0 ? -1 : 1;
        return result;
    }
};
