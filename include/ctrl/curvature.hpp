#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <vector>
#include "utils/params.hpp"

class Curvature
{
public:
    bool enabled = false;
    double previewPx = 45.0;
    double ffGain = 0.0;
    double ffLimit = 120.0;
    double minPreviewRow = 20.0;
    double aLatMax = 2.2;
    double pxPerMeter = 0.0;
    double pxPerMeterLateral = 0.0;
    double wheelbase = 0.0;
    double pwmPerRadian = 0.0;
    double rearAxleOffset = 0.0;
    double speedFloor = 0.40;
    double decelRate = 0.35;
    double accelRate = 0.08;
    double curvature = 0.0;
    double radiusM = 0.0;
    double speedLimit = 0.0;
    double previewTan = 0.0;
    bool valid = false;

    void configure(const Config &config)
    {
        enabled = config.curvatureEnabled;
        previewPx = config.curvaturePreviewPx;
        ffGain = config.curvatureFfGain;
        ffLimit = config.curvatureFfLimit;
        minPreviewRow = config.curvatureMinPreviewRow;
        aLatMax = config.curvatureALatMax;
        pxPerMeter = config.curvaturePxPerMeter;
        pxPerMeterLateral = config.curvaturePxPerMeterLateral;
        wheelbase = config.curvatureWheelbase;
        pwmPerRadian = config.curvaturePwmPerRadian;
        rearAxleOffset = config.curvatureRearAxleOffset;
        speedFloor = config.curvatureSpeedFloor;
        decelRate = config.curvatureDecelRate;
        accelRate = config.curvatureAccelRate;
        originRow_ = config.curvatureOriginRow;
        originCol_ = config.curvatureOriginCol;
        speedSmooth_ = -1.0;
    }

    double estimatePxPerMeter() const { return lastPxPerMeter_; }

    void update(std::shared_ptr<Params> &params)
    {
        curvature = radiusM = speedLimit = previewTan = 0.0;
        valid = false;
        params->ctrl.steerFF = params->ctrl.speedLimit = 0.0;
        if (!enabled || pxPerMeter <= 0.0 || pxPerMeterLateral <= 0.0 ||
            params->mode != FsmMode::NORMAL || params->ctrl.back || params->ctrl.stop ||
            params->ctrl.crossStop || params->ctrl.parking || params->ctrl.fitting || params->quit)
        {
            speedSmooth_ = -1.0;
            return;
        }
        speedLimit = std::max(std::numeric_limits<double>::min(),
                              std::min(speedFloor, double(params->config.velHigh)));
        params->ctrl.speedLimit = speedLimit;
        const auto &edge = params->ctrl.centerEdge;
        if (!validEdge(edge))
        {
            speedSmooth_ = -1.0;
            return;
        }
        const int idxFar = findLookaheadIndex(edge, previewPx * 2.4);
        const int idxMid = idxFar / 2;
        const int idxLA = findLookaheadIndex(edge, previewPx);
        if (idxMid == 0 || edge.front().x - edge[idxLA].x < minPreviewRow)
        {
            speedSmooth_ = -1.0;
            return;
        }
        const cv::Point2d origin = ipm.homography(cv::Point2d(originCol_, originRow_));
        cv::Point2d nearPoint, midPoint, farPoint, lookahead;
        if (!point2sd(edge.front(), origin, nearPoint) ||
            !point2sd(edge[idxMid], origin, midPoint) ||
            !point2sd(edge[idxFar], origin, farPoint) ||
            !point2sd(edge[idxLA], origin, lookahead) ||
            midPoint.x <= nearPoint.x || farPoint.x <= midPoint.x || lookahead.x <= 0.0)
        {
            speedSmooth_ = -1.0;
            return;
        }
        curvature = mengerCurvature(nearPoint.x, nearPoint.y, midPoint.x, midPoint.y,
                                    farPoint.x, farPoint.y, &radiusM);
        lastPxPerMeter_ = pxPerMeter;
        previewTan = lookahead.y / lookahead.x;
        valid = std::isfinite(curvature) && std::isfinite(previewTan);
        if (!valid)
        {
            speedSmooth_ = -1.0;
            return;
        }
        const double absK = std::abs(curvature);
        double target = params->config.velHigh;
        if (absK > 1e-4)
            target = std::min(target, std::max(std::numeric_limits<double>::min(),
                                             0.85 * std::sqrt(aLatMax / absK)));
        if (absK <= 1e-4 && speedSmooth_ < 0.0)
            speedLimit = 0.0;
        else
        {
            if (speedSmooth_ < 0.0)
                speedSmooth_ = target;
            const bool decelerating = target < speedSmooth_;
            const double rate = decelerating ? decelRate : accelRate;
            speedSmooth_ += (target - speedSmooth_) * rate;
            speedSmooth_ = decelerating ? std::max(speedSmooth_, target) : std::min(speedSmooth_, target);
            speedLimit = std::min(speedSmooth_, target);
            if (absK <= 1e-4 && speedSmooth_ >= params->config.velHigh - 1e-3)
            {
                speedSmooth_ = -1.0;
                speedLimit = 0.0;
            }
        }
        if (ffGain != 0.0 && wheelbase > 0.0 && pwmPerRadian != 0.0)
        {
            const double squaredDistance = lookahead.x * lookahead.x + lookahead.y * lookahead.y;
            const double angle = std::atan(wheelbase * 2.0 * lookahead.y / squaredDistance);
            params->ctrl.steerFF = std::clamp(angle * pwmPerRadian * ffGain, -ffLimit, ffLimit);
        }
        params->ctrl.speedLimit = speedLimit;
    }

private:
    double speedSmooth_ = -1.0;
    double lastPxPerMeter_ = 0.0;
    int originRow_ = ROWSIMAGE - 1;
    int originCol_ = COLSIMAGE / 2;

    static bool validEdge(const std::vector<PointX> &edge)
    {
        if (edge.size() < 12)
            return false;
        int previousRow = ROWSIMAGE;
        for (const auto &point : edge)
        {
            if (point.x < 0 || point.x >= ROWSIMAGE || point.y < 0 || point.y >= COLSIMAGE ||
                point.x > previousRow)
                return false;
            previousRow = point.x;
        }
        return edge.front().x > edge.back().x;
    }

    bool point2sd(const PointX &point, const cv::Point2d &origin, cv::Point2d &metric) const
    {
        const cv::Point2d mapped = ipm.homography(cv::Point2d(point.y, point.x));
        if (!std::isfinite(mapped.x) || !std::isfinite(mapped.y) ||
            !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
            mapped.x < 0.0 || mapped.x >= COLSIMAGEIPM || mapped.y < 0.0 || mapped.y >= ROWSIMAGEIPM)
            return false;
        metric.x = (origin.y - mapped.y) / pxPerMeter + rearAxleOffset;
        metric.y = (origin.x - mapped.x) / pxPerMeterLateral;
        return std::isfinite(metric.x) && std::isfinite(metric.y);
    }

    static int findLookaheadIndex(const std::vector<PointX> &edge, double preview)
    {
        for (size_t index = 1; index < edge.size(); ++index)
            if (edge.front().x - edge[index].x >= preview)
                return static_cast<int>(index);
        return static_cast<int>(edge.size()) - 1;
    }

    static double mengerCurvature(double ax, double ay, double bx, double by,
                                  double cx, double cy, double *radiusOut = nullptr)
    {
        const double abx = bx - ax, aby = by - ay;
        const double bcx = cx - bx, bcy = cy - by;
        const double cax = ax - cx, cay = ay - cy;
        const double denominator = std::hypot(abx, aby) * std::hypot(bcx, bcy) * std::hypot(cax, cay);
        const double value = denominator > 1e-9 ? 2.0 * (abx * bcy - aby * bcx) / denominator : 0.0;
        if (radiusOut)
            *radiusOut = std::abs(value) > 1e-9 ? 1.0 / std::abs(value) : 0.0;
        return value;
    }
};
