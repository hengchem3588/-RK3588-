/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file yfork.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 岔路口(Y型)控制：AI识别左/右岔路标牌，选择对应分支
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "fsm/yfork.hpp"
#include "ctrl/yfork_arrow.hpp"

FsmYfork::FsmYfork(std::shared_ptr<Params> par)
    : FSMState(FsmMode::YFORK, par)
{
}

FsmYfork::~FsmYfork() = default;

FsmMode FsmYfork::getMode()
{
    if (!params->config.yfork || step == Step::WAIT || step == Step::COOLDOWN)
        return FsmMode::NORMAL;
    return FsmMode::YFORK;
}

FsmYfork::Observation FsmYfork::observe() const
{
    Observation observation;
    const PredictResult *best = nullptr;
    double bestPriority = -1.0;
    for (const auto &result : params->results)
    {
        if (!std::isfinite(result.score) || result.width <= 0 || result.height <= 0)
            continue;
        if (result.type == LABEL_FORK && result.score >= std::max(0.55f, params->config.score) &&
            result.y + result.height >= ROWSIMAGE * 0.8 && result.width >= 32)
            observation.fork = true;
        if ((result.type != LABEL_YLEFT && result.type != LABEL_YRIGHT) ||
            result.score < params->config.yforkSignScore ||
            result.y + result.height / 2 < params->config.yforkSignMinRow)
            continue;
        const double priority = result.score + 0.003 * (result.y + result.height);
        if (priority > bestPriority)
        {
            bestPriority = priority;
            best = &result;
        }
    }
    if (!best)
        return observation;
    observation.box = cv::Rect(best->x, best->y, best->width, best->height) &
        cv::Rect(0, 0, COLSIMAGE, ROWSIMAGE);
    observation.present = observation.box.area() >= 36;
    if (!observation.present)
        return observation;

    bool distinctConflict = false;
    bool labelConflict = false;
    for (const auto &result : params->results)
    {
        if ((result.type != LABEL_YLEFT && result.type != LABEL_YRIGHT) ||
            result.score < params->config.yforkSignScore || result.width <= 0 || result.height <= 0)
            continue;
        const cv::Rect other = cv::Rect(result.x, result.y, result.width, result.height) &
            cv::Rect(0, 0, COLSIMAGE, ROWSIMAGE);
        if (other.area() == 0)
            continue;
        const bool sameObject = (other & observation.box).area() >=
            std::min(other.area(), observation.box.area()) * 0.25;
        if (!sameObject && result.score >= best->score - 0.15f &&
            result.y + result.height >= best->y + best->height - 20)
            distinctConflict = true;
        if (sameObject && result.type != best->type && result.score >= best->score - 0.15f)
            labelConflict = true;
    }
    if (distinctConflict)
        return observation;
    if (params->config.yforkArrowCheck)
    {
        const auto arrow = YforkArrow::inspect(params->detectionImage, observation.box);
        if (arrow.box.area() > 0)
            observation.box = arrow.box;
        observation.direction = arrow.direction;
    }
    else if (!labelConflict)
        observation.direction = best->type == LABEL_YLEFT ? -1 : 1;
    return observation;
}

void FsmYfork::updateSign(const Observation &observation, double elapsedFrames)
{
    const cv::Rect previous = signBox;
    bool referenceUpdated = false;
    cv::Rect proposal = observation.present ? observation.box : signBox;
    const auto current = YforkArrow::inspect(params->cameraImage, proposal, true);
    if (current.box.area() > 0)
    {
        signBox = current.box;
        signLostFrames = 0.0;
        referenceUpdated = true;
    }
    else if (observation.present && params->cameraFrame >= params->detectionFrame &&
             params->cameraFrame - params->detectionFrame <= 3)
    {
        signBox = observation.box;
        signLostFrames = 0.0;
        referenceUpdated = true;
    }
    else
        signLostFrames += elapsedFrames;
    if (referenceUpdated && previous.area() > 0 && signBox.area() > 0)
    {
        const double frameSpan = std::max(1.0, elapsedFrames);
        const cv::Point2f measured(
            float((signBox.x + signBox.width * 0.5) - (previous.x + previous.width * 0.5)) / frameSpan,
            float((signBox.y + signBox.height * 0.5) - (previous.y + previous.height * 0.5)) / frameSpan);
        const cv::Point2f bounded(std::clamp(measured.x, -35.0f, 35.0f),
            std::clamp(measured.y, -35.0f, 45.0f));
        signVelocity = signVelocity * 0.55f + bounded * 0.45f;
    }
    else if (referenceUpdated)
        signVelocity = cv::Point2f(0.0f, 0.0f);
    if (signBox.area() > 0 && signLostFrames <= 6.0)
    {
        dividerCol = signBox.x + signBox.width / 2;
        const int padding = params->config.yforkSignPaddingPx;
        exclusion = cv::Rect(signBox.x - padding, signBox.y - padding,
            signBox.width + padding * 2, signBox.height + padding * 2) &
            cv::Rect(0, 0, COLSIMAGE, ROWSIMAGE);
    }
    else
        exclusion = cv::Rect();
    updateObstacle(signBox, referenceUpdated, elapsedFrames);
}

cv::Mat FsmYfork::buildObstacleMask(const cv::Rect &referenceBox) const
{
    cv::Mat mask = cv::Mat::zeros(ROWSIMAGE, COLSIMAGE, CV_8UC1);
    const cv::Rect imageRect(0, 0, COLSIMAGE, ROWSIMAGE);
    const cv::Rect reference = referenceBox & imageRect;
    if (reference.area() <= 0)
        return mask;

    const int basePadding = params->config.yforkBasePaddingPx;
    const int sidePadding = std::max(params->config.yforkObstacleSidePaddingPx,
        params->config.yforkVehicleHalfWidthPx + params->config.yforkObstacleClearancePx + basePadding);
    const int bottom = std::min(ROWSIMAGE - 1,
        reference.y + reference.height + params->config.yforkObstacleDepthPx);
    const bool clippedAtBottom = reference.y + reference.height >= ROWSIMAGE - 8;
    const int baseTop = clippedAtBottom ?
        std::max(0, reference.y + reference.height - params->config.yforkObstacleClippedExtensionPx) :
        std::min(bottom, reference.y + std::max(1, reference.height * 35 / 100));
    const int center = reference.x + reference.width / 2;
    const int topHalf = std::max(8, reference.width * 35 / 100);
    const int bottomHalf = topHalf + sidePadding;

    const cv::Rect signEnvelope(reference.x - basePadding, reference.y - basePadding,
        reference.width + basePadding * 2, reference.height + basePadding * 2);
    cv::rectangle(mask, signEnvelope & imageRect, cv::Scalar(255), cv::FILLED);
    const std::vector<cv::Point> basePolygon = {
        cv::Point(center - topHalf, baseTop),
        cv::Point(center + topHalf, baseTop),
        cv::Point(center + bottomHalf, bottom),
        cv::Point(center - bottomHalf, bottom)};
    cv::fillConvexPoly(mask, basePolygon, cv::Scalar(255));

    if (params->cameraImage.empty() || params->cameraImage.type() != CV_8UC3 ||
        params->cameraImage.size() != cv::Size(COLSIMAGE, ROWSIMAGE))
        return mask;

    const int roiPadding = sidePadding + basePadding + 8;
    const cv::Rect roi(reference.x - roiPadding,
        std::max(0, std::min(reference.y - reference.height, baseTop - reference.height / 2)),
        reference.width + roiPadding * 2,
        std::max(1, bottom - std::max(0, std::min(reference.y - reference.height,
            baseTop - reference.height / 2)) + 1));
    const cv::Rect clippedRoi = roi & imageRect;
    if (clippedRoi.area() <= 0)
        return mask;

    cv::Mat hsv, white;
    cv::cvtColor(params->cameraImage(clippedRoi), hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 130), cv::Scalar(179, 115, 255), white);
    cv::morphologyEx(white, white, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::morphologyEx(white, white, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::Mat support = mask(clippedRoi).clone();
    const int linkPadding = params->config.yforkObstacleComponentLinkPx;
    const int verticalPadding = std::max(4, linkPadding / 2);
    cv::Mat linkedSupport;
    cv::dilate(support, linkedSupport,
        cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(linkPadding * 2 + 1, verticalPadding * 2 + 1)));
    cv::Mat labels, stats, centroids;
    const int components = cv::connectedComponentsWithStats(white, labels, stats, centroids);
    const int minimumArea = params->config.yforkObstacleMinAreaPx;
    const int maximumArea = std::max(200, clippedRoi.area() * 45 / 100);
    for (int label = 1; label < components; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < minimumArea || area > maximumArea)
            continue;
        cv::Mat component = labels == label;
        cv::Mat linkedComponent;
        cv::bitwise_and(component, linkedSupport, linkedComponent);
        if (cv::countNonZero(linkedComponent) < std::max(3, minimumArea / 8))
            continue;
        cv::bitwise_or(mask(clippedRoi), component, mask(clippedRoi));
    }
    return mask;
}

void FsmYfork::updateObstacle(const cv::Rect &referenceBox, bool referenceUpdated,
                              double elapsedFrames)
{
    const cv::Rect reference = referenceBox & cv::Rect(0, 0, COLSIMAGE, ROWSIMAGE);
    if (referenceUpdated && reference.area() > 0)
    {
        obstacleMask = buildObstacleMask(reference);
        if (obstacleMask.empty())
            return;
        obstacleAgeFrames = 0.0;
        obstacleTracked = true;
    }
    else if (!obstacleMask.empty())
    {
        obstacleAgeFrames += elapsedFrames;
        if (obstacleAgeFrames <= params->config.yforkObstacleMemoryFrames)
        {
            const double frames = std::max(0.0, elapsedFrames);
            const double dx = std::clamp(double(signVelocity.x) * frames, -35.0, 35.0);
            double dy = std::clamp(double(signVelocity.y) * frames, -45.0, 45.0);
            const cv::Rect bounds = cv::boundingRect(obstacleMask);
            if (dy > 0.0)
                dy = std::min(dy, double(std::max(0, ROWSIMAGE - bounds.y - bounds.height)));
            cv::Mat shifted = cv::Mat::zeros(obstacleMask.size(), obstacleMask.type());
            const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1.0, 0.0, dx, 0.0, 1.0, dy);
            cv::warpAffine(obstacleMask, shifted, transform, obstacleMask.size(),
                cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
            obstacleMask = std::move(shifted);
            obstacleTracked = true;
        }
        else
        {
            obstacleMask.release();
            obstacleBox = cv::Rect();
            obstacleTracked = false;
        }
    }
    if (obstacleTracked && !obstacleMask.empty())
        obstacleBox = cv::boundingRect(obstacleMask);
    else
        obstacleBox = cv::Rect();
}

bool FsmYfork::obstacleSpanAt(int row, int &left, int &right) const
{
    left = COLSIMAGE;
    right = -1;
    if (!obstacleTracked || obstacleMask.empty() ||
        obstacleAgeFrames > params->config.yforkObstacleMemoryFrames)
        return false;
    const int radius = params->config.yforkObstacleRowTolerancePx +
        params->config.yforkVehicleFrontPx;
    const int firstRow = std::max(0, row - radius);
    const int lastRow = std::min(ROWSIMAGE - 1, row + radius);
    for (int currentRow = firstRow; currentRow <= lastRow; ++currentRow)
    {
        const uchar *pixels = obstacleMask.ptr<uchar>(currentRow);
        int begin = -1;
        for (int col = 0; col <= COLSIMAGE; ++col)
        {
            const bool occupied = col < COLSIMAGE && pixels[col] > 127;
            if (occupied && begin < 0)
                begin = col;
            if ((!occupied || col == COLSIMAGE) && begin >= 0)
            {
                if (col - begin >= 2)
                {
                    left = std::min(left, begin);
                    right = std::max(right, col - 1);
                }
                begin = -1;
            }
        }
    }
    return right >= left;
}

bool FsmYfork::lastSafePathClear() const
{
    if (lastSafePath.size() < 25)
        return false;
    for (const auto &point : lastSafePath)
    {
        int obstacleLeft = 0;
        int obstacleRight = -1;
        if (obstacleSpanAt(point.x, obstacleLeft, obstacleRight))
        {
            const int row = std::clamp(point.x, 0, ROWSIMAGE - 1);
            const int requiredClearance = std::max(6,
                (params->config.yforkVehicleHalfWidthPx + params->config.yforkObstacleClearancePx) *
                    row / ROWSIMAGE);
            if (point.y >= obstacleLeft - requiredClearance && point.y <= obstacleRight + requiredClearance)
                return false;
        }
        if (params->config.obstacle)
            for (const auto &result : params->results)
            {
                if ((result.type != LABEL_CONE && result.type != LABEL_PERSON) ||
                    result.score < params->config.score || result.width <= 0 || result.height <= 0)
                    continue;
                const int bottom = result.y + result.height;
                if (point.x >= std::max(100, bottom - 12) && point.x <= bottom + 18 &&
                    point.y >= result.x - params->config.yforkClearancePx &&
                    point.y <= result.x + result.width + params->config.yforkClearancePx)
                    return false;
            }
    }
    return true;
}

bool FsmYfork::restoreLastSafePath()
{
    const int maxAgeFrames = std::max(4, params->config.yforkLostPathFrames);
    if (direction == 0 || direction != lastSafePathDirection ||
        lastSafePathAgeFrames > maxAgeFrames || !lastSafePathClear())
        return false;
    params->ctrl.centerEdge = lastSafePath;
    params->ctrl.fitting = true;
    params->ctrl.yforkPath = true;
    params->ctrl.lineArea = params->track->lineArea;
    params->ctrl.yforkPreviewRow = lastSafePreviewRow;
    seedCol = lastSafePath.front().y;
    return true;
}

void FsmYfork::updateSpeedCap(double target, double elapsedFrames)
{
    target = std::max(0.0, target);
    if (!yforkSpeedCapInitialized)
    {
        yforkSpeedCapCurrent = std::max(target, double(params->ctrl.speed));
        yforkSpeedCapInitialized = true;
    }
    const double slew = std::max(0.02, double(params->config.speedSlew));
    const double step = slew * std::max(0.0, elapsedFrames);
    if (target < yforkSpeedCapCurrent)
        yforkSpeedCapCurrent = std::max(target, yforkSpeedCapCurrent - step);
    else
        yforkSpeedCapCurrent = std::min(target, yforkSpeedCapCurrent + step);
    params->ctrl.yforkSpeedCap = yforkSpeedCapCurrent;
}

void FsmYfork::applyObstacleSpeedCap(double elapsedFrames)
{
    double target = params->config.velYFork;
    if (minObstacleClearance >= 0 &&
        minObstacleClearance <= params->config.yforkObstacleSlowClearancePx)
        target = std::min(target, double(params->config.yforkObstacleSpeed));
    updateSpeedCap(target, elapsedFrames);
}

bool FsmYfork::planBranch(const Mat &img)
{
    const auto rawLeft = params->track->pointsEdgeLeft;
    const auto rawRight = params->track->pointsEdgeRight;
    geometry = params->track->handleBranch(img, direction < 0, seedCol, dividerCol, exclusion);
    normalAligned = false;
    minClearance = 0;
    minRoadClearance = -1;
    minObstacleClearance = -1;
    minObstacleRow = minObstaclePathCol = obstacleLeftAtMin = obstacleRightAtMin = -1;
    planBlockedByObstacle = false;
    if (!geometry.valid)
        return false;

    std::vector<PointX> path;
    int clearance = COLSIMAGE;
    double alignmentError = 0.0;
    double alignmentMax = 0.0;
    int alignmentCount = 0;
    const auto columnAt = [](const std::vector<PointX> &edge, int row)
    {
        int distance = ROWSIMAGE;
        int column = -1;
        for (const auto &point : edge)
            if (std::abs(point.x - row) < distance)
            {
                distance = std::abs(point.x - row);
                column = point.y;
            }
        return distance <= 3 ? column : -1;
    };
    for (size_t index = 0; index < params->track->pointsEdgeLeft.size(); ++index)
    {
        const auto &left = params->track->pointsEdgeLeft[index];
        const auto &right = params->track->pointsEdgeRight[index];
        const int width = right.y - left.y;
        const int row = std::clamp(left.x, 0, ROWSIMAGE - 1);
        const int rowScaleMargin = std::max(params->config.yforkClearancePx,
            params->config.yforkVehicleHalfWidthPx) * row / ROWSIMAGE;
        const int margin = std::max(5, rowScaleMargin);
        if (width < margin * 2 + 8)
            break;
        const double bias = geometry.nearSplitRows >= 2 || exclusion.area() > 0 ?
            params->config.yforkOutsideBias : 0.0;
        double column = left.y + width * (0.5 + direction * bias);
        if (!path.empty())
            column = column * 0.65 + path.back().y * 0.35;
        column = std::clamp(column, double(left.y + margin), double(right.y - margin));
        int obstacleLeft = 0;
        int obstacleRight = -1;
        if (obstacleSpanAt(left.x, obstacleLeft, obstacleRight))
        {
            const int requiredClearance = std::max(6,
                (params->config.yforkVehicleHalfWidthPx + params->config.yforkObstacleClearancePx) *
                    row / ROWSIMAGE);
            const double leftSafe = obstacleLeft - requiredClearance;
            const double rightSafe = obstacleRight + requiredClearance;
            const bool leftCorridor = left.y + margin <= leftSafe;
            const bool rightCorridor = right.y - margin >= rightSafe;
            if (!leftCorridor && !rightCorridor)
            {
                planBlockedByObstacle = true;
                return false;
            }
            if (direction < 0)
            {
                column = leftCorridor ? std::min(column, leftSafe) : std::max(column, rightSafe);
            }
            else
            {
                column = rightCorridor ? std::max(column, rightSafe) : std::min(column, leftSafe);
            }
            column = std::clamp(column, double(left.y + margin), double(right.y - margin));
            const double obstacleClearance = column < obstacleLeft ?
                obstacleLeft - column - requiredClearance :
                column > obstacleRight ? column - obstacleRight - requiredClearance : -requiredClearance;
            const int obstacleClearancePx = int(std::floor(obstacleClearance));
            if (minObstacleRow < 0 || obstacleClearancePx < minObstacleClearance)
            {
                minObstacleClearance = obstacleClearancePx;
                minObstacleRow = left.x;
                minObstaclePathCol = int(std::lround(column));
                obstacleLeftAtMin = obstacleLeft;
                obstacleRightAtMin = obstacleRight;
            }
            clearance = std::min(clearance, obstacleClearancePx);
            if (obstacleClearance < 0.0)
            {
                planBlockedByObstacle = true;
                return false;
            }
        }
        path.emplace_back(left.x, int(std::lround(column)));
        if (left.x >= 100)
        {
            const int roadClearance = std::min(path.back().y - left.y, right.y - path.back().y);
            minRoadClearance = minRoadClearance < 0 ? roadClearance :
                std::min(minRoadClearance, roadClearance);
            clearance = std::min(clearance, roadClearance);
        }
        if (std::abs(left.x - params->config.yforkPreviewRow) <= 30 || left.x >= 195)
        {
            const int rawLeftCol = columnAt(rawLeft, left.x);
            const int rawRightCol = columnAt(rawRight, left.x);
            if (rawLeftCol >= 0 && rawRightCol > rawLeftCol)
            {
                const double delta = std::abs((rawLeftCol + rawRightCol) * 0.5 - column);
                alignmentError += delta;
                alignmentMax = std::max(alignmentMax, delta);
                ++alignmentCount;
            }
        }
    }
    if (path.size() < 25 || path.front().x < 170 ||
        path.back().x > params->config.yforkPreviewRow + 12 || path.front().x - path.back().x < 60)
        return false;
    minClearance = clearance;
    normalAligned = alignmentCount >= 8 && alignmentError / alignmentCount <= 16.0 && alignmentMax <= 30.0;
    params->ctrl.centerEdge = std::move(path);
    params->ctrl.fitting = true;
    params->ctrl.yforkPath = true;
    params->ctrl.lineArea = params->track->lineArea;
    params->ctrl.yforkPreviewRow = params->config.yforkPreviewRow;
    if (geometry.splitTipRow >= 150)
        params->ctrl.yforkPreviewRow = std::max(params->ctrl.yforkPreviewRow,
            double(std::min(180, geometry.splitTipRow - 25)));
    if (exclusion.area() > 0 && exclusion.y + exclusion.height >= 130)
        params->ctrl.yforkPreviewRow = std::max(params->ctrl.yforkPreviewRow,
            double(std::min(180, exclusion.y + exclusion.height / 2 + 15)));
    if (params->config.obstacle)
        for (const auto &result : params->results)
        {
            if ((result.type != LABEL_CONE && result.type != LABEL_PERSON) ||
                result.score < params->config.score || result.width <= 0 || result.height <= 0)
                continue;
            const int bottom = result.y + result.height;
            for (const auto &point : params->ctrl.centerEdge)
                if (point.x >= std::max(100, bottom - 12) && point.x <= bottom + 18 &&
                    point.y >= result.x - params->config.yforkClearancePx &&
                    point.y <= result.x + result.width + params->config.yforkClearancePx)
                {
                    planBlockedByObstacle = true;
                }
        }
    seedCol = params->ctrl.centerEdge.front().y;
    if (!planBlockedByObstacle)
    {
        lastSafePath = params->ctrl.centerEdge;
        lastSafePathDirection = direction;
        lastSafePreviewRow = params->ctrl.yforkPreviewRow;
        lastSafePathAgeFrames = 0.0;
    }
    sawSplit = sawSplit || geometry.nearSplitRows >= 4 || exclusion.area() > 0;
    obstacleWasNear = obstacleWasNear || geometry.splitTipRow >= 160 ||
        (signLostFrames <= 6.0 && signBox.y + signBox.height >= 120);
    return !planBlockedByObstacle;
}

bool FsmYfork::exitReady() const
{
    const bool signClear = signLostFrames > 6.0 || signBox.x + signBox.width < 35 || signBox.x > COLSIMAGE - 35;
    return sawSplit && obstacleWasNear && turnFrames >= params->config.yforkMinTurnFrames &&
        geometry.valid && geometry.nearSplitRows <= 1 && normalAligned && signClear;
}

void FsmYfork::run(Mat &img)
{
    if (!params->config.yfork)
        return;
    const double elapsedFrames = std::clamp(params->ctrl.frameDt, 1.0 / 120.0, 0.2) * 30.0;
    lastSafePathAgeFrames = std::min(1e9, lastSafePathAgeFrames + elapsedFrames);
    stateFrames += elapsedFrames;
    noObservationFrames += elapsedFrames;
    Observation observation;
    const bool freshObservation = params->detectionFrame != 0 && params->detectionFrame != lastDetectionFrame;
    if (freshObservation)
    {
        lastDetectionFrame = params->detectionFrame;
        noObservationFrames = 0.0;
        observation = observe();
    }
    if (step == Step::SAFE_STOP)
    {
        updateSign(observation, elapsedFrames);
        bool recovered = direction != 0 && planBranch(img);
        if (!recovered)
            recovered = restoreLastSafePath();
        if (recovered)
        {
            params->ctrl.yforkStop = false;
            lostPathFrames = 0;
            stopReason.clear();
            updateSpeedCap(params->config.velYFork, elapsedFrames);
            setStep(Step::TURN);
            return;
        }
        params->ctrl.yforkStop = true;
        if (stateFrames >= 60.0 && !planBlockedByObstacle)
        {
            std::cerr << "[Yfork] Safety stop timeout, resume normal following" << std::endl;
            params->ctrl.yforkPath = false;
            params->ctrl.yforkSpeedCap = 0.0;
            params->ctrl.yforkStop = false;
            setStep(Step::COOLDOWN);
        }
        return;
    }
    params->ctrl.yforkStop = false;
    if (step == Step::COOLDOWN)
    {
        updateSpeedCap(params->config.velYFork, elapsedFrames);
        if (observation.present)
            stateFrames = 0.0;
        if (stateFrames >= params->config.yforkCooldownFrames)
            reset();
        return;
    }
    if (step == Step::WAIT)
    {
        if (freshObservation)
            forkVotes = observation.fork ? forkVotes + 1 : 0;
        if (!observation.present && forkVotes < 2)
            return;
        setStep(Step::APPROACH);
    }
    updateSign(observation, elapsedFrames);
    if (step == Step::APPROACH)
    {
        updateSpeedCap(std::min(params->config.yforkApproachSpeed, params->config.velYFork), elapsedFrames);
        if (freshObservation)
        {
            if (observation.direction != 0)
            {
                if (candidateDirection != observation.direction)
                {
                    candidateDirection = observation.direction;
                    votes = 0;
                }
                ++votes;
            }
            else
            {
                votes = 0;
                candidateDirection = 0;
            }
        }
        if (votes >= params->config.yforkSignConfirmFrames)
        {
            direction = candidateDirection;
            turnFrames = 0.0;
            seedCol = COLSIMAGE / 2;
            setStep(Step::TURN);
        }
        else
        {
            bool provisionalPath = false;
            if (candidateDirection != 0)
            {
                const int lockedDirection = direction;
                direction = candidateDirection;
                provisionalPath = planBranch(img);
                direction = lockedDirection;
                if (provisionalPath)
                    applyObstacleSpeedCap(elapsedFrames);
            }
            if (stateFrames >= params->config.yforkMaxTurnFrames)
            {
                std::cerr << "[Yfork] Sign unconfirmed, resume normal following" << std::endl;
                setStep(Step::COOLDOWN);
            }
            return;
        }
    }
    if (step == Step::TURN || step == Step::EXIT)
    {
        turnFrames += elapsedFrames;
        updateSpeedCap(params->config.velYFork, elapsedFrames);
        if (turnFrames >= params->config.yforkMaxTurnFrames)
        {
            // 能走到这里，说明安全走廊一直是通的（否则早被 lostPathFrames 拦下），
            // 已经连续跟了 8 秒、远超一个岔路口的长度。此时停车只会丢分，直接收尾。
            std::cerr << "[Yfork] Turn timeout, fall back to normal following" << std::endl;
            setStep(Step::COOLDOWN);
            return;
        }
        if (!planBranch(img))
        {
            ++lostPathFrames;
            exitVotes = 0;
            if (restoreLastSafePath())
            {
                lostPathFrames = 0;
                applyObstacleSpeedCap(elapsedFrames);
                return;
            }
            if (planBlockedByObstacle || lostPathFrames >= params->config.yforkLostPathFrames)
                safeStop("NO_SAFE_CORRIDOR");
            return;
        }
        lostPathFrames = 0;
        applyObstacleSpeedCap(elapsedFrames);
        if (step == Step::TURN)
        {
            exitVotes = exitReady() ? exitVotes + 1 : 0;
            if (exitVotes >= params->config.yforkExitConfirmFrames)
                setStep(Step::EXIT);
        }
        else if (!exitReady())
        {
            exitVotes = 0;
            setStep(Step::TURN);
        }
        else if (stateFrames >= params->config.yforkExitConfirmFrames)
            setStep(Step::COOLDOWN);
    }
}

void FsmYfork::safeStop(const std::string &reason)
{
    stopReason = reason;
    setStep(Step::SAFE_STOP);
    params->ctrl.yforkStop = true;
    params->ctrl.speed = 0.0f;
    std::cerr << "[Yfork] Safety stop: " << reason << std::endl;
}

const char *FsmYfork::stepName() const
{
    static const char *const names[] = {"WAIT", "APPROACH", "TURN", "EXIT", "COOLDOWN", "SAFE_STOP"};
    return names[static_cast<size_t>(step)];
}

FsmYfork::Status FsmYfork::status() const
{
    Status value;
    value.step = stepName();
    value.direction = direction;
    value.votes = votes;
    value.nearSplitRows = geometry.nearSplitRows;
    value.splitTipRow = geometry.splitTipRow;
    value.minClearance = minClearance;
    value.minRoadClearance = minRoadClearance;
    value.minObstacleClearance = minObstacleClearance;
    value.minObstacleRow = minObstacleRow;
    value.minObstaclePathCol = minObstaclePathCol;
    value.obstacleLeftAtMin = obstacleLeftAtMin;
    value.obstacleRightAtMin = obstacleRightAtMin;
    value.lostPathFrames = lostPathFrames;
    value.normalAligned = normalAligned;
    value.obstacleTracked = obstacleTracked;
    value.obstacleMemoryFrames = std::max(0, int(std::ceil(
        params->config.yforkObstacleMemoryFrames - obstacleAgeFrames)));
    value.exclusion = exclusion;
    value.obstacleBox = obstacleBox;
    value.stopReason = stopReason;
    return value;
}

void FsmYfork::show(Mat &img)
{
    if (!params->config.yfork)
        return;
    const cv::Scalar color = params->ctrl.yforkStop ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    if (exclusion.area() > 0)
        cv::rectangle(img, exclusion, cv::Scalar(0, 0, 255), 1);
    if (obstacleBox.area() > 0)
        cv::rectangle(img, obstacleBox, cv::Scalar(0, 165, 255), 1);
    putText(img, std::string("Y:") + stepName() + " Dir:" +
        (direction == 0 ? "?" : direction < 0 ? "L" : "R") + " Vote:" + std::to_string(votes),
        Point(8, 36), FONT_HERSHEY_PLAIN, 0.9, color, 1);
    putText(img, "Split:" + std::to_string(geometry.nearSplitRows) + " Tip:" +
        std::to_string(geometry.splitTipRow) + " Clear:" + std::to_string(minClearance),
        Point(8, 52), FONT_HERSHEY_PLAIN, 0.9, color, 1);
    putText(img, "T:" + std::to_string(int(turnFrames)) + " Lost:" + std::to_string(lostPathFrames) +
        " Exit:" + std::to_string(exitVotes), Point(8, 68), FONT_HERSHEY_PLAIN, 0.9, color, 1);
    putText(img, "Obs:" + std::to_string(obstacleTracked), Point(8, 84),
        FONT_HERSHEY_PLAIN, 0.9, color, 1);
    putText(img, "YStop:" + std::to_string(params->ctrl.yforkStop) + " Cap:" +
        doble2String(params->ctrl.yforkSpeedCap, 2), Point(8, 100), FONT_HERSHEY_PLAIN, 0.9, color, 1);
    if (!stopReason.empty())
        putText(img, stopReason, Point(8, 116), FONT_HERSHEY_PLAIN, 0.9, color, 1);
}

void FsmYfork::setStep(Step next)
{
    if (params->config.debug)
        std::cout << "[Yfork] " << stepName() << " -> ";
    step = next;
    stateFrames = 0.0;
    if (params->config.debug)
        std::cout << stepName() << std::endl;
}

void FsmYfork::reset()
{
    direction = 0;
    candidateDirection = 0;
    votes = forkVotes = exitVotes = lostPathFrames = 0;
    seedCol = dividerCol = COLSIMAGE / 2;
    signBox = exclusion = obstacleBox = cv::Rect();
    lastSafePath.clear();
    lastSafePathDirection = 0;
    lastSafePathAgeFrames = 1e9;
    lastSafePreviewRow = params->config.yforkPreviewRow;
    yforkSpeedCapCurrent = 0.0;
    yforkSpeedCapInitialized = false;
    params->ctrl.yforkSpeedCap = 0.0;
    params->ctrl.yforkPath = false;
    obstacleMask.release();
    signVelocity = cv::Point2f(0.0f, 0.0f);
    signLostFrames = turnFrames = obstacleAgeFrames = 0.0;
    obstacleTracked = false;
    sawSplit = obstacleWasNear = normalAligned = false;
    geometry = Track::BranchGeometry();
    minClearance = 0;
    minRoadClearance = minObstacleClearance = -1;
    minObstacleRow = minObstaclePathCol = obstacleLeftAtMin = obstacleRightAtMin = -1;
    stopReason.clear();
    params->ctrl.yforkStop = false;
    setStep(Step::WAIT);
}
