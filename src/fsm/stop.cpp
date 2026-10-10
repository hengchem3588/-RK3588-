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
 * @file stop.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车区规划与控制
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/stop.hpp"
#include <algorithm>

namespace
{
// 黑黄相间横杆在反相二值图中可出现两种极性；检测上下背景相同、条带与背景相反。
bool hasHorizontalGateBar(const Mat &img, const cv::Rect &roi)
{
    for (int radius = 2; radius <= 8; radius += 2)
    {
        for (int r = roi.y + radius; r < roi.y + roi.height - radius; r += 2)
        {
            const uchar *row = img.ptr<uchar>(r);
            const uchar *above = img.ptr<uchar>(r - radius);
            const uchar *below = img.ptr<uchar>(r + radius);
            int support = 0, samples = 0, first = -1, last = -1;
            for (int c = roi.x; c < roi.x + roi.width; c += 2)
            {
                samples++;
                if (above[c] == below[c] && row[c] != above[c])
                {
                    support++;
                    if (first < 0)
                        first = c;
                    last = c;
                }
            }
            if (samples > 0 && support * 100 >= samples * 45 && last - first >= roi.width / 2)
                return true; // 至少45%横向样本、跨度至少半个ROI；需按实际横杆图像标定
        }
    }
    return false;
}

void observeGate(const Mat &img, const vector<PredictResult> &results, GateGeometry &geometry, bool stopped)
{
    geometry.imageValid = false;
    geometry.barVisible = false;
    if (img.empty() || img.type() != CV_8UC1 || img.cols != COLSIMAGE || img.rows != ROWSIMAGE)
    {
        geometry.closedFrames = 0;
        return;
    }
    bool foreground = false, background = false;
    for (int r = 0; r < img.rows && !(foreground && background); r += 8)
    {
        const uchar *row = img.ptr<uchar>(r);
        for (int c = 0; c < img.cols; c += 8)
        {
            foreground = foreground || row[c] == 255;
            background = background || row[c] == 0;
        }
    }
    if (!foreground || !background) // 全黑/全白图像不能作为抬杆证据
    {
        geometry.closedFrames = 0;
        return;
    }
    if (!geometry.closedSeen)
    {
        int bottom = -1;
        for (const auto &result : results)
        {
            if (result.type != LABEL_GATE || result.x < 0 || result.x >= img.cols ||
                result.y < 0 || result.y >= img.rows || result.width <= 0 || result.width > img.cols ||
                result.height <= 0 || result.height > img.rows)
                continue;
            int left = result.x + result.width / 8;
            int right = std::min(img.cols, result.x + result.width * 7 / 8);
            int top = std::max(0, result.y - result.height / 8);
            int end = std::min(img.rows, result.y + result.height + result.height / 8);
            if (right - left >= COLSIMAGE / 8 && end - top >= 12 && end > bottom)
            {
                geometry.roi = cv::Rect(left, top, right - left, end - top);
                bottom = end;
            }
        }
    }
    const cv::Rect &roi = geometry.roi;
    if (roi.width < COLSIMAGE / 8 || roi.height < 12 || roi.x < 0 || roi.y < 0 ||
        roi.x + roi.width > img.cols || roi.y + roi.height > img.rows)
        return;
    geometry.imageValid = true;
    geometry.barVisible = hasHorizontalGateBar(img, roi);
    if (stopped)
    {
        if (geometry.barVisible)
        {
            if (geometry.closedFrames < 3)
                geometry.closedFrames++;
            if (geometry.closedFrames >= 3)
                geometry.closedSeen = true;
        }
        else
            geometry.closedFrames = 0;
    }
}
}


/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmStop::FsmStop(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STOP, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmStop::~FsmStop()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmStop::getMode()
{
    // 输出场景状态结果
    if (step == Step::NONE || !params->config.stop)
        return FsmMode::NORMAL;
    else
        return FsmMode::STOP;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmStop::run(Mat &img)
{
    if (!params->config.stop) // 该模式未启用
        return;

    polyRoad.clear();
    polyCar.clear();
    if (step == Step::ENABLE || step == Step::STOP)
        observeGate(img, params->results, params->gateGeometry, step == Step::STOP);

    switch (step)
    {
    case Step::NONE: // AI未识别
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE) // 障碍物：道闸
            {
                countRec++;
                break;
            }
        }

        if (countRec > 2)
            setStep(Step::ENABLE); // 设置新状态

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes > 4)
                setStep(Step::NONE);
        }
        break;
    }

    case Step::ENABLE: // 场景使能
    {
        countSes++; // 场次计数器
        timeout++;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE) // 障碍物：道闸
            {
                countSes = 0;
                timeout = 0;
                if ((params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.3) // 停车距离计算
                {
                    countRec++;
                    break;
                }
                break;
            }
        }
        if (countRec > 2)
            setStep(Step::STOP); // 设置新状态
        if (countSes >= 10 || timeout > 50)
            setStep(Step::NONE); // 设置新状态
        break;
    }

    case Step::STOP: // 停车
    {
        params->ctrl.stop = true; // 停车标志
        countSes++;               // 场次计数器
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE) // 障碍物：道闸
            {
                countSes = 0;
                break;
            }
        }
        // 标签消失且参考ROI中横杆消失才累计；标签仍在/横杆仍在/图像无效均清零。
        if (!params->gateGeometry.closedSeen || !params->gateGeometry.imageValid || params->gateGeometry.barVisible)
            countSes = 0;
        if (countSes >= 20)
        {
            setStep(Step::NONE); // 设置新状态
            params->ctrl.stop = false;
        }

        break;
    }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmStop::show(Mat &img)
{
    if (step == Step::NONE || !params->config.stop)
        return;

    putText(img, "[7] Stop", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::ENABLE: // 场景使能
        putText(img, "[7] STOP - ENABLE", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;

    case Step::STOP: // 停车
        putText(img, "[7] STOP - STOPING", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }

    if (params->gateGeometry.roi.width > 0)
        rectangle(img, params->gateGeometry.roi, cv::Scalar(255, 255, 0), 1);
    putText(img, "Gate V:" + to_string(params->gateGeometry.imageValid) +
                 " B:" + to_string(params->gateGeometry.barVisible) +
                 " Seen:" + to_string(params->gateGeometry.closedSeen) +
                 " Clear:" + to_string(countSes) + "/20", Point(8, 100),
            cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);

    if (polyRoad.size() > 0 && polyCar.size() > 0)
    {
        drawPolygon(img, polyRoad, false, true); // 绘制赛道多边形
        drawPolygon(img, polyCar, false, true);  // 绘制车辆多边形

        putText(img, "Overlap:" + doble2String(overlap, 2),
                Point(100, ROWSIMAGE - 60), FONT_HERSHEY_TRIPLEX, 0.5,
                Scalar(0, 0, 255), 0.5);
    }
}

/**
 * @brief 计算多边形重合度
 *
 * @param track
 * @param result
 */
double FsmStop::getRoadCarPloy(PredictResult result)
{
    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 5 && params->track->pointsEdgeRight.size() < ROWSIMAGE / 5)
        return 0.0;

    // 添加赛道多边形轮廓
    PointX ppoliy;
    if (params->track->pointsEdgeLeft.size() < 5 && params->track->pointsEdgeRight.size() > 5) // 右单边
    {
        // [1]左上点
        ppoliy.x = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x;
        ppoliy.y = 1;
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        // [2]左下点
        ppoliy.x = params->track->pointsEdgeRight[0].x;
        ppoliy.y = 1;
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 2
    }
    else
    {
        // [1]左上点
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.8];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.6];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.4];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.2];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        // [2]左下点
        ppoliy = params->track->pointsEdgeLeft[0];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 2
    }

    if (params->track->pointsEdgeLeft.size() > 5 && params->track->pointsEdgeRight.size() < 5) // 左单边
    {
        //[3] 右下点
        ppoliy.x = params->track->pointsEdgeLeft[0].x;
        ppoliy.y = COLSIMAGE - 1;

        //[4] 右上点
        ppoliy.x = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x;
        ppoliy.y = COLSIMAGE - 1;
    }
    else
    {
        //[3] 右下点
        ppoliy = params->track->pointsEdgeRight[0];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.2];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.4];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.6];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.8];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        //[4] 右上点
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
    }

    // 添加车辆多边形轮廓
    polyCar.push_back(cv::Point(result.x, result.y));
    polyCar.push_back(cv::Point(result.x, result.y + result.height));
    polyCar.push_back(cv::Point(result.x + result.width, result.y + result.height));
    polyCar.push_back(cv::Point(result.x + result.width, result.y));

    return getOverlapArea(polyRoad, polyCar); // 计算多边形重合度
}

/**
 * @brief 计算两个多边形图形的面积重叠度
 *
 * @param polyA 多边形顶点
 * @param polyB
 * @return double [0,1]
 */
double FsmStop::getOverlapArea(const std::vector<cv::Point> &polyA, const std::vector<cv::Point> &polyB)
{
    // 创建两个多边形的掩模
    cv::Mat maskA = cv::Mat::zeros(500, 500, CV_8UC1);
    cv::Mat maskB = cv::Mat::zeros(500, 500, CV_8UC1); // 智能车

    // 填充多边形
    cv::fillConvexPoly(maskA, polyA, cv::Scalar(255));
    cv::fillConvexPoly(maskB, polyB, cv::Scalar(255));

    // 计算交集
    cv::Mat intersection;
    cv::bitwise_and(maskA, maskB, intersection);

    // 计算交集的面积
    double areaB = cv::countNonZero(maskB);
    double areaAnd = cv::countNonZero(intersection);

    // 计算重合度
    double overlap = (areaB > 0) ? (areaAnd / areaB) : 0;

    return overlap;
}

/**
 * @brief 绘制多边形
 *
 * @param img
 * @param poly 多边形点集
 * @param fill  填充
 * @param close 封闭图形
 */
void FsmStop::drawPolygon(Mat img, vector<Point> poly, bool fill, bool close)
{
    vector<vector<Point>> contours;
    contours.push_back(poly);

    if (fill)
        fillPoly(img, contours, Scalar(255, 255, 255), 8);
    else
        polylines(img, poly, close, Scalar(255, 255, 255), 2, 8);
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmStop::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
    if (st == Step::NONE)
        params->gateGeometry = GateGeometry{};
    if (st == Step::STOP)
        params->ctrl.stop = true; // 切换到停车的当帧立即停止
}