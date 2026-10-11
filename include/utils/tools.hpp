#pragma once
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
 * @file tools.hpp
 * @author Leo
 * @brief 通用方法类
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ctrl/mapping.hpp"
#include "utils/json.hpp"

using namespace std;
using namespace cv;

#define COLSCAMERA 320   // 相机的列数
#define ROWSCAMERA 240   // 相机的行数
#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
#define COLSIMAGEIPM 800 // IPM图像的列数
#define ROWSIMAGEIPM 500 // IPM图像的行数
#define PWMSERVOMAX 1900 // 舵机PWM最大值（左）1840
#define PWMSERVOMID 1500 // 舵机PWM中值 1520
#define PWMSERVOMIN 1100 // 舵机PWM最小值（右）1200

#define LABEL_CONE 0    // AI标签: 锥桶
#define LABEL_PERSON 1  // AI标签: 行人
#define LABEL_BUSY 2    // AI标签: 施工区标
#define LABEL_LIMIT 3   // AI标签: 限速标志
#define LABEL_UNLIMIT 4 // AI标签: 解除限速标志
#define LABEL_STOP 5    // AI标签: 禁止通行标志
#define LABEL_FLOW 6    // AI标签: 畅通标志
#define LABEL_PARK 7    // AI标签: 停车场
#define LABEL_GATE 8    // AI标签: ETC阻拦杆
#define LABEL_CROSS 9   // AI标签: 斑马线
#define LABEL_FORK 10   // AI标签: 岔路标志
#define LABEL_LEFT 11   // AI标签: 左转标志
#define LABEL_CHOICE 12 // AI标签: 双向箭头标志
#define LABEL_YLEFT 13  // AI标签: 左岔路口标志
#define LABEL_YRIGHT 14 // AI标签: 右岔路口标志

inline Mapping ipm = Mapping(Size(COLSIMAGE, ROWSIMAGE),
                             Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // 逆透视变换类

/**
 * @brief 目标检测结果
 *
 */
struct PredictResult
{
    int type = -1;     // ID
    std::string label; // 标签
    float score = 0.0f; // 置信度
    int x = 0;         // 坐标(左上角点)
    int y = 0;         // 坐标
    int width = 0;     // 尺寸
    int height = 0;    // 尺寸
};

/**
 * @brief 构建二维坐标
 *
 */
struct PointX
{
    int x = 0;
    int y = 0;
    float slope = 0.0f;

    PointX() {};
    PointX(int x, int y) : x(x), y(y) {};
    PointX(int x, int y, float z) : x(x), y(y), slope(z) {};
};

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
inline void savePicture(Mat &image)
{
    // 存图
    string name = ".jpg";
    static int counter = 0;
    counter++;
    string img_path = "../res/samples/train/";
    name = img_path + to_string(counter) + ".jpg";
    imwrite(name, image);
}

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
inline void savePicture(string path, Mat &image)
{
    // 存图
    string name = ".jpg";
    static int counterSave = 0;
    counterSave++;
    name = path + to_string(counterSave) + ".jpg";
    imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
inline double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
inline double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
inline double sigma(vector<PointX> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double sigma = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
inline int factorial(int x)
{
    if (x < 0)
        return 0;
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        if (f > std::numeric_limits<int>::max() / i)
            return std::numeric_limits<int>::max();
        f *= i;
    }
    return f;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<PointX>
 */
inline vector<PointX> Bezier(double dt, vector<PointX> input)
{
    vector<PointX> output;
    if (input.empty() || !std::isfinite(dt) || dt < 0.00001)
        return output;
    vector<cv::Point2d> work(input.size());

    double t = 0;
    while (t <= 1)
    {
        for (size_t index = 0; index < input.size(); ++index)
            work[index] = cv::Point2d(input[index].x, input[index].y);
        for (size_t remaining = work.size(); remaining > 1; --remaining)
        {
            for (size_t index = 0; index + 1 < remaining; ++index)
                work[index] = work[index] * (1.0 - t) + work[index + 1] * t;
        }
        output.emplace_back(static_cast<int>(work[0].x), static_cast<int>(work[0].y));
        t += dt;
    }
    return output;
}

/**
 * @brief 格式化double类型数据为字符串
 *
 * @param val 输入数据
 * @param fixed 保留小数位数
 * @return auto 输出字符串
 */
inline auto doble2String(double val, int fixed)
{
    auto str = std::to_string(val);
    const auto dot = str.find('.');
    if (!std::isfinite(val) || dot == std::string::npos)
        return str;
    fixed = std::clamp(fixed, 0, 6);
    return str.substr(0, dot + (fixed > 0 ? fixed + 1 : 0));
}
/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
inline double distanceForPoint2Line(PointX a, PointX b, PointX p)
{
    const double deltaX = static_cast<double>(b.x) - a.x;
    const double deltaY = static_cast<double>(b.y) - a.y;
    const double length = std::hypot(deltaX, deltaY);
    if (length < 1e-9)
        return std::hypot(static_cast<double>(p.x) - a.x, static_cast<double>(p.y) - a.y);
    return std::abs(deltaX * (p.y - a.y) - deltaY * (p.x - a.x)) / length;
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
inline double distanceForPoints(PointX a, PointX b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
inline double distanceForPoints(Point a, Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief 计算两点原是域上距离
 *
 * @param startPoint起点
 * @param endPoint终点
 *        KickEdgePoint 是否剔除边界点（包含边界点的距离默认999）
 * @return double
 */
inline double distanceForOrigin(PointX startPoint, PointX endPoint)
{
    Point2d a, b;
    a.x = startPoint.y;
    a.y = startPoint.x;
    b.x = endPoint.y;
    b.y = endPoint.x;
    Point2d c = ipm.homography(a);
    Point2d d = ipm.homography(b);
    double e = distanceForPoints(c, d);
    return e;
}

/**
 * @brief 自定义的比较函数，根据元素的x值进行比较
 * @inform
 * @return
 */
inline bool compareX(const PointX &a, const PointX &b)
{
    return a.x > b.x; // 降序排序
}

/**
 * @brief 图像帧数是否在范围内判断
 * @param 输入起始点和终止点
 * @inform 可以用于任何判断是否在范围内
 * @return
 */
inline bool posInRange(double pos, double a = 0, double b = ROWSIMAGE)
{
    if (pos < a || pos > b)
        return false;
    else
        return true;
}
/**
 * @brief 利用贝塞尔曲线的补线函数
 *
 * @param k 纵向倍率，默认为1
 * @param n 贝塞尔参数，越大补线间隔越大，点越稀疏
 * @param 输入起始点和终止点
 *
 * @return
 */
inline vector<PointX> simpleLine(PointX startPoint, PointX endPoint, double n = 0.1, double k = 1)
{
    PointX midPoint1((startPoint.x + endPoint.x) / 2 * k, (startPoint.y + endPoint.y) / 2);
    vector<PointX> input = {startPoint, midPoint1, endPoint};
    vector<PointX> b_modify = Bezier(n, input); // 贝塞尔曲线方法
    return b_modify;
}
/**
 * @brief 图像平滑算法
 * @param 输入点集
 *
 * @return
 */
inline vector<PointX> smoothLine(vector<PointX> Temp1, double n = 0.1, double k = 1)
{
    vector<PointX> AimPoints;
    if (Temp1.size())
    {
        AimPoints.push_back(Temp1[0]);
        for (int i = 1; i < Temp1.size(); i++)
        {
            if (abs(Temp1[i].y - AimPoints[AimPoints.size() - 1].y) < 5)
            {
                AimPoints.push_back(Temp1[i]);
            }
            else
            {
                vector<PointX> tp = simpleLine(AimPoints[AimPoints.size() - 1], Temp1[i], n, k);
                for (int i = 1; i < tp.size() - 1; i++)
                {
                    if (abs(tp[i].y - AimPoints[AimPoints.size() - 1].y) > 3)
                        AimPoints.push_back(tp[i]);
                }
            }
        }
    }
    return AimPoints;
}

/**
 * @brief 计算两点之间斜率
 * @param PointX a,b
 * @return double
 */
inline double gradientCal(PointX a, PointX b)
{
    double k = (double)(a.y - b.y) / (a.x - b.x); // 斜率
    return k;
}

/**
 * @brief 判断范围内边界点是否为一条直线
 *
 * @param  PointsEdge边界点集
 *         startLine起点
 *         endLine终点
 * @return bias起点终点拟合直线后偏移较大的点
 */
inline int linearCheck(vector<PointX> PointsEdge, int startLine = 0, int endLine = ROWSIMAGE - 1, int judgebias = 3)
{
    int bias = 0;
    if (PointsEdge.size() < 2)
        return bias;
    startLine = std::clamp(startLine, 0, static_cast<int>(PointsEdge.size()) - 1);
    endLine = std::clamp(endLine, startLine, static_cast<int>(PointsEdge.size()) - 1);
    double k = gradientCal(PointsEdge[startLine], PointsEdge[endLine]); // 斜率
    double b = PointsEdge[startLine].y - k * PointsEdge[startLine].x;
    for (int j = startLine; j < endLine; j++)
    {                                                                   // 从找到的圆环拐点开始遍历，直到上拐点对应的右侧直道斜率都差不多
        if (abs(k * PointsEdge[j].x + b - PointsEdge[j].y) > judgebias) // 斜率差值过大
            bias++;                                                     // 环外部分
    }
    return bias;
}

/**
 * @brief 矫正域上判断点的位置是否适合
 *
 * @param startPoint起点
 * @param endPoint终点
 * @param Min_Len 矫正域赛道允许最短长度
 * @param Max_Len 矫正域赛道允许最长长度
 * @return bool
 */
inline bool breakpointCheck(PointX startPoint, PointX endPoint, int Min_Len = 80, int Max_Len = 110)
{
    Point2f a, b;
    a.x = startPoint.y;
    a.y = startPoint.x;
    b.x = endPoint.y;
    b.y = endPoint.x;
    Point2f c = ipm.homography(a), d = ipm.homography(b);
    double e = distanceForPoints(c, d);
    if (e >= Min_Len && e <= Max_Len)
        return true;
    else
        return false;
}
