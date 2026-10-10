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
 * @file motion.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 运动控制器
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include "utils/tools.hpp"
#include "utils/params.hpp"

using namespace std;

class Motion
{

public:
    /**
     * @brief 姿态PD控制器
     *
     * @param center 智能车控制中心
     */
    void poseControl(shared_ptr<Params> &params)
    {
        float error = params->ctrl.center - COLSIMAGE / 2; // 图像控制中心转换偏差
        static int errorLast = 0;                          // 记录前一次的偏差
        if (params->mode == FsmMode::YFORK)                // yfork单边：立即跟随center，不被上一帧偏差限速拖反
            errorLast = (int)error;
        else if (abs(error - errorLast) > COLSIMAGE / 20)  // 普通跟线：限速防猛打方向
        {
            error = error > errorLast ? errorLast + COLSIMAGE / 10
                                      : errorLast - COLSIMAGE / 10;
        }

        params->config.turnP = abs(error) * params->config.runP2 + params->config.runP1;
        int pwmDiff = (error * params->config.turnP) + (error - errorLast) * params->config.turnD;
        errorLast = error;
        params->ctrl.servo = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
        if (params->ctrl.servo > PWMSERVOMAX)
            params->ctrl.servo = PWMSERVOMAX;
        else if (params->ctrl.servo < PWMSERVOMIN)
            params->ctrl.servo = PWMSERVOMIN;
    }

    /**
     * @brief 变加速控制
     *
     * @param params
     */
    void speedControl(shared_ptr<Params> &params)
    {
        if (params->ctrl.stop) // 停车
        {
            params->ctrl.speed = 0.0;
            return;
        }
        if (params->ctrl.crossStop) // 斑马线停车专用：优先于任何状态速度
        {
            params->ctrl.speed = 0.0;
            return;
        }

        if (params->ctrl.back) // 倒车(停车场)
        {
            params->ctrl.speed = -params->config.velPark;
            return;
        }
        if (params->ctrl.slow) // 减速区速度
        {
            params->ctrl.speed = params->config.velSlow;
            return;
        }
        if (params->mode == FsmMode::STOP) // 停车区速度
        {
            params->ctrl.speed = params->config.velStop;
            return;
        }
        else if (params->mode == FsmMode::PARK) // 停车场速度
        {
            params->ctrl.speed = params->config.velPark;
            return;
        }
        else if (params->mode == FsmMode::CROSS) // 斑马线速度
        {
            params->ctrl.speed = params->config.velCross;
            return;
        }
        else if (params->mode == FsmMode::BUSY) // 施工区速度
        {
            params->ctrl.speed = params->config.velBusy;
            return;
        }
        else if (params->mode == FsmMode::YFORK) // 岔路口速度
        {
            params->ctrl.speed = params->config.velYFork;
            return;
        }

        else if (params->ctrl.countAcc < 50)
        {
            params->ctrl.countAcc++;
            params->ctrl.speed = params->config.velCross;
            return;
        }

        int line = params->ctrl.lineArea; // 动态速度，返回点越高，速度越大，线性变化

        // 控制率
        uint8_t controlLow = 3;   // 速度控制下限
        uint8_t controlMid = 5;   // 控制率
        uint8_t controlHigh = 10; // 速度控制上限

        float upper = ROWSIMAGE * 0.35; // 84
        line = std::max(line - 20, 0);
        if (line > upper)
            line = upper;

        params->ctrl.speed = params->config.velLow +
                             std::pow(float(upper - line) / (upper), 3) * (params->config.velHigh - params->config.velLow);
        if (params->ctrl.speed > params->config.velHigh)
            params->ctrl.speed = params->config.velHigh;
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(shared_ptr<Params> &params, Mat &img)
    {
        std::string str = "Vel: " + doble2String(params->ctrl.speed, 2);
        putText(img, str, Point(COLSIMAGE - 100, 120), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 速度
    }

    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    void outlineCheck(shared_ptr<Params> &params)
    {
        if (outline) // 已出现
        {
            params->ctrl.stop = true; // 停车
            countRes++;
            if (countRes > 15)
            {
                std::cout << "-----> [Stop] Game over, system exit!!! <-----" << std::endl;
                std::exit(0); // 程序退出
            }
        }
        else // 出线检测
        {
            if (params->track->pointsEdgeLeft.size() < 30 &&
                params->track->pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
            {
                countRes++;
                countOut = 0;
                if (countRes > 10)
                    outline = true;
            }
            else
            {
                countOut++;
                if (countOut > 30)
                {
                    countRes = 0;
                    countOut = 30;
                }
            }
        }
    }

private:
    int countRes = 0;
    int countOut = 0;
    bool outline = false; // 出线标志
};
