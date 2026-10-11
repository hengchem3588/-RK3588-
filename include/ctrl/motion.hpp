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
        if (params->quit)
        {
            params->ctrl.servo = PWMSERVOMID;
            errorLast = 0;
            return;
        }
        float error = params->ctrl.center - COLSIMAGE / 2; // 图像控制中心转换偏差
        if (params->mode != modeLast || params->ctrl.back != backLast)
        {
            errorLast = static_cast<int>(error);
            modeLast = params->mode;
            backLast = params->ctrl.back;
        }
        if (params->mode == FsmMode::YFORK)                // yfork单边：立即跟随center，不被上一帧偏差限速拖反
            errorLast = (int)error;
        else if (abs(error - errorLast) > COLSIMAGE / 10)  // 普通跟线：限速防猛打方向
        {
            error = error > errorLast ? errorLast + COLSIMAGE / 10
                                      : errorLast - COLSIMAGE / 10;
        }

        const float turnP = std::min(params->config.turnP,
                                    abs(error) * params->config.runP2 + params->config.runP1);
        double pwmDiff = (error * turnP) + (error - errorLast) * params->config.turnD;
        if (std::isfinite(params->ctrl.steerFF) && !params->ctrl.back &&
            !params->ctrl.stop && !params->ctrl.crossStop)
            pwmDiff += params->ctrl.steerFF;
        errorLast = error;
        if (!std::isfinite(pwmDiff))
            pwmDiff = 0.0;
        params->ctrl.servo = static_cast<uint16_t>(std::clamp(PWMSERVOMID + pwmDiff,
                                                           double(PWMSERVOMIN), double(PWMSERVOMAX)));
    }

    /**
     * @brief 变加速控制
     *
     * @param params
     */
    void speedControl(shared_ptr<Params> &params)
    {
        if (params->ctrl.stop || params->quit) // 停车
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
            params->ctrl.speed = -(params->ctrl.slow ? std::min(params->config.velPark, params->config.velSlow)
                                                   : params->config.velPark);
            return;
        }
        if (params->mode == FsmMode::STOP) // 停车区速度
        {
            params->ctrl.speed = params->config.velStop;
        }
        else if (params->mode == FsmMode::PARK) // 停车场速度
        {
            params->ctrl.speed = params->config.velPark;
        }
        else if (params->mode == FsmMode::CROSS) // 斑马线速度
        {
            params->ctrl.speed = params->config.velCross;
        }
        else if (params->mode == FsmMode::BUSY) // 施工区速度
        {
            params->ctrl.speed = params->config.velBusy;
        }
        else if (params->mode == FsmMode::YFORK) // 岔路口速度
        {
            params->ctrl.speed = params->config.velYFork;
        }

        else if (params->ctrl.countAcc < 50)
        {
            params->ctrl.countAcc++;
            params->ctrl.speed = std::min(params->config.velCross, params->config.velLow);
        }
        else
        {
            const float upper = std::max(1, ROWSIMAGE - 1 - int(params->track->rowCutUp));
            const float line = std::clamp(float(params->ctrl.lineArea), 0.0f, upper);
            params->ctrl.speed = params->config.velLow +
                std::pow(line / upper, 3) * (params->config.velHigh - params->config.velLow);
        }
        if (params->ctrl.slow)
            params->ctrl.speed = std::min(params->ctrl.speed, params->config.velSlow);
        if (std::isfinite(params->ctrl.speedLimit) && params->ctrl.speedLimit > 0.0)
            params->ctrl.speed = std::min(params->ctrl.speed, float(params->ctrl.speedLimit));
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
    int errorLast = 0;
    FsmMode modeLast = FsmMode::NORMAL;
    bool backLast = false;
    int countRes = 0;
    int countOut = 0;
    bool outline = false; // 出线标志
};
