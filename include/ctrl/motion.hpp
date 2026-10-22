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
            dFiltered = 0.0;
            yforkControllerActive = false;
            return;
        }
        float error = params->ctrl.center - COLSIMAGE / 2; // 图像控制中心转换偏差
        if ((params->mode == FsmMode::YFORK || params->ctrl.yforkPath) && !params->ctrl.back)
        {
            const double dt = std::clamp(params->ctrl.frameDt, 1.0 / 120.0, 0.2);
            const double frames = dt * 30.0;
            if (!yforkControllerActive)
            {
                yforkErrorLast = error;
                yforkDerivative = 0.0;
            }
            yforkControllerActive = true;
            const double alpha = 1.0 - std::pow(1.0 - params->config.yforkDerivativeAlpha, frames);
            const double derivative = std::clamp((error - yforkErrorLast) / frames, -40.0, 40.0);
            yforkDerivative += alpha * (derivative - yforkDerivative);
            yforkErrorLast = error;
            errorLast = static_cast<int>(error);
            modeLast = params->mode;
            backLast = params->ctrl.back;
            if (params->ctrl.yforkStop)
            {
                yforkDerivative = 0.0;
                return;
            }
            const double gain = params->config.yforkRunP1 + std::abs(error) * params->config.yforkRunP2;
            double target = PWMSERVOMID + error * gain + yforkDerivative * params->config.yforkTurnD;
            if (!std::isfinite(target))
            {
                params->ctrl.yforkStop = true;
                return;
            }
            const double slew = params->config.yforkServoSlew * frames;
            target = std::clamp(target, double(params->ctrl.servo) - slew, double(params->ctrl.servo) + slew);
            params->ctrl.servo = static_cast<uint16_t>(std::lround(std::clamp(target,
                double(PWMSERVOMIN), double(PWMSERVOMAX))));
            return;
        }
        if (yforkControllerActive) // 刚从 Y 岔路专用控制器切回普通控制器，清掉 D 的历史防抽动
            dFiltered = 0.0;
        yforkControllerActive = false;
        if (params->mode != modeLast || params->ctrl.back != backLast)
        {
            errorLast = static_cast<int>(error);
            dFiltered = 0.0; // 换模式瞬间清 D 历史，避免模式切换时舵机抽一下
            modeLast = params->mode;
            backLast = params->ctrl.back;
        }
        if (abs(error - errorLast) > COLSIMAGE / 10)  // 普通跟线：限速防猛打方向
        {
            error = error > errorLast ? errorLast + COLSIMAGE / 10
                                      : errorLast - COLSIMAGE / 10;
        }

        const float turnP = abs(error) * params->config.runP2 + params->config.runP1;
        // D 项一阶低通：turnDFilter=0 时 dFiltered 直接等于本帧差分，与旧代码完全等价
        const double derivative = double(error - errorLast);
        const double dAlpha = std::clamp(double(params->config.turnDFilter), 0.0, 1.0);
        dFiltered = dAlpha <= 0.0 ? derivative : dFiltered + dAlpha * (derivative - dFiltered);
        // 关于"D 要不要随速度放大"：2025 工程在 1.6 m/s 用的是 D=2.0 定值，
        // GPT 在 1.0~1.2 m/s 用的也是 D=2.0 定值。两个都能跑的配置都用了定值，
        // 所以这里不做速度缩放，保持和 2025 一致的定值 D，只加低通滤掉高频噪声。
        double pwmDiff = (error * turnP) + dFiltered * params->config.turnD;
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
        if (params->ctrl.stop || params->ctrl.yforkStop || params->quit) // 停车
        {
            params->ctrl.speed = 0.0;
            speedCmd = 0.0; // 车已停住，指令也从 0 重新爬升
            return;
        }
        if (params->ctrl.crossStop) // 斑马线停车专用：优先于任何状态速度
        {
            params->ctrl.speed = 0.0;
            speedCmd = 0.0;
            return;
        }

        if (params->ctrl.back) // 倒车(停车场)
        {
            params->ctrl.speed = -(params->ctrl.slow ? std::min(params->config.velPark, params->config.velSlow)
                                                   : params->config.velPark);
            speedCmd = 0.0; // 出库前进时从 0 平滑爬升
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
            const double steering = std::abs(double(params->ctrl.servo) - PWMSERVOMID);
            params->ctrl.speed = params->config.velYFork * std::clamp(1.0 - steering / 650.0, 0.55, 1.0);
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
        const bool yforkSpeedControl = params->mode == FsmMode::YFORK || params->ctrl.yforkPath;
        if (yforkSpeedControl && std::isfinite(params->ctrl.yforkSpeedCap) &&
            params->ctrl.yforkSpeedCap > 0.0)
            params->ctrl.speed = std::min(params->ctrl.speed, float(params->ctrl.yforkSpeedCap));

        // Y 岔路专用速度斜率限制，普通巡线和丁字口保留原速度响应。
        const double slewPerFrame = double(params->config.speedSlew);
        if (yforkSpeedControl && slewPerFrame > 0.0)
        {
            const double frames = std::clamp(params->ctrl.frameDt, 1.0 / 120.0, 0.2) * 30.0;
            const double upStep = slewPerFrame * frames;
            const double downStep = upStep;
            const double target = double(params->ctrl.speed);
            speedCmd = target > speedCmd ? std::min(target, speedCmd + upStep)
                                         : std::max(target, speedCmd - downStep);
            params->ctrl.speed = static_cast<float>(speedCmd);
        }
        else
            speedCmd = double(params->ctrl.speed);
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
    bool yforkControllerActive = false;
    double yforkErrorLast = 0.0;
    double yforkDerivative = 0.0;
    double dFiltered = 0.0;  // 普通巡线 D 项的一阶低通状态
    double speedCmd = 0.0;   // 速度指令斜率限制的状态（m/s）
    int errorLast = 0;
    FsmMode modeLast = FsmMode::NORMAL;
    bool backLast = false;
    int countRes = 0;
    int countOut = 0;
    bool outline = false; // 出线标志
};
