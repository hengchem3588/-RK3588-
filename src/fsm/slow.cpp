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
 * @file slow.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 减速区识别与规划
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/slow.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmSlow::FsmSlow(std::shared_ptr<Params> par)
    : FSMState(FsmMode::SLOW, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmSlow::~FsmSlow()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmSlow::getMode()
{
    // 输出场景状态结果
    if (!params->config.slow || step == Step::NONE)
        return FsmMode::NORMAL;

    return FsmMode::SLOW;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmSlow::run(Mat &img)
{
    if (!params->config.slow) // 该模式未启用
    {
        setStep(Step::NONE);
        return;
    }

    switch (step)
    {
    case Step::NONE: // AI标志检测

        params->ctrl.slow = false; // 车辆减速标志
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_LIMIT) // AI识别标志
            {
                if (params->results[i].height < 100 && params->results[i].width < 80) // 标志距离计算
                {
                    countRec++;
                    break;
                }
            }
        }
        if (countRec >= 2)
        {
            params->ctrl.slow = true; // 立马置减速标志：切换帧即生效，避免掉到动态速度
            setStep(Step::ENABLE);
        }

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes >= 5)
            {
                countRec = 0;
                countSes = 0;
            }
        }
        break;

    case Step::ENABLE: // 减速阶段
    {
        const int MIN_SLOW_FRAMES = params->config.slowMinFrames;
        if (params->slowActiveFrames < 60000)
            params->slowActiveFrames++; // 独立累计；只有 setStep 才复位
        if (timeout < 60000)
            timeout++;
        params->ctrl.slow = true; // 车辆减速标志
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_LIMIT) // AI识别标志
            {
                if (params->results[i].height < 100 && params->results[i].width < 80 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.2) // 标志距离计算
                {
                    timeout = 0;
                }
            }
            if (params->results[i].type == LABEL_UNLIMIT) // AI识别标志
            {
                if (params->results[i].height < 100 && params->results[i].width < 80 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.2) // 标志距离计算
                {
                    if (params->slowActiveFrames >= MIN_SLOW_FRAMES)
                        countRec++;
                    break;
                }
            }
        }
        if ((countRec >= 2 || timeout >= MIN_SLOW_FRAMES) && params->slowActiveFrames >= MIN_SLOW_FRAMES) // 最小持续量保护全部退出路径
            setStep(Step::NONE);

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes >= 5)
            {
                countRec = 0;
                countSes = 0;
            }
        }
        break;
    }

    default:
        break;
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmSlow::show(Mat &img)
{
    if (params->mode != FsmMode::SLOW)
        return;

    putText(img, "[6] Slow", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
    putText(img, "Slow:" + to_string(params->slowActiveFrames) + "/" + to_string(params->config.slowMinFrames) +
                 " Gap:" + to_string(timeout), Point(8, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmSlow::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
    params->slowActiveFrames = 0;
    params->ctrl.slow = (st == Step::ENABLE);
}
