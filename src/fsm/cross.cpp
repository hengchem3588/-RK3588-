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
 * @file cross.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 斑马线停车控制
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/cross.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmCross::FsmCross(std::shared_ptr<Params> par)
    : FSMState(FsmMode::CROSS, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmCross::~FsmCross()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmCross::getMode()
{
    // 输出场景状态结果
    if (step == Step::NONE || !params->config.cross)
        return FsmMode::NORMAL;
    else
        return FsmMode::CROSS;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmCross::run(Mat &img)
{
    if (!params->config.cross) // 该模式未启用
        return;

    countInit++; // 起点屏蔽计数器
    if (countInit > 999)
        countInit = 999;
    else if (countInit < 60)
        return;

    // 用持续前行指令和标志消失作为离开代理；短暂或原地丢框不解除计圈屏蔽。
    if (step == Step::NONE && !lapArmed)
    {
        bool startFinishSeen = false;
        for (const auto &result : params->results)
        {
            if (result.type == LABEL_CROSS || result.type == LABEL_STOP)
            {
                startFinishSeen = true;
                break;
            }
        }
        if (startFinishSeen || params->ctrl.stop || params->ctrl.crossStop || params->ctrl.back ||
            !(params->ctrl.speed > 0.0f)) // 无实测里程；至少要求前行指令连续有效
            countLapClear = 0;
        else if (countLapClear < 60)
            countLapClear++;
        if (countLapClear >= 60) // 沿用原屏蔽尺度：30fps 下连续 2 秒无标志
            lapArmed = true;
    }

    switch (step)
    {
    case Step::NONE: // AI未识别
    {
        countCross++; // 斑马线屏蔽计数器
        if (countCross > 999)
            countCross = 999;

        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CROSS && countCross > 60 && lapArmed) // 禁行标志：斑马线
            {
                countRec++;
                break;
            }
        }

        if (countRec >= 2)
            setStep(Step::ENABLE); // 设置新状态

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes > 4)
            {
                countRec = 0; // AI场景识别计数器
                countSes = 0; // 场次计数器
            }
        }
        break;
    }

    case Step::ENABLE: // 场景使能
    {
        // 停标检测(识别到停标即进入等待期,后续不再被FLOW打断/timeout兜底)
        bool stopNow = false;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STOP &&
                params->results[i].height < 120 && params->results[i].width < 90)
            {
                stopNow = true;
                break;
            }
        }
        if (stopNow)
            countStop++;
        else
            countStop = 0;
        if (!stopSeen && countStop >= 2)
        {
            stopSeen = true; // 识别到停标,进入等待期
            timeout = 0;     // 清零兜底计数,等待期不被timeout截断
        }

        if (stopSeen) // 已识别停标:等待30帧后再进STOP停车
        {
            countRec++;
            if (countRec >= 50)
            {
                setStep(Step::STOP); // 30帧等待结束才停
                return;
            }
        }
        else // 未识别停标:仍可被通行标志放行
        {
            countRec = 0;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_FLOW &&
                    params->results[i].height < 120 && params->results[i].width < 90 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.1)
                {
                    setStep(Step::NONE); // 放行，不进入停车
                    return;
                }
            }

            timeout++;
            if (timeout > 30) // 未识别到停标超时兜底
            {
                setStep(Step::NONE); // 设置新状态
            }
        }
        break;
    }

    case Step::STOP: // 停车
    {
        params->ctrl.crossStop = true; // 停车标志

        timeout++; // 场次计数器
        if (timeout >= 30) // 每圈先停留 30 帧；只有第二圈请求退出
        {
            if (!lapCounted)
            {
                params->lapCount++;
                lapCounted = true;
                lapArmed = false;
                countLapClear = 0;
            }
            if (params->lapCount >= 2)
            {
                params->finishStop = true;
                params->quit = true; // 置退出标志，主循环统一执行舵机归中后退出
            }
            else
                setStep(Step::NONE); // 第一圈只解除 crossStop，不清除其他 FSM 的 stop
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
void FsmCross::show(Mat &img)
{
    putText(img, "Lap:" + to_string(params->lapCount) + "/2 Armed:" + to_string(lapArmed),
            Point(8, ROWSIMAGE - 8), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);
    if (params->mode != FsmMode::CROSS)
        return;

    putText(img, "[8] Cross", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::ENABLE: // 场景使能
        putText(img, "[8] Cross - ENABLE", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;

    case Step::STOP: // 停车
        putText(img, "[8] Cross - STOPING", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmCross::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
    // params->ctrl.stop = false; // 保留原代码：stop 属于道闸/停车场，Cross 仅管理 crossStop
    params->ctrl.crossStop = false;
    if (st == Step::ENABLE)
        lapCounted = false;
    countCross = 0;
    countStop = 0;
    stopSeen = false;
}