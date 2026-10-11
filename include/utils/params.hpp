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
 * @file state.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 机器人状态信息
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <string>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include "utils/json.hpp"
#include "ctrl/track.hpp"

using namespace std;

/**
 * @brief FSM状态场景
 *
 */
enum class FsmMode
{
    NORMAL, // 基础赛道
    FORK,   // 岔路
    PARK,   // 停车场
    BUSY,   // 施工障碍
    SLOW,   // 慢行区
    FINE,   // 禁行区
    STOP,   // 停车区
    CROSS,  // 斑马线
    YFORK,  // 岔路口
    OBSTACLE, // 避障（锥桶/行人）
};

/**
 * @brief 车辆控制指令
 *
 */
struct Control
{
    bool stop = false;            // 车辆停止运动
    bool crossStop = false;       // 斑马线停车专用
    bool back = false;            // 倒车
    bool slow = false;            // 车辆减速
    uint16_t servo = PWMSERVOMID; // 发送给舵机的PWM
    float speed = 0.0;            // 发送给电机的速度
    int center = COLSIMAGE / 2;   // 控制中心
    vector<PointX> centerEdge;    // 赛道中心点集
    int lineArea = 0;             // 最远有效行号，失败为0；按已确认语义，行号越大速度越大
    double steerFF = 0.0;
    double speedLimit = 0.0;
    bool fitting = false;         // 控制中心拟合标志(停车场专用)
    bool parking = false;         // 停车场特殊模式
    int countAcc = 0;             // 起步低速保持帧数，不是速度爬升计算
};
/**
 * @brief 控制器核心参数
 *
 */
struct Config
{
    float velLow = 1.0;                                 // 智能车最低速:m/s
    float velHigh = 1.0;                                // 智能车最高速:m/s
    float velSlow = 1.0;                                // 慢性区速度:m/s
    float velPark = 1.0;                                // 充电站车速
    float velBusy = 1.0;                                // 施工区速度:m/s
    float velStop = 1.0;                                // 停车区速度: m/s
    float velCross = 1.0;                               // 斑马线速度: m/s
    float velYFork = 1.0;                               // 岔路口速度: m/s
    float runP1 = 1.5;                                  // 比例系数：直线控制量
    float runP2 = 0.012;                                // 动态P变化系数
    float turnP = 3.5;                                  // 比例系数：转弯控制量
    float turnD = 3.5;                                  // 微分系数：转弯控制量
    bool debug = false;                                 // 调试模式使能
    bool saveImg = false;                               // 存图使能
    bool saveIpm = false;                               // 存储IPM图像
    uint16_t rowCutUp = 10;                             // 图像顶部切行
    uint16_t rowCutBottom = 10;                         // 图像底部切行
    bool fork = true;                                   // 岔路使能
    bool fine = true;                                   // 禁行区使能
    bool park = true;                                   // 停车场使能
    bool spot = true;                                   // 停车位使能
    int parkSpot = 1;                                   // 目标停车位（1-4号车库，可配置）
    bool busy = true;                                   // 施工区使能
    bool slow = true;                                   // 慢行区使能
    bool stop = true;                                   // 停车区使能
    bool yfork = true;                                  // 岔路口使能
    bool cross = true;                                  // 斑马线停车使能
    bool obstacle = true;                               // 避障使能
    float overlap = 0.3;                                // 智能车与车道线重合度(%)
    float score = 0.5;                                  // AI检测置信度
    int binary = -1;                                    // 图像二值化阈值
    string model = "../res/models/yolov3_mobilenet_v1"; // 模型路径
    string video = "../res/samples/fine.mp4";           // 视频路径
    int aiStartupTimeoutMs = 5000;
    int aiResultMaxAgeMs = 1000;
    bool curvatureEnabled = false;
    double curvaturePreviewPx = 45.0;
    double curvatureFfGain = 0.0;
    double curvatureFfLimit = 120.0;
    double curvatureMinPreviewRow = 20.0;
    double curvatureALatMax = 2.2;
    double curvaturePxPerMeter = 0.0;
    double curvaturePxPerMeterLateral = 0.0;
    double curvatureWheelbase = 0.0;
    double curvaturePwmPerRadian = 0.0;
    double curvatureRearAxleOffset = 0.0;
    double curvatureSpeedFloor = 0.40;
    double curvatureDecelRate = 0.35;
    double curvatureAccelRate = 0.08;
    int curvatureOriginRow = ROWSIMAGE - 1;
    int curvatureOriginCol = COLSIMAGE / 2;
    int parkEnterMaxFrames = 60; // 入库看门狗，30fps下2秒；超时停车，不作为入位成功
    int parkCenterTolerance = 8; // 近/远两行中点允许偏差(px)，需相机姿态实测
    int parkStableFrames = 5;    // 连续稳定观测下限，30fps下约0.17秒
    int slowMinFrames = 111; // 原 timeout>110 的退出尺度：111/30=3.7秒，区间长度仍需实测
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, velLow, velHigh, velSlow, velPark, velYFork, velBusy, velStop, velCross,
                                   runP1, runP2, turnP, turnD, debug, saveImg, saveIpm, rowCutUp, rowCutBottom,
                                   fork, fine, park, spot, parkSpot, busy, slow, stop, yfork, cross, obstacle,
                                   overlap, score, binary, model, video); // 添加构造函数
};

// 道闸几何观测仅由 FsmStop 使用，不与其他 FSM 交换判定结果。
struct GateGeometry
{
    cv::Rect roi{};              // 停车时横杆参考区域，闭杆确认后冻结
    bool imageValid = false;    // 当前二值图和 ROI 可用
    bool barVisible = false;    // 当前区域存在水平对比条带
    bool closedSeen = false;    // 本次停车曾连续确认闭杆
    int closedFrames = 0;       // 闭杆连续观测帧数
};

/**
 * @brief 车辆状态参数（FSM共享传递）
 *
 */
struct Params
{
public:
    /**
     * @brief Construct a new Params object
     *
     */
    Params()
    {
        // 加载本地json配置文件
        string path = "../res/config.json";
        std::ifstream fileStr(path);
        if (!fileStr.good())
        {
            std::cout << "Error: Params file path:[" << path << "] not find !!!" << std::endl;
            exit(-1);
        }
        nlohmann::json configs;
        try
        {
            fileStr >> configs;
            config = configs.get<Config>();
            const auto optional = [&configs](const char *name, auto &value)
            {
                if (configs.contains(name))
                    configs.at(name).get_to(value);
            };
            optional("curvatureEnabled", config.curvatureEnabled);
            optional("curvaturePreviewPx", config.curvaturePreviewPx);
            optional("curvatureFfGain", config.curvatureFfGain);
            optional("curvatureFfLimit", config.curvatureFfLimit);
            optional("curvatureMinPreviewRow", config.curvatureMinPreviewRow);
            optional("curvatureALatMax", config.curvatureALatMax);
            optional("curvaturePxPerMeter", config.curvaturePxPerMeter);
            optional("curvaturePxPerMeterLateral", config.curvaturePxPerMeterLateral);
            optional("curvatureWheelbase", config.curvatureWheelbase);
            optional("curvaturePwmPerRadian", config.curvaturePwmPerRadian);
            optional("curvatureRearAxleOffset", config.curvatureRearAxleOffset);
            optional("curvatureSpeedFloor", config.curvatureSpeedFloor);
            optional("curvatureDecelRate", config.curvatureDecelRate);
            optional("curvatureAccelRate", config.curvatureAccelRate);
            optional("curvatureOriginRow", config.curvatureOriginRow);
            optional("curvatureOriginCol", config.curvatureOriginCol);
            const double geometryValues[] = {config.curvaturePreviewPx, config.curvatureFfGain,
                config.curvatureFfLimit, config.curvatureMinPreviewRow, config.curvatureALatMax,
                config.curvaturePxPerMeter, config.curvaturePxPerMeterLateral, config.curvatureWheelbase,
                config.curvaturePwmPerRadian, config.curvatureRearAxleOffset, config.curvatureSpeedFloor,
                config.curvatureDecelRate, config.curvatureAccelRate};
            for (double value : geometryValues)
                if (!std::isfinite(value))
                    throw std::runtime_error("Curvature configuration must be finite");
            if (config.curvaturePreviewPx <= 0.0 || config.curvaturePreviewPx >= ROWSIMAGE ||
                config.curvatureMinPreviewRow <= 0.0 || config.curvatureMinPreviewRow > config.curvaturePreviewPx ||
                config.curvatureFfGain < 0.0 || config.curvatureFfLimit < 0.0 || config.curvatureFfLimit > 400.0 ||
                config.curvatureALatMax <= 0.0 || config.curvaturePxPerMeter < 0.0 ||
                config.curvaturePxPerMeterLateral < 0.0 || config.curvatureWheelbase < 0.0 ||
                config.curvatureRearAxleOffset < 0.0 || config.curvatureSpeedFloor < 0.0 ||
                config.curvatureDecelRate <= 0.0 || config.curvatureDecelRate > 1.0 ||
                config.curvatureAccelRate <= 0.0 || config.curvatureAccelRate > 1.0 ||
                config.curvatureOriginRow < 0 || config.curvatureOriginRow >= ROWSIMAGE ||
                config.curvatureOriginCol < 0 || config.curvatureOriginCol >= COLSIMAGE)
                throw std::runtime_error("Invalid curvature parameter bounds");
            if (config.curvatureEnabled &&
                (config.curvaturePxPerMeter <= 0.0 || config.curvaturePxPerMeterLateral <= 0.0 ||
                 (config.curvatureFfGain > 0.0 && (config.curvatureWheelbase <= 0.0 || config.curvaturePwmPerRadian == 0.0))))
                throw std::runtime_error("Curvature requires measured IPM scales; feedforward also requires wheelbase and PWM calibration");
            if (configs.contains("aiStartupTimeoutMs"))
                configs.at("aiStartupTimeoutMs").get_to(config.aiStartupTimeoutMs);
            if (configs.contains("aiResultMaxAgeMs"))
                configs.at("aiResultMaxAgeMs").get_to(config.aiResultMaxAgeMs);
            if (config.aiStartupTimeoutMs < 1 || config.aiStartupTimeoutMs > 60000 ||
                config.aiResultMaxAgeMs < 1 || config.aiResultMaxAgeMs > 60000)
                throw std::runtime_error("AI timeout must be in [1,60000] ms");
            const float speeds[] = {config.velLow, config.velHigh, config.velSlow, config.velPark,
                                    config.velBusy, config.velStop, config.velCross, config.velYFork};
            for (float speed : speeds)
                if (!std::isfinite(speed) || speed < 0.0f)
                    throw std::runtime_error("Speed must be finite and nonnegative");
            if (config.velLow > config.velHigh || (config.slow && config.velSlow >= 0.5f))
                throw std::runtime_error("Require velLow<=velHigh and velSlow<0.5 when slow is enabled");
            if (!std::isfinite(config.score) || config.score < 0.0f || config.score > 1.0f ||
                !std::isfinite(config.overlap) || config.overlap < 0.0f || config.overlap > 1.0f)
                throw std::runtime_error("Detection score and overlap must be finite values in [0,1]");
            if (!std::isfinite(config.runP1) || !std::isfinite(config.runP2) ||
                !std::isfinite(config.turnP) || !std::isfinite(config.turnD) ||
                config.runP1 < 0.0f || config.runP2 < 0.0f || config.turnP < 0.0f || config.turnD < 0.0f)
                throw std::runtime_error("PD gains must be finite and nonnegative");
            if (configs.contains("parkEnterMaxFrames"))
                configs.at("parkEnterMaxFrames").get_to(config.parkEnterMaxFrames);
            if (configs.contains("parkCenterTolerance"))
                configs.at("parkCenterTolerance").get_to(config.parkCenterTolerance);
            if (configs.contains("parkStableFrames"))
                configs.at("parkStableFrames").get_to(config.parkStableFrames);
            if (config.parkEnterMaxFrames < 21 || config.parkEnterMaxFrames > 60000 ||
                config.parkCenterTolerance < 1 || config.parkCenterTolerance > COLSIMAGE / 4 ||
                config.parkStableFrames < 1 || config.parkStableFrames > 60000)
            {
                std::cerr << "Invalid parking frame/tolerance configuration" << std::endl;
                exit(-1);
            }
            if (configs.contains("slowMinFrames"))
                configs.at("slowMinFrames").get_to(config.slowMinFrames);
            if (config.slowMinFrames < 1 || config.slowMinFrames > 60000)
            {
                std::cerr << "slowMinFrames must be in [1,60000]" << std::endl;
                exit(-1);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << std::endl;
            exit(-1);
        }

        mode = FsmMode::NORMAL;                    // 初始化控制模式
        modeLast = FsmMode::NORMAL;                // 初始化控制模式
        track = make_shared<Track>();              // 赛道线处理
        track->rowCutUp = config.rowCutUp;         // 图像顶部切行（前瞻距离）
        track->rowCutBottom = config.rowCutBottom; // 图像底部切行（盲区距离）
    };
    ~Params() {};

    Control ctrl;                       // 车辆控制指令(实时)
    Config config;                      // 系统配置
    FsmMode mode, modeLast;             // FSM状态场景
    shared_ptr<Track> track;            // 赛道识别类
    std::vector<PredictResult> results; // AI推理结果
    GateGeometry gateGeometry{};      // 道闸当次独立观测状态
    int slowActiveFrames = 0;          // 本次慢行累计帧数，LIMIT 检测不清零
    int lapCount = 0;                  // 已完成圈数：仅由起终点停车事件累计
    bool finishStop = false;            // 第二圈终点停车完成标志
    bool quit = false;                  // 停标触发退出进程标志(主循环检测后先舵机归中再退出)
private:
};
