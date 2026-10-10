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
    int lineArea = 0;             // 面积规划行序号
    bool fitting = false;         // 控制中心拟合标志(停车场专用)
    bool parking = false;         // 停车场特殊模式
    int countAcc = 500;           // 缓加速计数器
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
    string video = "../res/samples/sample.mp4";         // 视频路径
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, velLow, velHigh, velSlow, velPark, velYFork, velBusy, velStop, velCross,
                                   runP1, runP2, turnP, turnD, debug, saveImg, saveIpm, rowCutUp, rowCutBottom,
                                   fork, fine, park, spot, parkSpot, busy, slow, stop, yfork, cross, obstacle,
                                   overlap, score, binary, model, video); // 添加构造函数
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
        fileStr >> configs;
        try
        {
            config = configs.get<Config>();
        }
        catch (const nlohmann::detail::exception &e)
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
    bool quit = false;                  // 停标触发退出进程标志(主循环检测后先舵机归中再退出)
private:
};