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
 * @file icar.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"
#include "fsm/obstacle.hpp"
#include "fsm/fork.hpp"
#include "fsm/park.hpp"
#include "fsm/slow.hpp"
#include "fsm/stop.hpp"
#include "fsm/cross.hpp"
#include "fsm/yfork.hpp"
#include "com/client.hpp"
#include "utils/detection.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"

using namespace std;
using namespace cv;

class Icar
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;       // 施工区控制
        shared_ptr<FsmObstacle> obstacle; // 避障控制
        shared_ptr<FsmPark> park;   // 停车场控制
        shared_ptr<FsmStop> stop;   // 停车区控制
        shared_ptr<FsmCross> cross; // 斑马线停车控制
        shared_ptr<FsmFork> fork;   // 停车场岔路控制
        shared_ptr<FsmSlow> slow;   // 慢行区控制
        shared_ptr<FsmYfork> yfork; // 岔路口控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    bool parkWasActive = false;           // 上一帧停车场是否激活（检测退出过渡）
    int yforkLock = 0;                    // 停车场退出后锁定 yfork 的剩余帧数
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Client> client;            // TCP客户端通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器

    // 全局共享数据链
    cv::Mat imgShare;
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};
    std::mutex mtxRes;
    std::atomic<bool> readyRes{false};

    /**
     * @brief 鼠标的事件回调函数
     *
     */
    static void callbackMouse(int event, int x, int y, int flags, void *userdata)
    {
        Icar *self = static_cast<Icar *>(userdata);
        if (self)
            self->handleMouse(event, x, y, flags);
    }
    void handleMouse(int event, int x, int y, int flags)
    {
        double value;
        switch (event)
        {
        case cv::EVENT_MOUSEWHEEL: // 鼠标滑球
        {
            value = cv::getMouseWheelDelta(flags); // 获取滑球滚动值
            if (value > 0)
                show->index++;
            else if (value < 0)
                show->index--;

            if (show->index < 0)
                show->index = 0;
            if (show->index > show->frameMax)
                if (show->index > show->frameMax)
                    show->index = show->frameMax;
            break;
        }
        default:
            break;
        }
    }

    /**
     * @brief AI 模型推理
     *
     */
    void runModel()
    {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this]
                   { return readyImg.load(); });
        cv::Mat img = imgShare.clone(); // 图像拷贝出来再释放锁
        readyImg = false;
        lock.unlock();

        // 启动AI推理
        detection->inference(img);
        std::lock_guard<std::mutex> lock_result(mtxRes);
        params->results = detection->results;
        readyRes = true;
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        if (params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW) // 状态复位
            params->mode = FsmMode::NORMAL;

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        params->mode = fsmFactory.cross->getMode();
        if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK) // 停车场图像处理
        {
            fsmFactory.park->run(img);
            FsmMode mode = fsmFactory.park->getMode();
            if (mode != FsmMode::NORMAL)
                params->mode = mode;
        }
        // 停车场退出后 100 帧内屏蔽 yfork 触发：park 从激活回到 NORMAL 的过渡视为"刚退出"
        {
            bool parkActive = (fsmFactory.park->getMode() != FsmMode::NORMAL);
            if (parkWasActive && !parkActive)
                yforkLock = 100; // 刚退出停车场，锁定 yfork
            parkWasActive = parkActive;
        }
        if (yforkLock > 0)
            yforkLock--;
        if (params->mode == FsmMode::NORMAL) // 岔路识别与规划
        {
            fsmFactory.fork->run(img);
            params->mode = fsmFactory.fork->getMode();
        }

        fsmFactory.obstacle->run(img); // 避障(锥桶贝塞尔绕行 + 行人对侧中线收缩)：track级别
                                                // 不占用任何FsmMode，任何状态下见到目标即重画边线；getMode恒NORMAL，不抢占slow/busy
        fsmFactory.slow->run(img); // 慢行区识别与规划
        if (params->mode == FsmMode::NORMAL)
            params->mode = fsmFactory.slow->getMode();

        fsmFactory.busy->run(img); // 施工区识别与规划
        if (params->mode == FsmMode::NORMAL)
            params->mode = fsmFactory.busy->getMode();

        if (yforkLock == 0) // 停车场退出后100帧内锁定 yfork，防止刚出停车场误触发岔路
        {
            fsmFactory.yfork->run(img); // 岔路口识别与规划
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.yfork->getMode();
        }

        if (params->mode != params->modeLast)
        {
            client->buzzerSound(client->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }

public:
    double fpsDisplay = 0.0;
    /**
     * @brief 参数初始化
     *
     */
    Icar()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        motion = make_shared<Motion>();                        // 运动控制器
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化TCP通信客户端
        client = make_shared<Client>();
        if (!client->start())
        {
            printf("[Error]: Socket init failed!!!\n");
            exit(-1);
        }
        client->buzzerSound(client->BUZZER_OK); // 提示音效

        // 相机初始化
        // USB摄像头初始化
        if (params->config.debug)
            capture = make_shared<cv::VideoCapture>(params->config.video); // 打开本地视频
        else
            capture = make_shared<cv::VideoCapture>("/dev/video0"); // 打开摄像头
        if (!capture->isOpened())
        {
            printf("[Error]: Can not open video device!!!\n");
            exit(-1);
        }
        capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        capture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率

        if (params->config.debug)
        {
            show = make_shared<Show>(4); // 调试UI初始化
            show->frameMax = capture->get(cv::CAP_PROP_FRAME_COUNT) - 1;
            cv::createTrackbar("Frame", "ICAR", &show->index, show->frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
            cv::setMouseCallback("ICAR", this->callbackMouse);                                     // 创建鼠标键盘快捷键事件
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);       // 施工区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);   // 停车场控制实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params); // 岔路口控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);   // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params); // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);   // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);   // 慢行区控制实例化

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Icar::runModel, this));
        loops->start(); // RL开始推理

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Icar() {};

    /**
     * @brief 程序主循环
     *
     */
    void running()
    {
        //[01] 视频源读取
        cv::Mat img;
        if (params->config.debug) // 综合显示调试UI窗口
        {
            if (show->indexLast == show->index) // 图像帧未更新
            {
                if (client->keypress)
                {
                    client->buzzerSound(client->BUZZER_FINISH); // 祖传提示音效
                    printf("-----> System Exit!!! <-----\n");
                    exit(0); // 程序退出
                }
                show->show();        // 显示综合绘图
                client->sendHeart(); // 发送给服务器在线心跳
                usleep(10 * 1000);   // us延迟
                return;
            }

            capture->set(cv::CAP_PROP_POS_FRAMES, show->index); // 设置读取帧
            if (!capture->read(img))
                return;
            show->indexLast = show->index;
        }
        else if (!capture->read(img))
            return;

        //[02] 图像存储
        if (params->config.saveImg && !params->config.debug) // 存储原始图像
            savePicture(img);
        else if (params->config.saveImg && params->config.debug) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        cv::Mat imgBin;
        predeal->correction(img); // 图像矫正
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        std::lock_guard<std::mutex> lock(mtxImg);
        imgShare = img.clone();
        readyImg = true;
        cvImg.notify_one();
        /*-------------------------------------------------------*/
        imgBin = predeal->binaryzation(img); // 图像二值化

        //[04] 赛道识别
        params->track->handle(imgBin);
        if (params->config.debug)
        {
            show->setNewWindow(1, "Bin", imgBin);
            cv::Mat imgTrack = img.clone();
            params->track->drawImage(imgTrack); // 图像绘制赛道识别结果
            show->setNewWindow(2, "Track", imgTrack);
            if (params->config.saveIpm && params->config.saveImg)
            {
                cv::Mat imgIpm;
                ipm.homography(imgTrack, imgIpm);
                savePicture(imgIpm); // 保存图像
            }
        }

        //[05] 有限状态机任务执行
        params->ctrl.fitting = false;
        runFsm(imgBin);

        //[06] 控制中心计算
        center->fitting(params);

        //[07] 车辆运动控制
        motion->poseControl(params);
        motion->speedControl(params);

        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img);        // 图像绘制AI结果
            center->drawImage(params, img); // 图像绘制控制路径
            motion->drawImage(params, img); // 图像绘制速度
            show->setNewWindow(3, "Ctrl", img);

            // 特殊区域图像处理结果显示
            Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
            fsmFactory.busy->show(imgRes);
            fsmFactory.park->show(imgRes);
            fsmFactory.stop->show(imgRes);
            fsmFactory.cross->show(imgRes);
            fsmFactory.fork->show(imgRes);
            fsmFactory.slow->show(imgRes);
            fsmFactory.yfork->show(imgRes);
            show->setNewWindow(4, "FSM", imgRes);
        }
        else // 实车控制
        {
            client->carControl(params->ctrl.speed, params->ctrl.servo); // 串口通信控制车辆
        }

        if (params->quit) // 停标触发退出：先舵机归中再退出进程
        {
            params->ctrl.servo = PWMSERVOMID;   // 舵机归中
            params->ctrl.speed = 0;             // 停车
            client->carControl(0, PWMSERVOMID); // 发送归中+停车指令
            printf("-----> System Exit (servo centered)! <-----\n");
            exit(0); // 程序退出
        }
    }
};
