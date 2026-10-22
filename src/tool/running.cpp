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
 * @file running.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）: 基于UART通信控制，避免一键自启出现问题
 * @version 0.1
 * @date 2026-09-19
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
#include "com/uart.hpp"
#include "utils/detection.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"
#include "ctrl/curvature.hpp"
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <limits.h>
#include <cctype>
#include <algorithm>
#include <fstream>
#include <sys/types.h>
#include <sys/wait.h>

using namespace std;
using namespace cv;

class Running
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;         // 施工区控制（原避障控制）
        shared_ptr<FsmObstacle> obstacle; // 避障控制(锥桶绕行 + 行人礼让)
        shared_ptr<FsmPark> park;   // 停车场控制
        shared_ptr<FsmYfork> yfork; // 岔路口控制
        shared_ptr<FsmStop> stop;   // 停车区控制
        shared_ptr<FsmCross> cross; // 斑马线停车控制
        shared_ptr<FsmFork> fork;   // 停车场岔路控制
        shared_ptr<FsmSlow> slow;   // 慢行区控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    bool parkWasActive = false;           // 上一帧停车场是否激活（检测退出过渡）
    int yforkLock = 0;                    // 停车场退出后锁定 yfork 的剩余帧数
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Uart> uart;                // UART通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器
    shared_ptr<Curvature> curvature;

    // 全局共享数据链
    cv::Mat imgShare;
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};
    std::mutex mtxRes;
    std::condition_variable cvRes;
    std::atomic<bool> stoppingAI{false};
    bool aiHasResult = false;
    bool aiFailed = false;
    bool aiReady = false;
    std::chrono::steady_clock::time_point aiStartupTime{};
    std::chrono::steady_clock::time_point imageTime{};
    std::chrono::steady_clock::time_point resultTime{};
    std::chrono::steady_clock::time_point previousFrameTime{};
    std::uint64_t imageFrameShare = 0;
    std::uint64_t resultFrameShare = 0;
    cv::Mat resultImageShare;
    std::vector<PredictResult> resultsShare{}; // mtxRes 保护的最新完整推理结果
    std::atomic<bool> readyRes{false}; // 已废弃：无消费者，当前不再写入，保留声明

    /**
     * @brief 鼠标的事件回调函数
     *
     */
    static void callbackMouse(int event, int x, int y, int flags, void *userdata)
    {
        Running *self = static_cast<Running *>(userdata);
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
        try
        {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this]
                   { return readyImg.load() || stoppingAI.load(); });
        if (stoppingAI.load())
            return;
        cv::Mat img = imgShare.clone(); // 图像拷贝出来再释放锁
        const auto sourceTime = imageTime;
        const auto sourceFrame = imageFrameShare;
        readyImg = false;
        lock.unlock();

        // 启动AI推理
            detection->inference(img);
            std::lock_guard<std::mutex> lock_result(mtxRes);
            resultsShare = detection->results;
            resultTime = sourceTime;
            resultFrameShare = sourceFrame;
            resultImageShare = img;
            aiHasResult = true;
        }
        catch (const std::exception &error)
        {
            std::cerr << "[Error]: AI inference failed: " << error.what() << std::endl;
            std::lock_guard<std::mutex> lock_result(mtxRes);
            aiFailed = true;
        }
        cvRes.notify_one();
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        params->ctrl.yforkPath = false;
        params->ctrl.yforkSpeedCap = 0.0;
        params->mode = FsmMode::NORMAL;

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        if (params->mode == FsmMode::NORMAL)
            params->mode = fsmFactory.cross->getMode();
        if (params->mode == FsmMode::NORMAL) // 停车场图像处理
        {
            fsmFactory.park->run(img);
            FsmMode mode = fsmFactory.park->getMode();
            if (mode != FsmMode::NORMAL)
                params->mode = mode;
        }
        {
            bool parkActive = (fsmFactory.park->getMode() != FsmMode::NORMAL);
            if (parkWasActive && !parkActive)
                yforkLock = params->config.yforkLockFrames;
            parkWasActive = parkActive;
        }
        if (yforkLock > 0)
            yforkLock--;
        if (params->mode == FsmMode::NORMAL && yforkLock == 0)
        {
            fsmFactory.yfork->run(img);
            params->mode = fsmFactory.yfork->getMode();
        }

        if (params->mode == FsmMode::NORMAL) // 岔路识别与规划
        {
            fsmFactory.fork->run(img);
            params->mode = fsmFactory.fork->getMode();
        }
        fsmFactory.obstacle->run(img); // 避障
                                                
        fsmFactory.slow->run(img); // 慢行区识别与规划
        if (params->mode == FsmMode::NORMAL)
            params->mode = fsmFactory.slow->getMode();

        if (params->mode == FsmMode::NORMAL)
        {
            fsmFactory.busy->run(img); // 施工区识别与规划
            params->mode = fsmFactory.busy->getMode();
        }

        if (params->mode != FsmMode::PARK)
        {
            params->ctrl.back = false;
            params->ctrl.parking = false;
            if (!params->ctrl.yforkPath)
                params->ctrl.fitting = false;
        }

        if (params->mode != params->modeLast)
        {
            uart->buzzerSound(uart->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }

public:
    double fpsDisplay = 0.0;
    /**
     * @brief 参数初始化
     *
     */
    Running()
    {
        // 初始化TCP通信客户端
        uart = make_shared<Uart>();
        if (uart->open("/dev/ttyUSB0") < 0)
        {
            printf("[Error]: Uart init failed!!!\n");
            exit(-1);
        }
        uart->carControl(0, PWMSERVOMID);
        uart->buzzerSound(uart->BUZZER_OK); // 提示音效
        try
        {
            params = make_shared<Params>();
            center = make_shared<Center>();
            motion = make_shared<Motion>();
            curvature = make_shared<Curvature>();
            curvature->configure(params->config);
            predeal = make_shared<Predeal>(params->config.binary);
            detection = make_shared<Detection>(params->config.model, params->config.score);
        }
        catch (const std::exception &error)
        {
            uart->carControl(0, PWMSERVOMID);
            std::cerr << "[Error]: Initialization failed, stopped: " << error.what() << std::endl;
            exit(-1);
        }

        // 相机初始化
        // USB摄像头初始化
        if (params->config.debug && !params->config.debugCamera)
            capture = make_shared<cv::VideoCapture>(params->config.video);
        else
            capture = make_shared<cv::VideoCapture>(params->config.cameraDevice);
        if (!capture->isOpened())
        {
            if (params->config.debug && !params->config.debugCamera)
            {
                char cwd[PATH_MAX] = {};
                const bool cwdValid = getcwd(cwd, sizeof(cwd)) != nullptr;
                std::string videoPath = params->config.video;
                std::string samplesPath = "../res/samples";
                if (cwdValid)
                {
                    if (videoPath.empty() || videoPath.front() != '/')
                        videoPath = std::string(cwd) + "/" + videoPath;
                    samplesPath = std::string(cwd) + "/" + samplesPath;
                }
                else
                    perror("[Error]: getcwd failed; paths below are relative");
                std::cerr << "[Error]: Can not open debug video: " << videoPath << std::endl;
                std::cerr << "可用视频候选（可读普通文件，解码支持以 OpenCV 为准）：" << samplesPath << std::endl;
                DIR *directory = opendir(samplesPath.c_str());
                if (!directory)
                    perror("[Error]: Can not list res/samples");
                else
                {
                    const std::vector<std::string> extensions = {".mp4", ".avi", ".mov", ".mkv", ".m4v", ".mpg", ".mpeg", ".wmv", ".webm", ".ts", ".mts", ".m2ts", ".mjpeg", ".h264", ".h265"};
                    bool foundVideo = false;
                    while (struct dirent *entry = readdir(directory))
                    {
                        std::string filename = entry->d_name;
                        const auto dot = filename.find_last_of('.');
                        if (dot == std::string::npos)
                            continue;
                        std::string extension = filename.substr(dot);
                        std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char ch)
                                       { return static_cast<char>(std::tolower(ch)); });
                        if (std::find(extensions.begin(), extensions.end(), extension) == extensions.end())
                            continue;
                        const std::string availablePath = samplesPath + "/" + filename;
                        struct stat fileInfo = {};
                        if (stat(availablePath.c_str(), &fileInfo) == 0 && S_ISREG(fileInfo.st_mode) && access(availablePath.c_str(), R_OK) == 0)
                        {
                            std::cerr << "  " << availablePath << std::endl;
                            foundVideo = true;
                        }
                    }
                    closedir(directory);
                    if (!foundVideo)
                        std::cerr << "  未找到可读的视频候选文件" << std::endl;
                }
                std::cerr << "请把 config.json 的 video 指向其中之一" << std::endl;
            }
            else
                std::cerr << "[Error]: Can not open camera: " << params->config.cameraDevice << std::endl;
            exit(-1);
        }
        capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        capture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率

        if (params->config.debug)
        {
            show = make_shared<Show>(4);
            show->live = params->config.debugCamera;
            if (!show->live)
            {
                show->frameMax = std::max(0, static_cast<int>(capture->get(cv::CAP_PROP_FRAME_COUNT)) - 1);
                if (show->frameMax > 0)
                    cv::createTrackbar("Frame", "ICAR", &show->index, show->frameMax, [](int, void *) {});
                cv::setMouseCallback("ICAR", this->callbackMouse, this);
            }
            std::cout << "[Preview] " << (show->live ? params->config.cameraDevice : params->config.video)
                      << " | motor output stays zero; q/Esc exits" << std::endl;
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);         // 施工区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);   // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);   // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params); // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);   // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);   // 慢行区控制实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params); // 岔路口控制实例化

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Running::runModel, this));
        if (!params->config.debug)
            loops->start(); // RL开始推理

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Running()
    {
        {
            std::lock_guard<std::mutex> lock(mtxImg);
            stoppingAI = true;
        }
        cvImg.notify_all();
        if (loops)
            loops->shutdown();
    };

    /**
     * @brief 程序主循环
     *
     */
    void running()
    {
        try
        {
        cv::Mat img;
        if (params->config.debug)
        {
            uart->carControl(0, PWMSERVOMID);
            if (uart->keypress || show->quit)
            {
                uart->buzzerSound(uart->BUZZER_FINISH);
                printf("-----> Preview Exit!!! <-----\n");
                exit(0);
            }
            if (!show->live && show->indexLast == show->index)
            {
                show->show();
                uart->sendHeart();
                usleep(10 * 1000);
                return;
            }
            if (!show->live)
                capture->set(cv::CAP_PROP_POS_FRAMES, show->index);
        }
        if (!capture->read(img) || img.empty())
        {
            uart->carControl(0, PWMSERVOMID);
            std::cerr << "[Error]: Input frame lost, stopped." << std::endl;
            params->quit = true;
            exit(-1);
        }
        if (params->config.debug)
        {
            if (show->live)
                show->index++;
            else
                show->indexLast = show->index;
        }
        const auto frameTime = std::chrono::steady_clock::now();
        if (params->config.debug && !params->config.debugCamera)
        {
            const double fps = capture->get(cv::CAP_PROP_FPS);
            params->ctrl.frameDt = std::isfinite(fps) && fps > 1.0 ? 1.0 / fps : 1.0 / 30.0;
        }
        else if (previousFrameTime != std::chrono::steady_clock::time_point{})
            params->ctrl.frameDt = std::clamp(std::chrono::duration<double>(frameTime - previousFrameTime).count(),
                                             1.0 / 120.0, 0.2);
        previousFrameTime = frameTime;
        ++params->cameraFrame;
        if (img.size() != cv::Size(COLSIMAGE, ROWSIMAGE))
            cv::resize(img, img, cv::Size(COLSIMAGE, ROWSIMAGE));

        // 帧首快照：本帧 FSM 与绘图共用，后续不再更新
        //[02] 图像存储
        if (params->config.saveImg && (!params->config.debug || params->config.debugCamera)) // 存储原始图像
            savePicture(img);
        else if (params->config.saveImg && params->config.debug && !params->config.debugCamera) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        cv::Mat imgBin;
        predeal->correction(img); // 图像矫正
        params->cameraImage = img;
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        if (params->config.debug)
        {
            detection->inference(img);
            params->results = detection->results;
            params->detectionFrame = params->cameraFrame;
            params->detectionImage = img;
        }
        else
        {
            if (aiStartupTime == std::chrono::steady_clock::time_point{})
                aiStartupTime = frameTime;
            // 锁只保护共享缓冲的交接，不要扩大作用域
            std::lock_guard<std::mutex> lock(mtxImg);
            img.copyTo(imgShare);
            imageTime = frameTime;
            imageFrameShare = params->cameraFrame;
            readyImg = true;
            cvImg.notify_one();
        }
        /*-------------------------------------------------------*/
        if (!params->config.debug)
        {
            std::unique_lock<std::mutex> lock_result(mtxRes);
            if (!aiHasResult && !aiFailed)
                cvRes.wait_for(lock_result, std::chrono::milliseconds(params->config.aiStartupTimeoutMs),
                               [this] { return aiHasResult || aiFailed; });
            const auto now = std::chrono::steady_clock::now();
            const bool fresh = aiHasResult && now - resultTime <=
                std::chrono::milliseconds(params->config.aiResultMaxAgeMs);
            if (aiFailed || (!fresh && (aiReady || now - aiStartupTime >=
                std::chrono::milliseconds(params->config.aiStartupTimeoutMs))))
            {
                uart->carControl(0, PWMSERVOMID);
                std::cerr << "[Error]: AI startup/result timeout, stopped." << std::endl;
                params->quit = true;
                exit(-1);
            }
            if (!fresh)
            {
                uart->carControl(0, PWMSERVOMID);
                return;
            }
            aiReady = true;
            params->results = resultsShare;
            params->detectionFrame = resultFrameShare;
            params->detectionImage = resultImageShare;
        }
        imgBin = predeal->binaryzation(img); // 图像二值化

        //[04] 赛道识别
        params->track->handle(imgBin);
        params->ctrl.lineArea = params->track->lineArea;
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
        curvature->update(params);

        //[07] 车辆运动控制
        motion->poseControl(params);
        motion->speedControl(params);

        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img, params->results);        // 图像绘制AI结果
            center->drawImage(params, img); // 图像绘制控制路径
            motion->drawImage(params, img); // 图像绘制速度
            if (curvature->enabled)
            {
                putText(img, "K:" + doble2String(curvature->curvature, 3) +
                        " R:" + doble2String(curvature->radiusM, 2) +
                        " Lim:" + doble2String(curvature->speedLimit, 2),
                        Point(8, 36), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 255, 255), 1);
                putText(img, "FF:" + doble2String(params->ctrl.steerFF, 1) +
                        " Valid:" + to_string(curvature->valid),
                        Point(8, 50), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 255, 255), 1);
            }
            show->setNewWindow(3, "Ctrl", img);

            // 特殊区域图像处理结果显示
            Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
            params->track->drawImage(imgRes);
            center->drawImage(params, imgRes);
            fsmFactory.busy->show(imgRes);
            fsmFactory.park->show(imgRes);
            fsmFactory.stop->show(imgRes);
            fsmFactory.cross->show(imgRes);
            fsmFactory.fork->show(imgRes);
            fsmFactory.slow->show(imgRes);
            fsmFactory.yfork->show(imgRes);
            show->drawStatus(imgRes, *params, yforkLock);
            show->setNewWindow(4, "FSM", imgRes);
            show->show();
            uart->carControl(0, PWMSERVOMID);
            uart->sendHeart();
        }
        else // 实车控制
        {
            uart->carControl(params->ctrl.speed, params->ctrl.servo); // 串口通信控制车辆
        }

        if (params->quit) // 停标触发退出：先舵机归中再退出进程
        {
            params->ctrl.servo = PWMSERVOMID; // 舵机归中
            params->ctrl.speed = 0;           // 停车
            uart->carControl(0, PWMSERVOMID); // 发送归中+停车指令
            printf("-----> System Exit (servo centered)! <-----\n");
            exit(0); // 程序退出
        }
        }
        catch (const std::exception &error)
        {
            params->ctrl.stop = true;
            params->quit = true;
            uart->carControl(0, PWMSERVOMID);
            std::cerr << "[Error]: Main loop failed, stopped: " << error.what() << std::endl;
            exit(-1);
        }
    }
};

int main(int argc, char const *argv[])
{
    Running icar;

    // 强制杀死TCP通信进程
    std::system("killall -9 icar boot");
    sleep(10);

    const std::chrono::milliseconds durations(1000 / 30); // 控制周期：30Fps
    while (1)
    {
        auto timeStart = std::chrono::high_resolution_clock::now();
        icar.running(); // 系统主线程

        // 计算处理耗时
        auto timeEnd = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart);
        //printf(">> FrameTime: %ldms | %.2ffps \n", elapsed.count(), 1000.0 / elapsed.count());
        icar.fpsDisplay = 1000.0 / std::max<int64_t>(1, elapsed.count()); // 与print同源,画到图上

        // 睡眠等待
        if (elapsed < durations)
            std::this_thread::sleep_for(durations - elapsed);
    }

    return 0;
}
