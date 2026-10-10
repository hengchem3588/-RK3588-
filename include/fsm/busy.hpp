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
 * @file busy.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 施工区控制（fork标志进出 + 区内限速/解除限速 + 锥桶避障）
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 施工区控制
 *
 */
class FsmBusy : public FSMState
{
public:
  FsmBusy(std::shared_ptr<Params> par);
  ~FsmBusy();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();
  bool slowing = false; // 减速使能

private:
  /**
   * @brief 施工区执行步骤
   *
   */
  enum Step
  {
    NONE = 0,  // 未知状态
    ENABLE,    // 施工区使能
    FORKIN,    // 进区岔路转向
    RUNNING,   // 区内巡线（限速/避障）
    FORKOUT,   // 出区岔路转向
  };

  Step step = Step::NONE;   // 施工区步骤
  bool enable = false;      // 场景检测使能标志
  int countRec = 0;         // AI场景识别计数器
  int countSes = 0;         // 场次计数器
  int timeout = 0;          // 超时计数器
  bool slowingIn = false;   // 区内低速标志（限速）
  PredictResult resultObs;  // 避障目标锥桶

  void setStep(Step st);
  void replanTracking(bool left);
  bool findSymbols(vector<PredictResult> results, int label);
  void curtailTracking(bool left);
};