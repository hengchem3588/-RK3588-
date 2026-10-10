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
 * @file obstacle.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障控制
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 避障控制
 *
 */
class FsmObstacle : public FSMState
{
public:
  FsmObstacle(std::shared_ptr<Params> par);
  ~FsmObstacle();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();

private:
  /**
   * @brief 避障执行步骤
   *
   */
  enum Step
  {
    NONE = 0, // 未知状态
    ENABLE    // 避障使能
  };

  Step step = Step::NONE; // 避障步骤
  int countRec = 0;       // AI目标识别计数器
  int countSes = 0;       // 让路/丢失计数器
  int timeout = 0;        // 超时计数器
  int lostFrames = 0;     // 目标丢失计数
  PredictResult resultObs; // 避障目标（锥桶/行人）

  void setStep(Step st);
  PredictResult findTarget();                 // 搜索目标，取面积最大
  void planObstacle(PredictResult obs);       // 锥桶贝塞尔绕行 / 行人对侧中线收缩
  void curtailTracking(bool left);            // 双车道→单车道中线收缩
};