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
 * @file slow.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 减速区识别与规划
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 减速区识别与规划
 *
 */
class FsmSlow : public FSMState
{
public:
  FsmSlow(std::shared_ptr<Params> par);
  ~FsmSlow();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();

private:
  /**
   * @brief 场景状态
   *
   */
  enum Step
  {
    NONE = 0, // 未知状态
    ENABLE,   // 场景使能
  };
  Step step = Step::NONE; // 场景状态
  uint16_t countRec = 0;  // AI场景识别计数器
  uint16_t countSes = 0;  // 场次计数器
  uint16_t timeout = 0;   // 程序退出计数器

  void setStep(Step st);
};
