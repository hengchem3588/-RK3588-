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
 * @file yfork.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 岔路口(Y型)控制：AI识别左/右岔路标牌，选择对应分支
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 岔路口(Y型)控制
 *
 */
class FsmYfork : public FSMState
{
public:
  FsmYfork(std::shared_ptr<Params> par);
  ~FsmYfork();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();

  struct Status
  {
    const char *step = "WAIT";
    int direction = 0;
    int votes = 0;
    int nearSplitRows = 0;
    int splitTipRow = -1;
    int minClearance = 0;
    int minRoadClearance = -1;
    int minObstacleClearance = -1;
    int minObstacleRow = -1;
    int minObstaclePathCol = -1;
    int obstacleLeftAtMin = -1;
    int obstacleRightAtMin = -1;
    int lostPathFrames = 0;
    bool normalAligned = false;
    bool obstacleTracked = false;
    int obstacleMemoryFrames = 0;
    cv::Rect exclusion;
    cv::Rect obstacleBox;
    std::string stopReason;
  };

  Status status() const;

private:
  enum class Step
  {
    WAIT,
    APPROACH,
    TURN,
    EXIT,
    COOLDOWN,
    SAFE_STOP
  };

  struct Observation
  {
    bool present = false;
    bool fork = false;
    int direction = 0;
    cv::Rect box;
  };

  Step step = Step::WAIT;
  int direction = 0;
  int candidateDirection = 0;
  int votes = 0;
  int forkVotes = 0;
  int exitVotes = 0;
  int lostPathFrames = 0;
  int seedCol = COLSIMAGE / 2;
  int dividerCol = COLSIMAGE / 2;
  int minClearance = 0;
  int minRoadClearance = -1;
  int minObstacleClearance = -1;
  int minObstacleRow = -1;
  int minObstaclePathCol = -1;
  int obstacleLeftAtMin = -1;
  int obstacleRightAtMin = -1;
  double stateFrames = 0.0;
  double turnFrames = 0.0;
  double signLostFrames = 0.0;
  double obstacleAgeFrames = 0.0;
  double noObservationFrames = 0.0;
  double yforkSpeedCapCurrent = 0.0;
  double lastSafePathAgeFrames = 1e9;
  double lastSafePreviewRow = 125.0;
  bool yforkSpeedCapInitialized = false;
  bool sawSplit = false;
  bool obstacleWasNear = false;
  bool normalAligned = false;
  std::uint64_t lastDetectionFrame = 0;
  cv::Rect signBox;
  cv::Rect exclusion;
  cv::Rect obstacleBox;
  cv::Mat obstacleMask;
  cv::Point2f signVelocity{0.0f, 0.0f};
  bool obstacleTracked = false;
  Track::BranchGeometry geometry;
  std::vector<PointX> lastSafePath;
  int lastSafePathDirection = 0;
  bool planBlockedByObstacle = false;
  std::string stopReason;

  Observation observe() const;
  void updateSign(const Observation &observation, double elapsedFrames);
  cv::Mat buildObstacleMask(const cv::Rect &referenceBox) const;
  void updateObstacle(const cv::Rect &referenceBox, bool referenceUpdated, double elapsedFrames);
  bool obstacleSpanAt(int row, int &left, int &right) const;
  bool lastSafePathClear() const;
  bool restoreLastSafePath();
  void updateSpeedCap(double target, double elapsedFrames);
  void applyObstacleSpeedCap(double elapsedFrames);
  bool planBranch(const Mat &img);
  bool exitReady() const;
  void safeStop(const std::string &reason);
  void setStep(Step st);
  void reset();
  const char *stepName() const;
};
