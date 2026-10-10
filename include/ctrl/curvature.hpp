#pragma once
/**
 ********************************************************************************************************
 *                                    icar_autopilot_2026th  扩展模块
 *
 *  @file curvature.hpp
 *  @brief 曲率预瞄 / 纯追踪前馈 / 曲率限速规划器
 *
 *  本文件为「国一冲刺」新增模块，用于替换官方代码中"纯 PD + 8 档离散速度"的控制方案。
 *
 *  解决的原工程缺口：
 *    1) tools.hpp 里没有任何 atan2 / 曲率 / 角度函数（已全量 grep 确认），
 *       "曲率预瞄 + 前馈转向"在官方工程里无法直接落地；
 *    2) motion.hpp 的 poseControl 是纯 PD（无前馈），弯道必然滞后——
 *       PD 只能"看到偏差才纠正"，前馈可以"提前打方向"；
 *    3) motion.hpp 的 speedControl 只有 8 个离散档位（if-else 短路链），
 *       且 ctrl.lineArea 全工程只读不写 → 动态速度恒等于 velHigh（全程恒速）。
 *
 *  设计要点：
 *    - **纯追踪前馈在像素域内是几何精确的**：前视点的横向偏移与前瞻距离之比
 *      是无量纲量（tan α），与像素/米的换算无关，因此可以像 runP1/turnP 一样
 *      直接调参，不需要先完成相机标定。这是本模块能"当天上车"的关键。
 *    - 曲率限速需要"像素 → 米"的比例。用一个可标定的 `pxPerMeter` 承担，
 *      默认 1.0（此时退化为"按弯道急缓程度限速"，本身就是相对原代码的净增益），
 *      并按文档给出的 3 步标定法收敛到真实值。
 *    - 所有输出都做了兜底：输入点太少 / 曲率异常时不输出前馈、不限速，
 *      保证最坏情况下退化为原始 PD 行为，不会让车变差。
 *********************************************************************************************************
 */

#include <cmath>
#include <vector>
#include <memory>
#include "utils/tools.hpp"  // PointX、COLSIMAGE / ROWSIMAGE 等尺寸宏
#include "utils/params.hpp" // Params / Control（经 track.hpp 引入 Track，含 PointX）

/**
 * @brief 曲率预瞄与速度规划器
 *
 * 用法（在 Icar::running() 的 motion 调用之前）：
 *      curvature->update(params);                  // 计算前馈角与限速
 *      motion->poseControl(params);                // 内部叠加 ctrl.steerFF
 *      motion->speedControl(params);               // 内部使用 ctrl.speedLimit
 */
class Curvature
{
public:
    // ---------------- 可调参数（全部可在 config.json 里暴露） ----------------

    double previewPx   = 55.0;   // 纯追踪前瞻距离（像素）。越大越平顺、越小越激进
    double ffGain      = 1.0;    // 前馈增益。1.0 = 几何精确纯追踪；>1 更激进，<1 更保守
    double ffLimit     = 260.0;  // 前馈舵机 PWM 增量上限（防止异常点导致打死方向）
    double minPreviewRow = 20.0; // 最低有效前瞻行，低于此值认为前瞻太近，不输出前馈

    // 曲率限速：a_lat = v^2 * kappa  →  v = sqrt(aLatMax / kappa)
    double aLatMax     = 2.2;    // 允许的最大侧向加速度 (m/s^2)。1.5 保守 / 3.0 激进
    double pxPerMeter  = 1.0;    // 像素→米 比例（车头附近）。标定方法见代码补丁集 A3 节
    double speedFloor  = 0.45;   // 限速下限 (m/s)：绝不低于此值，避免弯中停车
    double decelRate   = 0.35;   // 降速平滑系数 (0~1]，越大降得越快
    double accelRate   = 0.08;   // 升速平滑系数 (0~1]，越小升得越缓（防出弯甩尾）

    // ---------------- 输出（本帧结果，供调试显示与 FSM 使用） ----------------

    double curvature   = 0.0;    // 估计曲率 (1/m，已按 pxPerMeter 换算)
    double radiusM     = 0.0;    // 曲率半径 (m)，0 表示直线
    double speedLimit  = 0.0;    // 曲率限速 (m/s)，0 表示不限速
    double previewTan  = 0.0;    // 前视点 tan(α)（无量纲，横向像素 / 前瞻像素）
    bool   valid       = false;  // 本帧估计是否有效

    /**
     * @brief 主入口：从 params->ctrl.centerEdge 估计曲率、前馈角与限速
     *
     * 计算完成后会把结果一并写回 params->ctrl 的两个新字段：
     *      params->ctrl.steerFF     —— 纯追踪前馈的舵机 PWM 增量（已限幅）
     *      params->ctrl.speedLimit  —— 曲率限速 (m/s)，0 表示不限速
     * 这样 Motion 侧只需读取，不必持有 Curvature 的引用。
     *
     * @param params 全局参数（会写入 ctrl.steerFF / ctrl.speedLimit）
     */
    void update(std::shared_ptr<Params> &params);

    /**
     * @brief 曲率标定辅助：返回当前几何下"1 米"对应多少像素（用于 pxPerMeter 自查）
     *
     * @return double 估计值；数据不足返回 0
     */
    double estimatePxPerMeter() const { return lastPxPerMeter_; }

private:
    // 升速/降速分离的平滑状态（避免出弯瞬间速度突增导致甩尾）
    double speedSmooth_    = -1.0;
    double lastPxPerMeter_ = 0.0;

    /**
     * @brief 输入安全性检查：点集是否足够且连续
     */
    static bool validEdge(const std::vector<PointX> &edge)
    {
        return edge.size() >= 12; // 少于 12 个点不估计曲率（噪声主导）
    }

    /**
     * @brief 在中心线上按"像素索引"取一个点，越界时钳制
     */
    static PointX at(const std::vector<PointX> &edge, int i)
    {
        if (i < 0)
            i = 0;
        if (i >= (int)edge.size())
            i = (int)edge.size() - 1;
        return edge[i];
    }

    /**
     * @brief 三点曲率（Menger 曲率），输入为 (s, d) 平面坐标
     *
     * kappa = 2 * cross(A,B,C) / (|AB| * |BC| * |CA|)
     *
     * 注意：调用方传入的 (s, d) 中 d 轴方向被翻转过（见 point2sd 说明），
     * 因此本函数的符号约定为：**kappa > 0 表示赛道右转**。
     */
    static double mengerCurvature(double ax, double ay,
                                  double bx, double by,
                                  double cx, double cy,
                                  double *radiusOut = nullptr)
    {
        const double abx = bx - ax, aby = by - ay;
        const double bcx = cx - bx, bcy = cy - by;
        const double cax = ax - cx, cay = ay - cy;

        const double cross = abx * bcy - aby * bcx; // 2 * 有向面积
        const double lab = std::sqrt(abx * abx + aby * aby);
        const double lbc = std::sqrt(bcx * bcx + bcy * bcy);
        const double lca = std::sqrt(cax * cax + cay * cay);

        const double denom = lab * lbc * lca;
        if (denom < 1e-9) // 三点共线
        {
            if (radiusOut)
                *radiusOut = 0.0;
            return 0.0;
        }

        const double kappa = 2.0 * cross / denom;
        if (radiusOut)
            *radiusOut = (std::fabs(kappa) > 1e-9) ? (1.0 / std::fabs(kappa)) : 0.0;
        return kappa;
    }

    /**
     * @brief PointX（转置存储：.x = 行, .y = 列）→ 局部平面坐标 (s, d)
     *
     * 局部平面定义（右手系，车头为原点）：
     *      s = 向前的距离  = ROWSIMAGE - .x      （行号越小越远 → s 越大）
     *      d = 向右的偏移  = COLSIMAGE/2 - .y    （列号越大越靠右 → d 越大）
     *
     * 注意 d 取的是"向右为正"，与图像列方向相反，这样 (s, d) 构成
     * 前-右右手系，配合 mengerCurvature 得到"正曲率 = 右转"的直观约定。
     */
    static void point2sd(const PointX &p, double &s, double &d)
    {
        s = (double)(ROWSIMAGE - p.x);
        d = (double)(COLSIMAGE / 2 - p.y);
    }

    /**
     * @brief 在中心线上按"向前弧长/索引"寻找前视点
     *
     * 工程简化：直接用索引步进近似弧长步进。中心线点集来自 Bezier(0.03,...)，
     * 相邻点在行方向上的间距基本均匀（约 3~4 行），因此索引近似线性于弧长。
     * 对纯追踪前馈而言，前瞻距离的绝对精度远比"是否均匀"重要，
     * 因此这里不做等弧长重采样（省一次每帧排序/插值）。
     */
    static int findLookaheadIndex(const std::vector<PointX> &edge, double previewPx)
    {
        // 车头参考点取索引 0（近处），向远处累计行差
        const double row0 = (double)edge[0].x;
        for (int i = 1; i < (int)edge.size(); i++)
        {
            const double dRow = row0 - (double)edge[i].x; // 正数：向前
            if (dRow >= previewPx)
                return i;
        }
        return (int)edge.size() - 1; // 前瞻不足则用最远点
    }
};

/**
 * @brief 曲率预瞄主逻辑（实现放在头文件内，保持与工程既有风格一致：大部分类为 header-only）
 *
 * @param params 全局参数
 */
inline void Curvature::update(std::shared_ptr<Params> &params)
{
    // 每帧清零，保证任何异常路径都退化为"无前馈、无限速"
    curvature  = 0.0;
    radiusM    = 0.0;
    speedLimit = 0.0;
    previewTan = 0.0;
    valid      = false;

    params->ctrl.steerFF    = 0.0; // 先按"无前馈"输出，后续有效才覆盖
    params->ctrl.speedLimit = 0.0; // 0 = 不限速

    const std::vector<PointX> &edge = params->ctrl.centerEdge;
    if (!validEdge(edge))
    {
        speedSmooth_ = -1.0; // 中线失效：重置平滑，等恢复后重新起步
        return;              // ctrl.steerFF / ctrl.speedLimit 已在上方置 0
    }

    // ------------------------------------------------------------------
    // [1] 曲率估计：用前方一段（而非全网）做三点曲率，抑制远端噪声
    //     取 前视点 与 车头点、中点 三点，窗口跨度适中
    // ------------------------------------------------------------------
    const int idxFar = findLookaheadIndex(edge, previewPx * 2.4); // 曲率估计用更远的点
    const int idxMid = idxFar / 2;

    double s0, d0, s1, d1, s2, d2;
    point2sd(edge[0], s0, d0);
    point2sd(at(edge, idxMid), s1, d1);
    point2sd(at(edge, idxFar), s2, d2);

    double radiusPx = 0.0;
    const double kappaPx = mengerCurvature(s0, d0, s1, d1, s2, d2, &radiusPx);

    // 合理性闸门：曲率半径过小说明是噪声/误识别（不可能有 <15px 的弯）
    const bool kappaSane = (radiusPx == 0.0) || (radiusPx > 15.0);

    // ------------------------------------------------------------------
    // [2] 像素→米 换算
    //     pxPerMeter 定义在"车头附近"（行 ~239 处）。越远的行透视压缩越强，
    //     同一行差对应的真实距离越大。用行号比例做一阶透视补偿：
    //        有效比例 = pxPerMeter * (ROWSIMAGE / 当前代表性行)
    //     这是工程近似（不是严格单应），但足以让"限速随弯道急缓变化"正确工作。
    // ------------------------------------------------------------------
    const double refRow = (double)at(edge, idxFar).x;
    const double persp  = (refRow > 1.0) ? ((double)ROWSIMAGE / refRow) : 1.0;
    lastPxPerMeter_     = pxPerMeter * persp;

    if (kappaSane && idxFar > 4)
    {
        // kappa_px 单位 1/pixel；除以 pxPerMeter 得 1/m
        const double kappaM = kappaPx / (lastPxPerMeter_ > 1e-6 ? lastPxPerMeter_ : 1.0);
        curvature = kappaM;
        radiusM   = (std::fabs(kappaM) > 1e-6) ? (1.0 / std::fabs(kappaM)) : 0.0;

        // ------------------------------------------------------------------
        // [3] 曲率限速：a_lat = v^2 * kappa  →  v = sqrt(aLatMax / |kappa|)
        //     只在确实有曲率时限速；直线段不限速（speedLimit 保持 0）
        // ------------------------------------------------------------------
        const double absK = std::fabs(kappaM);
        if (absK > 1e-4)
        {
            double v = std::sqrt(aLatMax / absK);

            // 远端曲率估计本身误差较大，做一个保守化折扣
            v *= 0.85;

            if (v < speedFloor)
                v = speedFloor;

            speedLimit = v;
        }
    }

    // ------------------------------------------------------------------
    // [4] 纯追踪前馈：前视点横向偏移 / 前瞻距离 = tan(α)
    //     这是**像素域内几何精确**的量，与 pxPerMeter 无关，
    //     因此即使完全没标定也能直接上车使用。
    // ------------------------------------------------------------------
    const int idxLA = findLookaheadIndex(edge, previewPx);
    const double dRowLA = (double)edge[0].x - (double)at(edge, idxLA).x; // 向前行数

    if (dRowLA >= minPreviewRow) // 前瞻足够远才输出前馈
    {
        // 横向偏移：前视点列 与 车头点列 之差（向右为正，与图像列反向）
        const double latPx = (double)edge[0].y - (double)at(edge, idxLA).y;

        // 透视补偿：前视点处的横向像素要折算到"车头等效横向像素"
        const double perspLA = (at(edge, idxLA).x > 1.0)
                                   ? ((double)ROWSIMAGE / (double)at(edge, idxLA).x)
                                   : 1.0;
        const double latEquiv = latPx * perspLA;

        previewTan = latEquiv / dRowLA; // 无量纲
        valid      = true;
    }

    // ------------------------------------------------------------------
    // [5] 限速平滑：升速远慢于降速
    //     —— 出弯瞬间速度突增会导致后轮甩尾（后驱 + 差速器尤其明显）
    // ------------------------------------------------------------------
    if (speedLimit > 0.0)
    {
        if (speedSmooth_ < 0.0)
            speedSmooth_ = speedLimit; // 首次进入限速
        else if (speedLimit < speedSmooth_)
            speedSmooth_ += (speedLimit - speedSmooth_) * decelRate; // 降速：快
        else
            speedSmooth_ += (speedLimit - speedSmooth_) * accelRate; // 升速：慢

        speedLimit = speedSmooth_;
    }
    else
    {
        // 无曲率：逐步放开限速（同样只允许缓慢升速）
        if (speedSmooth_ > 0.0)
        {
            speedSmooth_ += (speedFloor - speedSmooth_) * accelRate;
            if (speedSmooth_ <= speedFloor + 1e-3)
                speedSmooth_ = -1.0; // 完全放开
            else
                speedLimit = speedSmooth_;
        }
    }

    // ------------------------------------------------------------------
    // [6] 输出到 params->ctrl（供 Motion 直接读取，避免 Motion 持有本类引用）
    // ------------------------------------------------------------------

    // 前馈舵机增量：几何精确纯追踪为 atan(tan α) ≈ tan α（小角近似），
    // 直接以 tan α 作为"归一化转角指令"，乘 ffGain 后线性映射到 PWM 增量。
    // 映射系数 FFPWM_SCALE 把 tan α = 1（45°）对应到满量程 ~400 PWM，
    // 实测时只需调 ffGain 一个旋钮。
    constexpr double FFPWM_SCALE = 400.0;
    if (valid)
    {
        double ff = previewTan * ffGain * FFPWM_SCALE;
        if (ff > ffLimit)
            ff = ffLimit;
        else if (ff < -ffLimit)
            ff = -ffLimit;
        params->ctrl.steerFF = ff;
    }

    params->ctrl.speedLimit = speedLimit; // 0 = 不限速
}
