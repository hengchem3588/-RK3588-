# icar_autopilot_2026th

2026 AIC 智慧城市·智能服务机器人（ELF 2 / RK3588）自动驾驶工程。

## 功能概览

- 赛道巡线：贝塞尔中心线拟合 + 分段 PD 转向（`src/ctrl/`）
- 丁字口屏蔽：超宽行对称内收，防止中心线被支路吸偏（`junctionShield`）
- 动态速度：按 FSM 模式切换低速/巡线/施工/道闸目标速度
- 元素状态机：起终点停车、避障、停车场、Y 叉识别、道闸等（`src/fsm/`）
- 标志物识别：RKNN 推理 + 预编译 NMS 复用

## 构建（RK3588 板卡上）

```bash
mkdir build && cd build
cmake .. && make -j4
```

## 配置

主要参数集中在 `res/config.json`（速度、PD、丁字口屏蔽开关等），注释见同文件 `#` 前缀键。

## 版本说明

提交消息以日期开头（如 `2026-10-22 ...`），便于按日期回退：

```bash
git log --oneline          # 查看各日期版本
git checkout <commit>      # 回退到某一版查看
```
