# Final A300 Human-Avoidance Demo

日期：2026-09-11。**FINAL A300 DEMO PASS（两次完整功能验收；仅限本演示场景）**。

**The A300 platform is simulated using an idealized planar skid-steer kinematic model.**

**Collision distances are geometric evaluation proxies.**

本结果不推翻历史 physical A300 欠转向结论，不宣称轮胎动力学保真或安全认证。此次没有进行摩擦、damping、solver 或轮胎参数搜索。

## 最终交付与启动

- 场景：[indoor_demo_scene.usd](indoor_demo_scene.usd)
- 最终视频：[A300_HUMAN_AVOIDANCE_DEMO.mp4](outputs/stage7_final_demo_run2/A300_HUMAN_AVOIDANCE_DEMO.mp4)：第二次完整运行，H.264，960×540、10 fps，约 31.2 秒，实际仿真画面；未拼接挑选成功片段。
- 第一轮数据：[run 1](outputs/stage7_final_demo_run1/summary.json)
- 第二轮数据：[run 2](outputs/stage7_final_demo_run2/summary.json)
- 相机关键画面：[减速](outputs/stage7_final_demo_run2/detection_0024_bbox.png)、[停车](outputs/stage7_final_demo_run2/detection_0035_bbox.png)、[近距离观测](outputs/stage7_final_demo_run2/detection_0060_bbox.png)。保留原始 RGB 和标注版，包含 bbox、track ID、估计距离、控制状态。

在 `D:/detection/robot_human_isaac6` 执行：

```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --demo-scene --demo-controller --record-demo --label _final_demo
```

默认上限 40 仿真秒；到达终点后再保持停车 2 秒结束。输出在 `outputs/stage7_final_demo/`，自动生成 H.264 的 `A300_HUMAN_AVOIDANCE_DEMO.mp4`。复跑时更换 label 可保留以前日志。

`config.yaml` 默认 `robot_model=a300`、`motion_mode=kinematic`、`controller_mode=demo`、`record_demo=false`；录像由上面的 flag 启用。室内 Demo 必须显式选择 `--demo-scene`，不会覆盖 Debug Scene。单独的历史 physical 模式使用 `--motion-mode physical`；JetBot 自动保留 physical 模式。

## Robot

保留已导入的官方 Clearpath A300 description / USD。来源为本地 `assets/husky_a300/clearpath_common` 的官方 `clearpathrobotics/clearpath_common` jazzy description；没有重建、缩放或移动原始轮子。

- wheel radius：0.1651 m；实际轮心 track：0.5468 m；wheelbase：0.512 m。
- 原 collision 包围尺寸约 0.9954 × 0.6611 × 0.3782 m；保守 XY 半径 0.60278 m。
- 保留模型原指定质量合计 80 kg、四轮及车体外观、16 个 collision shapes。质量不是运动学积分参数。
- 固定 base Z = 0.13597 m；roll/pitch = 0；+X 前、+Y 左、+Z 上，正 omega 左转。

### 运动模式隔离

`robot_backend.py::KinematicA300` 只在 kinematic 分支实例化。该分支在当前 stage override 中关闭 rigid-body simulation、joint enabled，并移除 articulation-root API；原始资产文件和 physical 分支没有被删除或覆盖。轮子不向底盘施加驱动力，collider 几何保持，但不用于模拟真实动力碰撞响应。

每个 simulation step 使用 `world.get_physics_dt()` 积分：

```text
x += v cos(yaw) dt
y += v sin(yaw) dt
yaw += omega dt
```

速度采用简单限速率：加速 0.5 m/s²、减速 0.8 m/s²、角加速度 1.2 rad/s²。没有 wall-clock 驱动，也没有 omega multiplier。

四轮角度根据实际限速率后的 v/omega 积分：`(v ± omega × track/2)/radius`。轮子沿原 Y 轴做视觉旋转；轮胎接触不决定底盘 yaw。日志中的 12 rad/s 为该可视化适配器的软件界限，不是 OEM 电机辨识值；本 Demo 没有触发它。

## Motion validation

独立验证目录：`outputs/stage2_kinematic_usd_validation/`。日志位置/姿态从 **USD local-to-world transform 读回**，再做时间差分，不仅引用指令速度。每段 4 s，沿用第 1–3.95 s 的统计窗口；瞬态由加速度限制产生。

| v_cmd m/s | omega_cmd rad/s | USD pose-derived v m/s | USD pose-derived omega rad/s | 判定 |
|---:|---:|---:|---:|---|
| +0.30 | 0 | +0.300000 | 0 | PASS |
| −0.30 | 0 | −0.297366 | 0 | PASS，约 0.88% 误差 |
| +0.20 | +0.40 | +0.199959 | +0.400000 | PASS |
| +0.20 | −0.40 | +0.199959 | −0.400000 | PASS |
| 0 | +0.40 | 0 | +0.400000 | PASS |
| 0 | 0 | 0 | 0 | PASS |

非零 v/omega 均满足 <5% 目标。Z 固定，roll/pitch 无漂移，没有物理驱动拉回底盘。没有再次研究 physical yaw。

## Scene

`demo_scene.py::build` 创建独立室内测试区：14 m 长 × 8 m 宽，明亮室内色调，采用简单 primitive 和显式 Mesh 地板。包含后墙、侧墙、门洞、两组桌椅、实验设备方块、柜子和收纳箱；南侧开放便于第三人称观察。桌椅在路线侧边，不挡 nominal route。使用 RTX Real-Time，不下载新第三方装修资产。

使用既有官方 Isaac People 两个人物（medical actor 与另一个 seeded 官方角色），真实 walking animation，不是人物贴图。固定 Randomizer seed 17。

| 人物 | 起点 m | 目标 m | 预设速度 |
|---|---|---|---|
| A：横穿 | (2, −3, 0) | (2, 3, 0) | 0.65 m/s |
| B：偏移迎面 | (7, 1.25, 0) | (−1, 1.25, 0) | 0.65 m/s |

人物到达路线终点后停止，没有根据 robot GT 改变路线。实际记录中 A 的 Y 位移范围约 6.05 m、B 的 X 位移范围约 8.04 m；按 1 s 间隔计算的运动阶段速度中位数约 0.64–0.66 m/s，证实不是原地动画。

机器人 nominal route：(0,0) → (3.5,0) → (7,0)，waypoint 接受半径 0.3 m，无全局规划器。

## Camera / Perception

- Camera 相对 base_link：[0.37,0,0.50] m；640×360 RGB + optical depth；clipping .05–100 m；HFOV 约 106°。
- 相机跟随运动学位姿，每次移动后更新世界 pose；使用同帧 RGB、Depth 和 CameraParams，以及已有 simulation-time 同步。
- YOLO11n，官方 COCO person，conf=.25、imgsz=640；未训练、换模型或调阈值。
- 既有 robust Depth 和 KF tracking 未改。输出位置/速度来自观测和因果预测。
- 独立移动感知 smoke：`outputs/stage7_kinematic_perception/`，20 仿真秒，包含前进、左右转；198 响应均有有效 Depth，59 个响应检测到人物，轨迹链实际工作。人物随转弯离开视野，不能把有框响应比例当作 Recall。

物理积分、原生动画和渲染最终均沿用 60 Hz simulation stepping，机器人相机 tick 为 10 Hz。第三人称录像单独采用 960×540、10 fps，减少录像负担；机器人相机分辨率没有降低。

## Controller

独立 `demo_scene.py::DemoController`，没有启用旧 TURN/TRANSLATE/GUARDED SIDE SWITCH 等研究控制器；旧代码保留。

输入仅为 estimated tracks（位置、速度、confidence、观测时间）和机器人 self pose。使用当前及有限时间预测的位置，忽略超过 0.6 s 未观测的 track；仅考虑 robot-frame `x_forward > 0`、`|y_left| < 1.65 m` 的目标，按最近距离并兼顾接近运动排序。

| 状态 | 逻辑 / 输出 |
|---|---|
| CRUISE | 无前方风险或距离 ≥3 m，v=.30 m/s，按 waypoint heading 转向 |
| SLOW | 距离 <3 m，v=.18 m/s |
| AVOID_LEFT / RIGHT | 距离 <2 m 且 |y|<1.25 m，向人物当前侧的反侧轻转；v=.12 m/s，omega=±.40 rad/s，方向保持至少 .9 s |
| STOP | 距离 <1.55 m 停车；已停车时需距离 ≥1.75 m 才释放；短时失去风险观测保持停车最多 .6 s |

阈值结合 A300 footprint：robot .60278 + human .30 = .90278 m；加 .20 m 参考余量为 1.10278 m。1.55 m 停车触发不是碰撞判定，预留了观测误差和减速度空间。终点也使用 STOP。释放风险后回归 waypoint heading 并逐渐恢复 .30 m/s，不使用复杂规划或预测控制。

注意：现有 summary 的 `configuration.control` 仍保存历史研究控制器参数；本 Demo 实际阈值来自 `demo_scene.py::DemoController`，不能把旧字段中的 1.3 m 当作此次使用的停车阈值。

## Demo run 1 / run 2

两次验收使用相同场景路线、人物速度、感知参数和控制阈值；均为完整独立启动，不从成功片段拼接。

| 检查 | Run 1 | Run 2 |
|---|---:|---:|
| 完整运行 / 无 crash | PASS | PASS |
| 仿真运行时长 | 30.967 s | 31.133 s |
| 到达最后 waypoint | 28.967 s | 29.133 s |
| 最小 estimated risk distance | 0.614 m | 1.001 m |
| 最小 GT evaluation distance | **1.216 m** | **1.201 m** |
| 低于 .90278 m 几何阈值的采样数 | 0 | 0 |
| 低于 1.10278 m 含余量参考值的采样数 | 0 | 0 |
| 减速 / 转向 / 停车 | 均出现 | 均出现 |
| 风险后恢复 CRUISE | 9.40 s | 9.50 s |
| 最终到达并正常停车 | PASS | PASS |
| Depth 可用响应 | 308/308 | 310/310 |
| 有人物检测的响应 | 82 | 83 |
| 有轨迹的响应 | 87 | 88 |
| 接受的 Depth/XYZ observation rows | 138 | 138 |

最小 estimated distance 与最小 GT distance 不一定来自同一个人、同一个时刻。估计距离明显小于 GT 的情况仍存在，尤其近距离/截断下；本任务没有借此重新调整 Depth 或宣称定位精度提升。表中的计数也不构成 detector Precision/Recall。

每轮 `demo_evaluation.json` 逐 simulation step 保存估计风险、命令、机器人位姿、GT 评估距离与终点状态；`detections.json`、`position_evaluation.json` 保存感知证据。关键时刻及终点的真实录像帧已查看，没有明显穿过人物。

## Performance

| 实测指标 | Run 1 | Run 2 |
|---|---:|---:|
| Measurement-loop wall time | 59.298 s | 58.059 s |
| RTF | .5222 | .5362 |
| 完成 YOLO responses/s（墙钟） | **5.194** | **5.339** |
| YOLO mean / P95 | 14.547 / 24.712 ms | 12.753 / 19.858 ms |
| GPU 显存峰值 | 3850 MiB | 3802 MiB |
| 系统 RAM 峰值 | 13.901 GiB | 13.724 GiB |

资源读数为整块 GPU / 全系统，可能包含桌面进程；不是项目独占资源分配。视频按仿真时间以 10 fps 播放；RTF<1 表明这台电脑录制时的实际墙钟运行慢于实时时间，不能称为整套系统达到 real-time RTF=1。YOLO ≥5 responses/s 的本轮目标满足。

## GT Isolation

人物 GT 只出现在角色路线设置及 evaluation/debug 记录中。`DemoController.step(now, tracks, measurement_time, position, yaw)` 不接收 agent、GT 坐标、stage 或仿真查询接口；控制器文件的该类只使用输入的估计 track 和 self pose。GT minimum distance 在生成控制命令后独立计算，未进入方向选择、距离门限或命令生成。

## 直接修复与未通过开发运行

1. 新增运动学适配器，明确关闭物理 drive；修正 USD transform authoring，并增加 USD pose 读回验证。
2. 初版 primitive 地板未给人物形成有效导航面，人物只在原地动画。改为显式 Mesh 地板后，恢复完整 walking route。原运行 `stage7_indoor_demo_dev` 不算成功验收。
3. 曾尝试仅降低 render cadence，发现它改变原生人物相对 simulation time 的运动速度，并产生几何重叠。**已撤销**，`stage7_indoor_demo_mesh` 是失败证据，不用于 PASS。
4. 恢复时间推进后的开发运行最小距离 .958 m，无几何 overlap，但未达到 .20 m 参考余量。将停车触发从 1.35 前移至 1.55 m，加入 1.75 m 释放滞回后，固定配置完成上述两次验收。人物路线没有为这次门限调整而改变。
5. 最终仅降低第三人称录像分辨率；未改 YOLO、Depth、KF、人物速度或 robot camera。

## 保留模式与代码范围

修改：`robot_backend.py`、`stage_probe.py`、`config.yaml`、运动诊断摘要；新增小型 `demo_scene.py` 和生成的 `indoor_demo_scene.usd`。没有复制整个工程。`tracker.py`、`yolo_worker.py`、官方 A300 源资产与既有 Debug Scene 保留。

当前代码下已额外实跑短回归：`stage2_physical_mode_regression` 正确加载 A300 physical，稳定前进约 .29928 m/s；`stage2_jetbot_mode_regression` 正确加载 JetBot physical，稳定前进约 .33087 m/s。它们只是保留模式的加载/运动回归，不是新一轮物理转向验收。Python 编译检查通过。

```powershell
# 原 A300 physical 模式，仍保留原先欠转向，不是本轮修复对象
.\runtime\python.bat stage_probe.py --stage 2 --robot-model a300 --motion-mode physical --seconds 4 --label _physical_check
# JetBot 原 physical fallback
.\runtime\python.bat stage_probe.py --stage 2 --robot-model jetbot --seconds 4 --label _jetbot_check
```

## Limitations / Final Status

- Kinematic chassis；轮子是视觉动画，没有高保真轮胎模型、侧滑动力学或真实碰撞响应。
- 距离为保守圆形几何 proxy，不能覆盖所有姿态、肢体和传感器失效模式。
- 仅两个预设 walking route 的 simulation demo，不是 safety certification，也不是统计泛化 benchmark。
- Static obstacle navigation 未实现；静态家具预先放在 nominal route 外。
- 固定前向相机有视野限制，轨迹会消失或换 ID；没有完整遮挡安全保证。
- 仅仿真，尚未部署到真实 A300。

**FINAL A300 DEMO PASS**：两次完整功能验收均满足真实行走、感知输出、响应风险、无几何重叠、恢复前进与到达终点。不得将这一结论扩展为真实机器人安全性或 tire dynamics fidelity。
