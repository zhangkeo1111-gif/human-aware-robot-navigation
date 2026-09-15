# NavWareSet-style synthetic social-navigation environment

## Existing Demo Preservation

本任务通过 `--navwareset-scene` 增加独立场景，不复制工程。原 `demo_scene.py`、`robot_backend.py`、`tracker.py`、`yolo_worker.py` 未修改；`stage_probe.py` 只增加显式选择分支和新场景专用记录。`walking_actor.py` 增加可选第三人及新场景角色选择，原 1/2 人逻辑保留。

不覆盖 `outputs/stage7_final_demo_run1`、`outputs/stage7_final_demo_run2`、原 FINAL 报告或视频。新场景不加载/重建 `indoor_demo_scene.usd`，只生成 `navwareset_style_scene.usd`。原启动命令仍使用 `--demo-scene --demo-controller`。physical A300 与 JetBot 后端未改动。

保护检查：`demo_scene.py`、`indoor_demo_scene.usd`、`robot_backend.py`、`tracker.py`、`yolo_worker.py` 的本轮修改前后 SHA256 一致（仅这五个受保护小文件校验，未进行全项目大规模 hash）。原最终视频仍为 930,440 bytes，mtime 未变。旧 Demo 现在优先引用已有 USD，不再每次启动重新写入该受保护资产。

旧 Demo 短回归 `outputs/stage7_existing_demo_regression`：5 s / 300 steps，50 RGB frames，48/48 YOLO 响应有检测且有 Depth；两人场景成功运行，**SMOKE PASS**。短回归没有要求跑到目标，`route_done=false` 属预期；不把这次短回归冒充新的全流程验收。physical/JetBot 本轮仅检查后端与未选中分支未修改，未重跑物理试验。

## NavWareSet Inspiration

参考 [NavWareSet 官方实验说明](https://anr-navware.github.io/navwareset/) 的长形室内空间、侧向开口、预设社会导航交互和外部观察站。当前为自行设计的合成环境，**not an exact reconstruction**。不称为原 Scene01/28 的几何复现，不使用原数据集轨迹回放。

## Scene

- 主空间：X ∈ [-1, 9.7] m，Y ∈ [-1.95, 1.95] m，即 **10.7 × 3.9 m**。
- 北侧支路：X ∈ [3.8, 6.0] m，Y ∈ [1.95, 4.8] m。支路面积在主空间尺寸之外。
- 2.6 m 高外围墙，开口两侧墙形成真实渲染遮挡；地板采用可烘焙导航网格的 Mesh。
- 仅一个柜体与一个箱体，放在主路线外。没有 SLAM，也没有静态障碍物规划器。
- `navwareset_style_scene.usd` 是独立环境资产；机器人和角色由运行脚本组合，不是已录制轨迹的整套回放 USD。

## Platform and Perception

The A300 platform is simulated using an idealized planar skid-steer kinematic model.

保留官方 A300 资产、碰撞外形、视觉轮子及现有速度/加速度限制。没有新增轮胎物理调参。

- RGB-D：640×360；相对底盘 [0.37, 0, 0.50] m；HFOV 约 106°；clipping 0.05–100 m。
- YOLO11n COCO person；conf=0.25；imgsz=640。未训练、未改变 detector。
- 现有 robust depth/backprojection 和 KF；控制器仅读取感知轨迹与机器人自身位姿。
- 主物理/动画步长 1/60 s，RGB-D 采样 10 Hz；不跳过 walking animation 更新来制造吞吐提升。
- 5 completed YOLO responses/s 指**墙钟时间**吞吐，不是 10 Hz 的仿真时间采样率。

## GRS-style Camera

固定位置 [-0.7, -1.65, 2.17] m，朝向 [5.0, 0.15, 0.65] m，向下约 14.3°。记录 960×540、10 fps，广角焦距 9 mm / horizontal aperture 24 mm；仅扩展外部相机覆盖，机器人相机不变。

GRS 视角并非精确复制官方站点，也不保证观察到墙后的所有人。`enable_grs_lidar=false`；未添加 RTX LiDAR。视频采样间隔为 0.1 仿真秒，因此视频时长不是运行墙钟耗时。

## Scenarios and Motion Presets

所有场景机器人从 [0,0] 出发，途经 [4,0]，目标 [8.5,0]，到达半径 0.3 m，终点后停车 2 s。默认最多 55 s 仿真时间 / 180 s 墙钟测量时间。

角色速度设定统一为 0.65 m/s，seed=17。路径转弯/起步/终点制动时实际速度可以低于该值。下面列出当前 presets；早期试运行以自身 `meta.json` 为准。

实际成功加载并记录的三种官方角色：`male_adult_medical_01`、`male_adult_construction_03`、`male_adult_construction_01`。单人场景固定 medical；三人场景中后两人为不同 construction 角色。

| Scenario | People | Start → preset waypoints (XY m) | Initial delay |
|---|---:|---|---:|
| Frontal | 1 | (6,0) → (3,0) → (3,-1.4) → (0.3,-1.4) | 0 s |
| Obstruction | 1 | (3,0) → (3,-1.4) → (1,-1.4) | 8 s |
| Blind corner | 1 | (4.8,4.1) → (4.8,-1.3) → (1,-1.3) | 7 s |
| Perpendicular | 1 | (4.8,1.5) → (4.8,-1.3) → (1,-1.3) | 8 s |
| Circular A | 3 total | (5,1.5) → (5,-1.3) → (1,-1.3) | 5 s |
| Circular B | — | (6,-1.3) → (4.5,0.6) → (4.8,3.8) | 3 s |
| Circular C | — | (7,0) → (3,0) → (2,1.3) | 0 s |

Circular 指多方向路径在中央区域交叉，不是绕圆轨迹。Frontal 的初始路段真实迎面，随后预设横向离开，避免故意让行人永远停在机器人目标线上。人物不会根据 robot GT 决定让路：仅使用预设时钟及自身 waypoint 到达情况触发下一路段。native agent 负责真实 walking animation 和导航网格路径执行，不使用 cube/capsule 代替人。

## Behavior Modes

**social**：直接实例化原 `DemoController`，只替换 route waypoints，原 SLOW/AVOID/STOP 参数全部保留。STOP 距离 1.55 m，释放距离 1.75 m；AVOID 2.0 m；SLOW 3.0 m。恢复是重新进入 CRUISE，不新增另一套恢复状态机。

注意：通用 `summary.json` 中 `configuration.control` 仍是旧通用控制器配置，不能把其中的门限当成 DemoController 实际门限；此分支实际执行的是未修改的 `demo_scene.py::DemoController.step`。

**non-social**：相同 waypoint、速度上限和前端；忽略普通社交避让，保留基于估计轨迹的 1.15 m emergency stop。该阈值是合成对照的明确设定，不是数据集原始 teleoperation。终点 STOP 不计作 interaction stop。

## Synthetic Dataset Output

目录：`outputs/navwareset_style/<scenario>/<social|non-social>/run_NN/`，自动递增，不覆盖旧 run。

- `robot_and_participants.csv`：每步仿真时间、robot XY/yaw、每人 GT XY、controller state、v/omega 命令。
- `meta.json`：场景、机器人/行为模式、几何尺寸、路线/速度/初始等待、camera/seed、角色路径。
- `occupancy_xy_points.json`：墙及固定物的 XY polygon 顶点、Z 区间，world frame，单位 m。是合成 GT geometry，不是占据概率栅格。
- `detections.json`：同帧 YOLO/Depth/KF、时间戳与关键图引用。
- `position_evaluation.json`：现有 depth surface center 与 actor root 的诊断误差；不能直接解释为人体几何中心定位精度。
- `demo_evaluation.json`、`robot_backend.json`、`summary.json`、`scenario_evaluation.json`、`resources.csv`。
- 关键 `*_rgb.png`、`*_bbox.png` 和 `*_depth.npz`（Depth m 与原曝光时刻/相机参数；不是全帧深度数据集）。
- `NAVWARESET_<SCENARIO>.mp4`：GRS-style 外部真实渲染视频，H.264。

## GT Isolation

人物 GT 仅用于预设行走任务的自身到达判断，以及执行命令**之后**的 recording/evaluation。GT trajectory、occupancy、GRS camera 从不输入 social/non-social 的 `step()`。没有 GT 提前避让。控制器可使用机器人自己的位姿，这是模拟本体定位，不是行人未来信息。

## Evaluation Definitions

Collision distances are geometric evaluation proxies.

- human collision proxy：robot/human XY 中心距离 < robot conservative radius + 0.30 m，阈值约 **0.90278 m**；不是 PhysX contact。
- robot-static clearance：机器人保守圆到墙/固定矩形边界的距离；负值为保守几何相交。
- Resume：interaction 后再次 CRUISE 且实际速度 >0.25 m/s。
- Functional gate：到达目标、有感知/反应/恢复、人物确实移动、无人车圆形重叠且无 robot-static 圆形重叠。场景类型与盲角可见性另行核对，不能单靠此通用 gate。
- Performance gate：completed responses / measurement wall time ≥5。启动、资产加载和最后视频转码不计入该吞吐。
- non-social 不要求出现普通避让或恢复；若全程 CRUISE，Resume 不适用，不能因此判作失败。
- GPU/RAM 统计分别为整张显卡、整机占用，非本进程独占峰值。

## Development Failures Retained

1. 初期 Frontal / Obstruction 使用 18 mm GRS 相机，近处车体被裁切；两者功能完成，但吞吐分别约 4.21 / 3.35 responses/s。保留为 run_01，不作为最终广角视频。
2. 尝试通过 `set_updates_enabled` 每六步启停 GRS render product 时，Isaac/Warp 返回 CUDA illegal-address（error 700）。撤回此优化；blind_corner/social/run_01 和 perpendicular/social/run_01 为无效启动/渲染试验，不当成场景失败或成功。
3. 后续使用普通连续渲染及 headless 录像，减少窗口显示开销，不降低 robot RGB-D，也不改变控制门限。GUI 与 headless 性能分开记录。
4. 为固定角色而指定单个角色目录时，官方 loader 默认寻找子目录，错误地只扫描 textures，造成 0 actor 的 setup failure。新场景专用 loader 已按安装代码接口直接读取该目录中的 USD；旧 Demo loader 不变。相关启动失败保留，补跑使用新 run 目录。
5. 中止旧批处理时，`circular/social/run_01` 在启动输出阶段记录 stdout invalid-argument；没有开始有效测量。正式三人结果在后续 run。
6. 最终 headless 路径一次性禁用**未使用的 UI viewport**更新；机器人和 GRS sensor render products 始终正常启用。这不同于前述失败的按帧启停 sensor 操作。

## Reproduction

在 `D:\detection\robot_human_isaac6`：

```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --navwareset-scene --navwareset-scenario frontal --robot-behavior social --record-demo --label _nav_front
```

其他场景只替换 `--navwareset-scenario`；对照使用 `--robot-behavior non-social`。加 `--headless` 可不打开仿真窗口但继续记录录像。新分支目录由 scenario/behavior/run_NN 决定，`--label` 不参与输出命名，避免覆盖。

旧 Demo 命令仍有效：

```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --demo-scene --demo-controller --label _existing_demo_regression
```

## Limitations

- NavWareSet-inspired, not exact geometry reconstruction；Husky A300 替代原 Jackal/HSR。
- Kinematic A300，无真实轮胎/接触动力学保证。
- 默认无 GRS LiDAR；外部相机存在遮挡。
- 固定前向相机可能丢失近距离/侧向行人，KF 不能补回不可观测真值。
- 原 DemoController 不是静态地图规划器，也不是保证安全的社会导航算法。
- 每个场景少量预设运行，非统计 benchmark，不构成真实机器人安全认证。
- Simulation only，未部署真实 A300。

## Scenario Results — Final Social Runs

以下仅使用修复完成后的 headless 录像运行，未使用启动失败或早期窄视角结果。所有数据来自对应 run 的 `summary.json`、`demo_evaluation.json` 和 `scenario_evaluation.json`。

| Scenario | Selected run | Completed | Min GT distance (m) | Collision proxy | Interaction STOP count / AVOID | Resume | YOLO responses/s | Result |
|---|---|---|---:|---|---|---|---:|---|
| Frontal | social/run_03 | Yes | 1.4488 | No | 0 / Yes | Yes | 6.019 | PASS |
| Obstruction | social/run_03 | Yes | 1.0524 | No | 2 / Yes | Yes | 6.243 | PASS |
| Blind corner | social/run_03 | Yes | 1.1397 | No | 1 / Yes | Yes | 6.056 | PASS |
| Perpendicular | social/run_02 | Yes | 0.9587 | No | 1 / Yes | Yes | 6.548 | PASS |
| Circular | social/run_02 | Yes | 1.2607 | No | 2 / Yes | Yes | 5.769 | PASS |

五组 robot-static conservative disk clearance 的最小值均约 0.3974 m，未出现该代理下的墙/固定物重叠。**横穿场景距人车 collision proxy 阈值仅约 0.056 m，不能解释为充分安全余量。** 本轮未把旧 Demo 的额外 0.2 m margin 当成新场景验收要求，也不声称所有场景满足该额外 margin。

人工检查了五个场景的外部交互画面、横穿/阻挡最近接近画面，以及盲角的遮挡、首次检测和反应 RGB。检查画面未见明显人车模型穿透；这不是逐像素全视频碰撞检测。

实际非静止步行速度中位数：Frontal 0.643、Obstruction 0.617、Blind corner 0.648、Perpendicular 0.643 m/s；Circular 三人分别 0.643 / 0.636 / 0.623 m/s，均为真实 native walking。三人路径长度分别约 7.58 / 6.30 / 7.01 m。

### Blind-corner evidence

- 84 个样本的 camera-to-head 几何线段与静态墙相交；保存的初始 RGB 中看不到人物，控制器保持 CRUISE。
- 首次 YOLO 曝光时刻 **11.0667 s**；首次 SLOW **11.2333 s**；随后 AVOID **11.8000 s**、STOP **12.4667 s**，恢复 CRUISE **18.1667 s**。
- 首次反应晚于首次检测约 0.167 s，没有基于 GT 的提前控制。
- 首次检测画面中人物刚在左边缘局部出现。**首次 YOLO 检测不等于首次可见像素**；84 个遮挡样本也不能解释为整个检测空窗都由墙导致，期间还存在相机视野边界约束。
- [墙体遮挡](D:/detection/robot_human_isaac6/outputs/navwareset_style/blind_corner/social/run_03/detection_0003_bbox.png) · [首次检测](D:/detection/robot_human_isaac6/outputs/navwareset_style/blind_corner/social/run_03/detection_0113_bbox.png) · [避让反应](D:/detection/robot_human_isaac6/outputs/navwareset_style/blind_corner/social/run_03/detection_0119_bbox.png)

### Detection / tracking caveat

三人场景最大同时输出过 4 个 detections/tracks，单人场景也存在 2 个输出的情况。这表明仍有重复检测、误检或短时重复轨迹；本任务没有测定检测 Precision/Recall，也不把成功到达目标等同于感知精度合格。没有为消除这些现象改变冻结前端或 KF。

## Social vs Non-social

两组对应运行的 human presets、robot waypoints、seed、robot camera、GRS camera 和 scene dimensions 均逐字段一致。只切换行为模式；异步感知与渲染仍可能带来少量运行差异。

| Scenario | Behavior / run | Min GT distance (m) | Interaction stop count | Travel time (sim s) | Collision proxy |
|---|---|---:|---:|---:|---|
| Frontal | social/run_03 | 1.4488 | 0 | 29.267 | No |
| Frontal | non-social/run_01 | 1.2959 | 0 | 27.650 | No |
| Obstruction | social/run_03 | 1.0524 | 2 | 36.233 | No |
| Obstruction | non-social/run_01 | 1.2006 | 2 | 31.467 | No |

Frontal social 有 SLOW/AVOID，而 non-social 全程普通 route following，没有 interaction stop；到终点后的停车已排除。Obstruction 两种模式都触发停车，但 social 还包含绕行。

**可支持的结论仅是行为对照确实不同。不能支持 social 在所有场景更安全：本次 Obstruction 的 social 最小距离反而更小。** 不挑选或调参数抹去这一结果；不是论文级统计显著性或算法优越性证明。

对照视频：[Frontal non-social](D:/detection/robot_human_isaac6/outputs/navwareset_style/frontal/non-social/run_01/NAVWARESET_FRONTAL.mp4) · [Obstruction non-social](D:/detection/robot_human_isaac6/outputs/navwareset_style/obstruction/non-social/run_01/NAVWARESET_OBSTRUCTION.mp4)

## Performance

| Scenario / behavior | RTF | YOLO responses/s | GPU peak (MiB, whole GPU) | RAM peak (GiB, system) |
|---|---:|---:|---:|---:|
| Frontal social | 0.605 | 6.019 | 3258 | 11.45 |
| Obstruction social | 0.626 | 6.243 | 3329 | 12.33 |
| Blind corner social | 0.608 | 6.056 | 3384 | 13.49 |
| Perpendicular social | 0.658 | 6.548 | 3366 | 11.07 |
| Circular social | 0.579 | 5.769 | 3451 | 14.16 |
| Frontal non-social | 0.547 | 5.444 | 3234 | 12.70 |
| Obstruction non-social | 0.606 | 6.028 | 3345 | 13.32 |

五组 social 全部达到本任务的 ≥5 responses/s 目标，三人场景无需降低 RGB-D 或录像分辨率。**RTF 仍小于 1，不是墙钟实时 1× 仿真**。不得把编码后视频的正常播放速度当作机器实时处理速度。

## Final Videos and Artifact Validation

| Scenario | Video | Duration | Frames |
|---|---|---:|---:|
| Frontal | [NAVWARESET_FRONTAL.mp4](D:/detection/robot_human_isaac6/outputs/navwareset_style/frontal/social/run_03/NAVWARESET_FRONTAL.mp4) | 31.3 s | 313 |
| Obstruction | [NAVWARESET_OBSTRUCTION.mp4](D:/detection/robot_human_isaac6/outputs/navwareset_style/obstruction/social/run_03/NAVWARESET_OBSTRUCTION.mp4) | 38.3 s | 383 |
| Blind corner | [NAVWARESET_BLIND_CORNER.mp4](D:/detection/robot_human_isaac6/outputs/navwareset_style/blind_corner/social/run_03/NAVWARESET_BLIND_CORNER.mp4) | 36.3 s | 363 |
| Perpendicular | [NAVWARESET_PERPENDICULAR.mp4](D:/detection/robot_human_isaac6/outputs/navwareset_style/perpendicular/social/run_02/NAVWARESET_PERPENDICULAR.mp4) | 36.1 s | 361 |
| Circular | [NAVWARESET_CIRCULAR.mp4](D:/detection/robot_human_isaac6/outputs/navwareset_style/circular/social/run_02/NAVWARESET_CIRCULAR.mp4) | 36.0 s | 360 |

ffprobe 检查五个视频均为 H.264、960×540、10 fps。轨迹时间戳严格递增；曝光时刻均不晚于响应时刻；关键 Depth arrays 为 360×640。五组分别保存 52 / 62 / 58 / 57 / 57 组关键 Depth。每组 `evidence_index.json` 的 detection、interaction、STOP/AVOID、recovery 图像均存在。最终三个 Python 修改文件通过 py_compile。

## Final Status

**NAVWARESET-STYLE SCENE PASS**

含义限于：五类新场景可运行，行人真实步行，感知驱动交互并恢复到目标，本次运行无所定义的人车/静态碰撞代理事件，五组达到吞吐目标；两组 behavior 对照、数据输出、视频和原 Demo smoke 已完成。

不代表高保真车辆动力学、精确复现 NavWareSet、感知无误检、social 优于 non-social、完整安全保证或真实机器人验证。原 FINAL A300 Demo 仍是独立保留的 stable baseline。
