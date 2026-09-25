# A300 integration report

日期：2026-09-11。工程：`D:/detection/robot_human_isaac6`。

## A300 source

使用 [Clearpath 官方 clearpath_common](https://github.com/clearpathrobotics/clearpath_common/tree/jazzy/clearpath_platform_description/urdf/a300)，分支 `jazzy`，实际提交 `893db1fdb37d6e601d85eb53573edafa991a67b9`。这是独立的 **A300** 描述，未使用 A200。

- 主描述：`assets/husky_a300/clearpath_common/clearpath_platform_description/urdf/a300/a300.urdf.xacro`。
- 四轮：同目录 `drivetrain/wheels/outdoor.urdf.xacro`；配置 `diff_4wd`、前后 outdoor wheels。
- 加载官方默认 top plate、前后 bumper；网格来自同一提交的 `meshes/a300/`，没有重画车体。
- `robot.urdf.xacro` 仅调用官方宏。`prepare_asset.py` 用 xacro 2.1.1 展开，`$(find ...)` 解析到本地官方包目录，不安装/运行 ROS 2。禁用平台控制插件，移除不用的 Gazebo 插件声明，保留官方几何、轴、质量和惯量。
- 官方源码未修改；URDF 中 package mesh URI 仅改为现有文件路径。

## Isaac asset

可重复加载：`D:/detection/robot_human_isaac6/assets/husky_a300/husky_a300.usd`。

该入口引用 `converted/husky_a300/husky_a300.usda`，必须与 `converted/` 一起保留。正常启动不会再次导入。

使用当前 Isaac Sim 6.0.1 自带 `URDFImporter / URDFImporterConfig`：floating base、合并固定关节、禁用自碰撞、使用源 collision，不从 visual 重建碰撞。四个 continuous joints 被导入成无角度上下限的 Y 轴 revolute joints；velocity drive、force 类型、stiffness=0。导入 damping=100 N·m·s/rad，USD 的每度表示为 1.7453293；这是仿真驱动设置，不是官方电机辨识值。

## Robot parameters

| 项目 | 实际值及来源 |
|---|---|
| 明确提供的总质量 | 80 kg：chassis 70 kg + 四轮各 2.5 kg |
| 未提供的质量 | 多个固定附件没有独立 inertial；没有补造附件质量。80 kg 不能称为所有实车配置的整备质量 |
| 官方 chassis 参数尺寸 | 0.860 × 0.378 × 0.229559 m |
| 含轮子/保险杠 collision 包络 | 0.995408 × 0.661100 × 0.378229 m，由源几何和全链变换计算 |
| Outdoor wheel radius / width | 0.1651 / 0.1143 m |
| Wheelbase | 0.512 m |
| 官方 wheel_track 属性 | 0.562 m |
| 实际关节中心轮距（本次映射） | **0.5468 m**；左 y=0.2829、右 y=−0.2639 m。保留官方描述的这个差异，没有自行移动轮子 |
| Wheel center x / z | 前后 ±0.256 m；z=0.02913 m，均相对 base_link |
| 轮关节轴 | 全部 `(0,1,0)` |
| 官方轮速限制 | **未提供**：continuous joint 没有 limit 元素 |
| 实际 Isaac DOF maxVelocity | 四轮均 **17453.29296875 rad/s**，为导入器/PhysX 的宽松默认值，**不是 A300 真实电机上限** |

四个关节为 `front_left_wheel_joint`、`front_right_wheel_joint`、`rear_left_wheel_joint`、`rear_right_wheel_joint`。没有沿用 JetBot 的 12.56 rad/s。

Base 约定保持 +x 前、+y 左、+z 上。初始 base z=0.15597 m，落稳后约 0.135970 m。质量及惯量来自官方 inertial，固定关节合并后保留 chassis 和四轮刚体。

## Motion test

数据：`outputs/stage2_a300_motion_contact/robot_backend.json`。每段 4 s，统计各段第 1–4 s；先前未指定轮胎材质的运行也保存在 `stage2_a300_motion/`。

| 测试 | 命令 v (m/s) | 命令 ω (rad/s) | 实际前向速度均值 | 实际 yaw rate 均值 |
|---|---:|---:|---:|---:|
| Forward | 0.30 | 0 | 0.299273 | −0.000015 |
| Reverse | −0.30 | 0 | −0.300575 | −0.000007 |
| Left curve | 0.20 | +0.40 | 0.145438 | +0.022173 |
| Right curve | 0.20 | −0.40 | 0.183646 | −0.027551 |
| Rotation | 0 | +0.40 | −0.005189 | +0.168164 |
| Stop | 0 | 0 | −0.000589 | −0.000077 |

前后与左右符号正确，能实际转动；但弧线转向严重欠响应，**不能判定角速度控制性能正常**。轮速本身接近目标，提示问题不只是命令没有送达。没有通过提高轮速掩盖问题。

轮地接触明确设置 static friction=0.5、dynamic friction=0.4、restitution=0、combine=min，绑定四个官方轮 cylinder collider。这是简单仿真假设，非官方实测轮胎参数。与默认接触试验相比，弧线角速度没有明显改善；尚不能断言摩擦是唯一原因。官方 Gazebo wheel-slip 插件未在 Isaac 中复现。

无倒置、飞起或沉地现象：本轮最大 |roll|/|pitch| 约 0.000210°/0.000217°。最大请求轮速 1.873773 rad/s、最大观测轮速 1.961371 rad/s；命令裁剪行数为 0。

重现：

```powershell
cd D:\detection\robot_human_isaac6
.\runtime\python.bat stage_probe.py --stage 2 --robot-model a300 --motion-test --seconds 90 --label _a300_motion_check
```

## Camera

- 相对 base_link 安装点 `[0.37, 0, 0.50] m`；落稳后光心离地约 0.636 m，在前部顶板上方。
- 相机安装旋转 xyzw=`[0.5,-0.5,-0.5,0.5]`；USD 相机 −Z 朝机器人 +X、+X 朝机器人 −Y、+Y 朝机器人 +Z。
- 每个物理步以 articulation pose 更新光心和朝向；不是静止世界相机。现有距离检查模式保留独立相机位置控制。
- 640×360、RGB + distance_to_image_plane、10 Hz、clipping 0.05–100 m。
- focalLength=9、aperture=24×13.5；HFOV=106.260°、VFOV≈73.740°。没有改 YOLO/FOV。
- 读取到的静止 CameraParams 光心为约 `[0.369524,0.000007,0.635970] m`，光轴近似 `[1,0,0]`；移动时光心随车前进。
- 支架/小相机壳是额外的 display-only sensor representation，无碰撞或附加质量；不是冒充官方 A300 附件。

已检查真实 RGB 和第三人称输出，人物完整可见，没有大面积车体遮挡，保留官方彩色车体和轮子模型。示例图片仅保存在本地运行结果中：`outputs/stage7_a300_stationary/overview.jpg` 和 `outputs/stage7_a300_stationary/detection_0020_bbox.png`；未上传 GitHub。

## Perception

YOLO、人物资产/运动代码、robust depth、KF 状态更新与关联算法未修改。沿用 COCO YOLO11n、imgsz=640、conf=0.25。按曝光时间配对 RGB/Depth，并保存同帧相机参数；异步 worker 通过 frame_id/stamp 对齐。

| 测试 | 仿真时间 | YOLO 响应 | 有检测响应 | 有轨迹响应 | accepted depth 观测 |
|---|---:|---:|---:|---:|---:|
| A300 静止，人物行走 | 8 s | 78 | 78 | 78 | 79 |
| A300 前进，前方人物 | 3 s | 28 | 28 | 28 | 28 |
| A300 前进，横穿人物 | 6 s | 58 | 6 | 6 | 7 |
| A300 横穿刹停 smoke | 10 s | 98 | 50 | 56 | 52 |

观测数可多于响应数，因为一帧可能有多个人框。以上不是 detection recall：分母没有做可见性标注。横穿样本多数时间在视野外，不能将 6/58 解释成算法召回率。

静止测试人物深度范围约 2.436–3.705 m；前方移动测试中的感知表面中心与人物 GT root 的 XY 差异 mean=0.128771 m、P95=0.205157 m。两者语义并不完全相同，该值不是精准人体中心精度声明，GT 仅用于评价。

证据分别为 `outputs/stage7_a300_stationary/`、`stage7_a300_front_moving/`、`stage7_a300_moving/`、`stage7_a300_brake/` 下的 detections/position_evaluation/robot_backend JSON。

## Control

上层仍输出 `(v, omega)`，小型 `robot_backend.py` 完成四轮映射：

```text
left  = (v - omega * 0.5468 / 2) / 0.1651
right = (v + omega * 0.5468 / 2) / 0.1651
front_left = rear_left = left
front_right = rear_right = right
```

按实际 articulation limit 检查并裁剪，日志保留 requested/clipped；一旦裁剪会显式打印 `WHEEL_CLIPPING`。当前真实电机极限未知，不能把仿真默认 limit 用于实机。

刹停 smoke 的状态顺序包含 CRUISE → BRAKE → STOP → CRUISE：约 t=4.77 s 进入 STOP，t=6–8 s 最大平面速度 0.000931 m/s，t=9.47 s 恢复 CRUISE。最小 GT 中心距离 1.140081 m。该短测试验证命令链，不证明通用避障安全；人物仍使用现有 native walking，不改变人物行为。

没有运行 staged lateral escape 或 Head-on×3/Crossing×3。侧移预测中的轮半径、轮距和 limit 已改为 backend 参数读取，未优化侧移算法。

启动本轮简单控制：

```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --scenario crossing --avoidance --robust-depth --sim-seconds 10 --seconds 90 --record-demo --label _a300_control_check
```

## Footprint

旧 JetBot circle radius=0.15 m。A300 根据官方碰撞几何在 base_link 下的完整包络：

```text
x ∈ [-0.49770397, +0.49770397] m
y ∈ [-0.32105,    +0.34005] m
radius = sqrt(max(abs(x))² + max(abs(y))²)
       = 0.602779598 m
```

该圆覆盖包络矩形，是保守 proxy；未缩小半径以通过测试。保留 human radius=0.30 m 和额外余量=0.20 m 后，本轮 collision/safety 中心距离阈值为 **0.902780 / 1.102780 m**，不再是 JetBot 的 0.45/0.65 m。这仍是几何代理，不是 PhysX 接触事件或实机安全认证。

## Performance

统计测量循环，不含启动/资产加载；GPU/RAM 是整机采样峰值，不是模型独占用量。带 record-demo 的运行另有 1280×720 第三人称渲染与编码。

| A300 运行 | physics steps/s | RTF | YOLO responses/s（墙钟） | peak VRAM MiB | peak RAM GiB |
|---|---:|---:|---:|---:|---:|
| 纯运动 | 96.107 | 1.602 | 不适用 | 3060 | 13.531 |
| 静止感知 + 录像 | 34.219 | 0.570 | 5.561 | 3658 | 14.776 |
| 横穿前进 + 录像 | 32.005 | 0.533 | 5.156 | 3656 | 12.905 |
| 横穿刹停 + 录像 | 32.406 | 0.540 | 5.293 | 3658 | 13.017 |
| 前方人物前进，无录像 | 34.589 | 0.576 | 5.380 | 3380 | 13.025 |
| 最终代码短 smoke，无录像 | 37.192 | 0.620 | 5.785 | 3375 | 12.993 |

最终代码 smoke 另保存于 `outputs/stage7_a300_final_smoke/`：28/28 次响应检测到人，有效深度和跟踪链继续工作。导入资产实读质量 80 kg、5 个 collision shapes（合并固定件的车体及四轮），articulation root 为 `/World/A300/Geometry/base_link`。

对应短 JetBot 回归无录像：53.346 steps/s、RTF=0.889、8.298 responses/s、3445 MiB、12.919 GiB。运行不是多次统计性能基准，不能把差异全归因于机器人网格，但 A300 当前墙钟实时能力不足，不能宣称实时 10 Hz YOLO 或 RTF≥1。

独立资产检查包含 USD instance proxies：50 个 mesh、790,958 个 faces，详见 `outputs/a300_geometry_inspection.log`。视觉网格较重，碰撞则只有 5 个 shape；未发现误将高精度轮胎 visual 全部作为碰撞的情况。尚未做简化网格 A/B，不能确认具体渲染耗时占比。早先 summary 的 mesh_faces=0 是普通遍历未进入实例代理的诊断计数错误，不代表没有网格；已修正统计路径，以上独立检查为准。

## Existing system regression

`config.yaml` 默认 `robot_model: a300`；CLI `--robot-model jetbot` 优先覆盖配置，不删除或覆盖原资产。

修改仅涉及 `stage_probe.py` 的加载/安装/诊断入口、小型 `robot_backend.py`、config 的模型选择，以及 tracker 的机器人几何常量读取；`walking_actor.py`、`yolo_worker.py` 和 KF/depth 算法未改。

JetBot 实际加载并完成 24 s 运动回归，前/后均值 +0.330886/−0.339099 m/s，左/右 yaw rate +0.418390/−0.415545 rad/s；读取到原 DOF limit=12.5600004 rad/s。另一次 3 s 感知回归 28/28 响应检测到人，Depth 正常。结果在 `outputs/stage2_jetbot_backend_regression/`、`outputs/stage7_jetbot_perception_regression/`。

```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model jetbot --scenario headon --robust-depth --sim-seconds 3 --seconds 90 --label _jetbot_check
```

Python 编译检查通过。导入程序只需首次或重建资产时执行，不由日常启动调用。

## Remaining issues

1. **Skid-steer 弧线转向欠响应明显**。正负方向正确、能转动，不等于 commanded/actual omega 正常匹配；尚未通过正常转向性能验收。没有引入复杂轮胎模型或通过夸张轮速补偿。
2. 官方该版本未提供轮速/力矩极限，也缺少部分固定附件的独立 inertial；导入器宽松 limit 不是实机参数。
3. A300 完整感知墙钟 RTF<1；尚未做严格配对的性能回归或优化。不能声称已达到实时部署性能。
4. 短刹停 smoke 不是复杂避障验收；A300 新 footprint 下的长期安全性没有验证。

## Final status

**PARTIAL**。

官方 A300 资产、四轮映射、前后运动、RGB-D、YOLO、Depth、Tracking、自状态、刹停/恢复和 JetBot fallback 均已实际接通。不能给出完整 `A300 INTEGRATION PASS`，原因是弧线转向实际响应仍明显不足，尚未满足所有运动验收项。
