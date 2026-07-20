# GUI-for-openEB 需求分析文档

> 版本：2.2  
> 日期：2026-07-15  
> 技术栈：C++17 + Qt 6  
> 基于：OpenEB SDK v5.2.0（Apache 2.0）  
> 参考：Metavision Studio（Prophesee Docs 5.3.1）、Metavision SDK Pro 产品页  
> v2.1: 同步 v1.0.9 GUI 重构（VSCode 风格侧栏、移除工具栏/Algorithm 菜单、纯英文界面）  
> v2.2: 同步 v1.9.0 发布（代码审查修复、E2VID 默认 fps 调整为 30、事件丢弃率遥测、文档与代码对齐）  
> v2.3: 同步系统性审计修复（doc/systematic_audit.md）：注册表口径改为 36 项（28 自研 + 8 个 OpenEB 滤波/变换阶段），清理无效参数与死代码，注册默认值对齐算法默认值，标定/锐度工具重写

---

## 一、项目概述

### 1.1 项目背景

**OpenEB** 是 Prophesee 发布的开源事件相机 SDK（Apache 2.0 许可证），包含以下开放模块：

| 模块 | 功能 |
|------|------|
| **HAL** | 硬件抽象层，提供相机硬件功能的通用访问接口 |
| **Base** | 基础类和工具函数 |
| **Core** | 通用事件处理模块 |
| **Core ML** | 通用机器学习函数：event_to_video / video_to_event 管线 |
| **Stream** | 用户友好的事件系统交互 API，基于 HAL 模块 |
| **UI** | 屏幕显示和用户/系统事件响应的工具类 |

**Metavision Studio** 是 Prophesee 推出的桌面端 GUI 应用，归属于 **Metavision SDK Pro**（付费商用），**不包含在 OpenEB 中**。OpenEB 目前仅提供功能较基础的 **Metavision Viewer** 作为替代。

此外，Metavision SDK Pro 还提供了大量 OpenEB 不具备的高级算法模块（如目标检测推理、光流推理、3D 跟踪、标定工具、振动/颗粒监测、机器学习训练管线等），这些模块同样不包含在 OpenEB 中。

### 1.2 项目目标

1. **GUI 应用**（`gui/`）：使用 **C++ + Qt 6** 开发功能对齐 Metavision Studio 的图形化操作界面，专注于可视化、配置和录制体验。`gui/` 代码仅负责用户交互和显示渲染，不包含算法实现。

2. **算法模块**（`algo/`）：统一存放**自研**事件处理算法——从基础的噪声过滤和光流，到原属 Metavision SDK Pro 的高级计算机视觉、分析以及标定功能。全部以 C++ 实现以保证吞吐量（≥10 Mev/s）。此外，openEB SDK 自带的算法/工具（见 1.5 节）通过 `gui/algo_bridge/` 直接封装复用，不重复实现。

### 1.3 项目目录结构

```
GUI-for-openEB/
├── openeb/                      # openEB 子树（Apache 2.0，v5.2.0）
├── gui/                         # GUI 应用代码（C++ + Qt 6）
│   ├── CMakeLists.txt
│   ├── main.cpp                 # 应用入口
│   ├── main_window.h / .cpp     # 主窗口（CustomTitleBar + 左侧栏 + 右侧 AlgoWindow）
│   ├── widgets/                 # 通用 GUI 控件
│   │   ├── custom_title_bar.h / .cpp    # 自定义标题栏（替代 QMenuBar，下拉菜单）
│   │   ├── activity_bar.h / .cpp        # VSCode 风格 48px 图标列（§5.6.5）
│   │   └── algo_window.h / .cpp      🆕 算法显示窗口（§5.6.6，仅输出；参数统一在侧栏调节）
│   ├── panels/                  # 设置面板（VSCode 风格侧栏分组）
│   │   ├── abstract_panel.h / .cpp      # 面板抽象基类（相机生命周期解耦）
│   │   ├── settings_panel.h / .cpp      # 侧栏容器（ActivityBar + QStackedWidget）
│   │   ├── devices_panel.h / .cpp       # 相机连接/设备管理
│   │   ├── information_panel.h / .cpp   # 相机信息显示
│   │   ├── display_panel.h / .cpp       # 显示模式/帧率/色彩
│   │   ├── statistics_panel.h / .cpp    # 事件率/吞吐统计
│   │   ├── biases_panel.h / .cpp        # Bias 参数调节
│   │   ├── roi_panel.h / .cpp           # 多矩形 ROI（参考 jAER MouseROI）+ 硬件控制
│   │   ├── esp_panel.h / .cpp           # ESP 配置
│   │   ├── trigger_panel.h / .cpp       # 触发配置
│   │   ├── preprocessing_panel.h / .cpp # OpenEB 预处理链（Polarity/Flip/ROI Filter 等）
│   │   ├── algorithms_panel.h / .cpp    # 自研算法选择 + 共享预处理（Noise/Downsample）
│   │   └── file_tools_panel.h / .cpp    # 录制/导出（从原 File 菜单迁入）
│   ├── display/                 # 事件显示渲染（OpenGL 加速）
│   │   ├── event_display_widget.h / .cpp
│   │   ├── display_strategy.h / .cpp    # 🆕 IDisplayStrategy 4 策略（Passive/Overlay/Replace/Standalone）
│   │   ├── frame_annotator.h / .cpp     # 🆕 帧标注叠加（bbox/ID/轨迹/箭头）
│   │   └── space_time_display.h / .cpp  # 🆕 XYT 3D 事件点云窗口（VBO+GLSL，参考 SpaceTimeRollingEventDisplayMethod）
│   ├── app/                     # 应用层控制器（原 stats/ 合并到此）
│   │   ├── camera_controller.h / .cpp   # 相机生命周期管理
│   │   ├── frame_pipeline.h / .cpp      # 帧管线（事件→帧→显示）
│   │   ├── file_frame_generator.h / .cpp # 文件源帧生成
│   │   ├── file_converter.h / .cpp      # RAW/HDF5 格式转换
│   │   ├── icon_provider.h / .cpp       # SVG 图标缓存（QHash + 主题适配）
│   │   ├── theme_controller.h / .cpp    # 主题控制器（LightGray/LightBlue × Light/Dark/FollowSystem）
│   │   └── statistics_controller.h / .cpp # 统计数据采集
│   ├── calibration/             # 标定向导 + 工具
│   │   ├── calibration_wizard.h / .cpp  # 内参标定向导（20Hz 闪烁棋盘 + 1ms/50ms 滑窗自动抓拍，worker 线程检测/标定）
│   │   ├── chessboard_display.h / .cpp  # 全屏闪烁棋盘（内嵌 HUD：进度/状态/Start-Stop，S/F/Esc 快捷键）
│   │   ├── calibration_event_tap.h / .cpp # 50ms 缓冲 + 100µs 细桶滑窗取 1ms 事件最多窗口
│   │   ├── sharpness_dialog.h / .cpp   # 锐度工具（事件流计数图，对比度/线宽双指标实时曲线）
│   │   └── sharpness_metrics.h / .cpp  # 锐度度量纯函数（孤立像素去噪、σ²/μ²、平均线宽，可单测）
│   ├── recorder/               # 录制/回放控制
│   ├── exporter/               # 数据导出
│   ├── config/                 # 配置序列化（JSON）+ 布局管理
│   ├── resources/              # Qt 资源（编译进二进制）
│   │   ├── theme/              #   主题令牌（tokens.h）+ QSS 模板（base.qss.in）
│   │   ├── icons/              #   Lucide 风格 SVG 图标集
│   │   ├── theme.qrc
│   │   └── icons.qrc
│   ├── tests/                  # 🆕 GUI 单元测试（GTest + CTest）
│   │   ├── CMakeLists.txt
│   │   ├── test_algo_bridge.cpp
│   │   ├── test_config_manager.cpp
│   │   ├── test_display_strategy.cpp
│   │   ├── test_layout_manager.cpp
│   │   ├── test_sharpness_metrics.cpp
│   │   └── test_theme_tokens.cpp
│   └── algo_bridge/            # 算法桥接（调用 algo/ 模块 + openEB SDK）
│       ├── algo_bridge.h / .cpp
│       ├── algo_backend.h               # AlgoBackend 基类 + AlgoResult 结构 + RoiFilter/ProcessRegion helper
│       ├── filter_chain.h / .cpp   🆕 自研算法链式装配（顺序/叠加）
│       └── backends/               # 🆕 按类别拆分的后端注册（§5.6.6）
│           ├── backend_factory.cpp          # 注册入口
│           ├── backend_common.h             # 共享 helper（RoiFilter/Preprocessor/ProcessRegion）
│           ├── backend_registry.h           # 按类别工厂函数声明
│           ├── cv_backends.cpp              # 自研 CV 算法
│           ├── cv_vector_backends.cpp       # CV 向量/轨迹类算法
│           ├── analytics_backends.cpp       # 自研 analytics 算法
│           ├── analytics_extra_backends.cpp # analytics 扩展（E2VID 等）
│           ├── display_backends.cpp         # 显示类算法
│           └── filter_backends.cpp          # 自研 noise/hot_pixel filter
├── algo/                        # 算法模块（C++17，被 gui/ 调用）
│   ├── CMakeLists.txt
│   ├── common/                  # 公共工具
│   │   ├── event.h                       🆕 EventCD POD 包装 + 极性/坐标访问
│   │   ├── event_packet.h                🆕 事件包（span 风格，零拷贝视图）
│   │   ├── event_buffer.h / .cpp         # 环形缓冲区，无锁读写
│   │   ├── lifo_event_buffer.h           🆕 LIFO 事件栈（jAER AEStack 式回溯）
│   │   ├── frame_generator.h / .cpp      # 帧生成器（多窗口累积）
│   │   ├── dvs_framer.h                  🆕 DVS 分帧器（仅 ON/OFF 计数）
│   │   ├── freme.h                       🆕 FREME 模板（jAER Frequency Representation）
│   │   ├── data_loader.h / .cpp          # HDF5/RAW 数据加载
│   │   ├── event_rate_estimator.h        🆕 事件率估计（IIR 平滑）
│   │   ├── performance_meter.h           🆕 性能剖析（FPS/延迟/事件率）
│   │   ├── time_limiter.h                🆕 时间限制器（防止单帧超时）
│   │   ├── kalman_filter.h               🆕 Kalman 滤波（2D 位置/速度）
│   │   ├── kmeans.h                      🆕 KMeans 聚类（事件颜色量化）
│   │   ├── particle_filter.h             🆕 粒子滤波（参考 jAER ParticleFilter）
│   │   ├── periodic_spline.h             🆕 周期样条（轨迹平滑）
│   │   ├── histogram_ring_buffer.h        🆕 环形直方图（ISI/方向统计）
│   │   ├── lif_integrator.h              🆕 LIF 神经元积分器（聚类膜电位）
│   │   └── filter/                       🆕 IIR 滤波器集
│   │       ├── lowpass.h / highpass.h / bandpass.h
│   │       ├── angular_lowpass.h        # 角度低通（参考 AngleLowPassFilter）
│   │       └── median_lowpass.h          # 中位数低通（去脉冲噪声）
│   ├── cv/                      # 计算机视觉与运动分析（全部 header-only）
│   │   ├── noise_filter.h                 # 🆕 多模式（BAF/STCF/Refractory/DWF/AgePolarity/Harmonic/Repetitious/SpatialBP）— v1.0.9 起从独立算法改为共享预处理
│   │   ├── hot_pixel_filter.h             🆕 热像素过滤（学习+查表+FPN 概率校正）
│   │   ├── orientation_filter.h           🆕 4 朝向边缘检测（参考 AbstractOrientationFilter）
│   │   ├── direction_selective_filter.h         🆕 8 方向运动估计（参考 AbstractDirectionSelectiveFilter）
│   │   ├── sparse_optical_flow.h          # 🆕 四模式（LocalPlanes/LucasKanade/BlockMatch/ClusterOF）
│   │   ├── blob_detector.h                # 🆕 连通域 + Histogram2DFilter 背景建模
│   │   ├── object_tracker.h               # 🆕 四模式（RCT/Median/Kalman/MultiHypothesis）
│   │   ├── corner_detector.h              # 🆕 三模式（EndStopped/TypeCoincidence/Harris）
│   │   ├── cluster_interface.h            # 🆕 聚类抽象基类（参考 ClusterInterface）
│   │   ├── cluster_path_point.h           # 🆕 聚类轨迹点（位置/速度/时间戳）
│   │   ├── line_segment_detector.h        # 🆕 ELiSeD 线段检测（参考 ELiSeD）
│   │   ├── hough_line_tracker.h           # 🆕 霍夫直线跟踪（✅ 移植自 jAER HoughLineTracker）
│   │   ├── hough_circle_tracker.h         # 🆕 霍夫圆跟踪（✅ 移植自 jAER HoughCircleTracker）
│   │   ├── orientation_cluster.h          # 🆕 方向共识过滤（朝向一致聚类）
│   │   ├── cluster_lif.h                 # 🆕 LIF 神经元网格聚类（参考 ClusterBubbles）
│   │   ├── background_mask_filter.h        # 🆕 背景掩码学习（运动/静止分离）
│   │   ├── trigger_synced_filter.h        # 🆕 触发同步过滤（仅保留触发窗口事件）
│   │   ├── bandpass_filter.h              # 🆕 IIR 带通滤波（频率域事件过滤）
│   │   ├── optical_gyro.h                 # 🆕 电子稳定 EIS（事件陀螺仪，参考 OpticalFlowGyroTracker）
│   │   ├── ultra_slow_motion.h            # 超高速等效回放（时间膨胀）
│   │   ├── xyt_visualizer.h               # XYT 3D 事件点云数据层（替代 Temporal Plot）
│   │   ├── time_surface.h                # Time Surface 窗口
│   │   └── overlay.h                     # 算法结果可视化叠加
│   ├── analytics/              # 分析模块（全部 header-only）
│   │   ├── active_marker.h                 # 主动标记跟踪（调制光方案）
│   │   ├── event_to_video.h                # 事件→灰度重建（3 模式：BardowVariational/InteractingMaps/E2VID）
│   │   ├── e2vid/                         # 🆕 E2VID DL 管线子模块（移植自 rpg_e2vid）
│   │   │   ├── e2vid_inference.h          #   ONNX Runtime 推理 + 启发式回退 + UNetRecurrent 状态管理
│   │   │   ├── event_voxel_grid.h         #   事件→体素网格（双线性时间插值 + 热像素掩码 + 归一化）
│   │   │   ├── intensity_rescaler.h       #   强度重缩放（auto-HDR 滑动窗口中值滤波）
│   │   │   └── unsharp_mask.h             #   Unsharp Mask 锐化 + 双边滤波去噪
│   │   ├── flow_statistics.h               🆕 光流评估（PE / EPE / 端点误差）
│   │   ├── isi_analyzer.h                  🆕 ISI 直方图分析（像素间事件间隔分布）
│   │   ├── particle_counter.h              🆕 通用颗粒计数器（参考 jAER ParticleCounter）
│   │   ├── auto_bias_controller.h          🆕 自适应 Bias 控制（事件率反馈 PID 调节，借鉴 AutoExposureController 思路）
│   │   ├── freq_detector.h                🆕 闪烁光源频率检测（参考 freq_analyzer 热图+FFT）
│   │   └── sensor_self_test.h            🆕 Sensor 自检（坏点检测 + 不应期估计，O(1)/事件）
│   ├── calibration/            # 相机标定
│   │   └── intrinsic.h / .cpp              # 内参标定（含事件去畸变 LUT）
│   └── tests/                  # 🆕 自研测试基建
│       ├── CMakeLists.txt                   # GTest + CTest 构建配置
│       ├── test_phase6_common.cpp           # ✅ Phase 6 公共工具单元测试
│       ├── test_phase7_cv.cpp               # ✅ Phase 7 CV 模块单元测试
│       ├── test_phase8_10.cpp               # ✅ Phase 8-10 analytics + E2VID 单元测试
│       └── test_raw_algos.cpp               # ✅ 真实 RAW 数据回归测试
├── third_party/                 # 第三方依赖（ONNX Runtime 等）
├── LICENSE
├── README.md
├── README_CN.md
└── doc/
    ├── design.md                # 本文档（系统设计规格说明）
    ├── compile.md               # 编译指南（Ubuntu 26.04 + GCC 15 + Qt 6.6+）
    └── gui_optimization.md      # GUI 优化文档（VSCode 风格侧栏、BUG 修复记录）
```

> 图例：🆕 = 基于 jAER 研究借鉴/新增的模块；🔄 = algo_bridge 直接调用 openEB 已有能力（见 §4.3）；无标注 = 原有模块。jAER 算法对照表见 1.6 节，每个模块的详细方案见第四章。

### 1.4 适用范围

- **gui/**：相机控制、事件可视化（OpenGL 渲染）、数据录制/回放/导出、配置管理（JSON）、ESP、Trigger
- **algo/cv/**（自研）：噪声过滤、光流估计、团块检测、事件级跟踪、角点检测、线段/霍夫检测、朝向/方向滤波、超高速回放、XYT 3D 事件点云、可视化叠加
- **algo/analytics/**（自研）：主动标记跟踪（滑动窗口聚类）、事件→灰度图像重建（2 种非 DL + DL 可选）、光流评估、ISI 直方图、颗粒计数、闪烁频率检测、自适应 Bias、Sensor 自检（坏点 + 不应期）
- **algo/calibration/**（自研）：单相机内参标定（含事件去畸变 LUT）
- **gui/algo_bridge/**（封装复用）：openEB 内置能力中，8 个事件滤波/变换阶段（ROI/极性过滤/极性取反/翻转/旋转/转置/缩放，见 1.5 节）经 `FilterChain` 在 Preprocessing 面板暴露并保留注册项；原 frame_* / preproc_* / util_* 注册项及其后端已删除（无任何 GUI 入口，属不可达死代码）
- **明确不在范围内**：ML 训练管线、ML 检测/分类/光流推理（Detection/Gesture/OpticalFlow Inference）、CV3D 边缘跟踪、企业级监测（颗粒/振动/飞溅）、双目立体视觉、DAVIS APS/IMU 依赖模块

> 注：`algo/analytics/event_to_video` 提供三种重建路径——两种非 DL 路径（InteractingMaps 松弛迭代、BardowVariational 变分优化，均无外部依赖）与 DL 路径（E2VID 神经网络，可选）。全部纯事件 (x,y,p,t) 输入，详见 4.4.2。DL 路径已从 [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) 移植（ONNX Runtime 后端 + 启发式回退）。

**关键约束**：openEB SDK 自身已提供大量内置算法和工具（见 1.5 节），GUI 必须将所有 openEB 内置能力暴露给用户——从界面中选择、启用、调整参数并观察效果。不得因为自研算法而忽略或隐藏 openEB 已有的功能。

### 1.5 openEB 内置能力清单

以下列出 openEB SDK v5.2.0 已提供的全部算法、处理器和工具（来源于 `openeb/sdk/modules/{core,stream,ui,base}/`）。GUI 必须为每个**用户可配置**的能力提供选择/启停/参数配置界面；标注"基础设施"的类为底层支撑，无需直接暴露给终端用户调参。

#### 1.5.1 Core 算法模块

| # | 算法类 | 功能描述 | 关键参数与合法范围 |
|---|--------|----------|----------|
| 1 | `RoiFilterAlgorithm` | 按矩形 ROI 过滤事件，仅保留区域内事件 | `x0,y0,x1,y1`：int，`0 ≤ x0 < x1 ≤ width-1`，`0 ≤ y0 < y1 ≤ height-1`；`output_relative_coordinates`：bool |
| 2 | `RoiMaskAlgorithm` | 基于像素掩码过滤事件 | 掩码图像：HxW 的 uint8 图，0=屏蔽 |
| 3 | `PolarityFilterAlgorithm` | 按极性过滤事件（仅 ON / 仅 OFF） | `polarity`：int，取值 `0`(OFF) 或 `1`(ON) |
| 4 | `PolarityInverterAlgorithm` | 反转事件极性（ON↔OFF） | 无参数 |
| 5 | `FlipXAlgorithm` | 水平翻转事件坐标 | 由传感器宽度自动确定，无需用户输入 |
| 6 | `FlipYAlgorithm` | 垂直翻转事件坐标 | 由传感器高度自动确定 |
| 7 | `RotateEventsAlgorithm` | 旋转事件坐标 | `rotation`：float，弧度；GUI 提供 0°/90°/180°/270° 预设按钮（自动换算弧度），亦支持自定义 `[-2π, 2π]` |
| 8 | `TransposeEventsAlgorithm` | 转置事件坐标（行列互换） | 无参数 |
| 9 | `EventRescalerAlgorithm` | 缩放事件坐标 | `scale_width, scale_height`：float，范围 `(0, 10]`（1.0=原尺寸） |
| 10 | `EventsIntegrationAlgorithm` | 将事件时间积分/累积为帧 | `decay_time`：timestamp(μs)，范围 `[10000, 10000000]`，默认 `1000000` |
| 11 | `EventFrameDiffGenerationAlgorithm` | 生成事件帧的帧间差分图 | `accumulation_time_us`：timestamp，范围 `[1000, 1000000]`，默认 `33000` |
| 12 | `EventFrameHistoGenerationAlgorithm` | 生成事件累积直方图帧 | `accumulation_time_us`：同上 |
| 13 | `TimeDecayFrameGenerationAlgorithm` | 基于指数时间衰减生成帧 | `exponential_decay_time_us`：timestamp，范围 `[10000, 10000000]`，默认 `100000`；`palette`：色彩枚举 |
| 14 | `ContrastMapGenerationAlgorithm` | 通过 ON-OFF 差分生成对比度图 | `accumulation_time_us`：同上 |
| 15 | `PeriodicFrameGenerationAlgorithm` | 按固定周期生成帧（时钟驱动） | `period_us`：timestamp，范围 `[1000, 1000000]`（1ms–1s），默认 `33000` |
| 16 | `OnDemandFrameGenerationAlgorithm` | 按需生成帧（事件驱动/手动触发） | 触发策略：枚举（N_EVENTS/N_US/MIXED） |
| 17 | `AdaptiveRateEventsSplitterAlgorithm` | 自适应速率分割事件流 | `thr_var_per_event`：float，范围 `[1e-5, 1e-2]`，默认 `5e-4`；`downsampling_factor`：int，范围 `[1, 8]`，默认 `2` |
| 18 | `EventBufferReslicerAlgorithm` | 将事件缓冲重新切片 | `delta_ts`：timestamp(μs) ≥ `1000`；`delta_n_events`：size_t ≥ `1`；condition 枚举（IDENTITY/N_EVENTS/N_US/MIXED） |
| 19 | `SharedCdEventsBufferProducerAlgorithm` | 共享缓冲区事件生产者（多消费者） | `pool_size`：int ≥ `4`，默认 `16` |
| 20 | `StreamLoggerAlgorithm` | 记录/回放事件流日志 | 输出路径：有效文件路径 |
| — | `BaseFrameGenerationAlgorithm` | *基础设施*：帧生成基类，可继承扩展自定义帧生成 | 继承扩展 |
| — | `AsyncAlgorithm` | *基础设施*：异步算法运行基类 | 线程数 ≥ `1` |
| — | `GenericProducerAlgorithm` / `SharedEventsBufferProducerAlgorithm` | *基础设施*：通用事件生产者基类 | 缓冲策略 |

#### 1.5.2 Core 事件张量预处理器（用于 ML 输入）

| # | 处理器类 | 功能描述 | 关键参数与合法范围 |
|---|----------|----------|----------|
| 23 | `DiffProcessor` | 事件帧差分预处理 | `accumulation_time_us`：timestamp，范围 `[1000, 1000000]`，默认 `33000` |
| 24 | `HistoProcessor` | 事件直方图预处理（2D 直方图） | `max_events_per_pixel`：int，范围 `[1, 255]`；`accumulation_time_us`：同上 |
| 25 | `HardwareDiffProcessor` | 硬件加速差分预处理 | `accumulation_time_us`：同上（需硬件支持） |
| 26 | `HardwareHistoProcessor` | 硬件加速直方图预处理 | 同上（需硬件支持） |
| 27 | `TimeSurfaceProcessor` | Time Surface 编码（每像素最近事件时间衰减） | `channels`：int，取值 `1`(合并极性) 或 `2`(分离极性)；宽高由传感器确定 |
| 28 | `EventCubeProcessor` | 3D 事件体素网格（C×H×W） | `num_bins`：int，范围 `[2, 20]`，默认 `10`；`accumulation_time_us`：同上 |
| 29 | `EventPreprocessorFactory` | 通过 JSON 配置自动创建预处理器 | 配置文件路径：有效 JSON 文件 |

#### 1.5.3 Core 工具类

| # | 工具类 | 功能描述 |
|---|--------|----------|
| 30 | `CdFrameGenerator` | 便捷生成 CD 事件累积帧 |
| 31 | `FrameComposer` | 将多个帧拼合为单帧（如叠加光流） |
| 32 | `RollingEventBuffer` | 滑动窗口事件缓冲 |
| 33 | `RateEstimator` | 实时事件率估计 |
| 34 | `RawEventFrameConverter` | 将原始事件帧转换为可视化帧 |
| 35 | `VideoWriter` / `CvVideoRecorder` | 将帧序列录制为 AVI/MP4 视频 |
| 36 | `Colors` / `CvColorMap` | 预定义色彩映射和 OpenCV 色彩图 |
| 37 | `DataSynchronizerFromTriggers` | 通过外部触发同步多路数据 |
| 38 | `CounterMap` | 多维计数器映射 |
| 39 | `SimilarityMetrics` | 相似性度量工具 |
| 40 | `MostRecentTimestampBuffer` | 每像素最近时间戳缓冲 |
| 41 | `TimingProfiler` | 性能剖析计时器 |

#### 1.5.4 Stream 模块 API（相机控制与数据流）

| # | 类 | 功能描述 |
|---|-----|----------|
| 42 | `Camera` | 实时相机连接与控制 |
| 43 | `CameraGeneration` | 相机代际/型号识别 |
| 44 | `EventFileReader` / `RawEventFileReader` | 读取 RAW/HDF5/DAT 事件文件 |
| 45 | `EventFileWriter` / `RawEvt2EventFileWriter` | 写入事件文件 |
| 46 | `Hdf5EventFileReader` / `Hdf5EventFileWriter` | HDF5 格式读写 |
| 47 | `DatEventFileReader` | DAT 格式读取 |
| 48 | `CameraStreamSlicer` | 将相机流按时间/事件数切片 |
| 49 | `SliceIterator` | 切片迭代器 |
| 50 | `SyncedCameraSystemBuilder` | 多相机同步系统构建 |
| 51 | `SyncedCameraSystemFactory` | 同步系统工厂 |
| 52 | `SyncedCameraStreamsSlicer` | 多相机流同步切片 |
| 53 | `FrameDiff` | 事件帧差分 |
| 54 | `FrameHisto` | 事件直方图帧 |
| 55 | `Cd` (CD events) | CD 事件回调接口 |
| 56 | `ErcCounter` | ERC 事件计数器 |
| 57 | `ExtTrigger` | 外部触发接口 |
| 58 | `Monitoring` | 相机监控（温度/功耗等） |
| 59 | `OfflineStreamingControl` | 离线文件流控制 |
| 60 | `FileConfigHints` | 文件配置提示 |
| 61 | `RawEventFileLogger` | 原始事件文件日志 |
| 62 | `CameraException` / `CameraErrorCode` | 异常与错误码 |

#### 1.5.5 UI 模块（*基础设施*，本项目 GUI 基于 Qt 6 而非 openEB UI，此处仅作记录）

| # | 类 | 功能描述 |
|---|-----|----------|
| 63 | `Window` / `MTWindow` | OpenGL 窗口框架（openEB 原生，本项目不使用） |
| 64 | `EventLoop` | UI 事件循环（openEB 原生，本项目用 Qt 事件循环） |
| 65 | `UIEvent` / `UIEvents` | 键盘/鼠标事件（openEB 原生） |

> 说明：openEB 的 UI 模块基于 GLFW/OpenGL，本项目 GUI 采用 Qt 6 自行实现窗口与事件系统，不依赖 openEB UI 模块。

#### 1.5.6 Base 模块

| # | 类 | 功能描述 |
|---|-----|----------|
| 66 | `EventCD` | CD 事件数据结构 |
| 67 | `EventExtTrigger` | 外部触发事件 |
| 68 | `EventErcCounter` | ERC 计数事件 |
| 69 | `EventPointCloud` | 点云事件 |
| 70 | `EventMonitoring` | 监控事件 |
| 71 | `RawEventFrameDiff` / `RawEventFrameHisto` | 原始帧类型 |
| 72 | `Event2D` | 2D 事件基类 |
| 73 | `Timestamp` | 时间戳工具 |
| 74 | `SoftwareInfo` | 软件版本信息 |
| 75 | `Log` / `SdkLog` | 日志系统 |

#### 1.5.7 已有示例应用（GUI 需包裹并增强）

| # | 示例 | 功能 |
|---|------|------|
| 76 | `metavision_viewer` | 事件可视化、ROI、ERC、Bias 保存/加载 |
| 77 | `metavision_file_to_hdf5` | RAW→HDF5 转换 |
| 78 | `metavision_file_to_csv` | RAW→CSV 转换 |
| 79 | `metavision_file_to_dat` | RAW→DAT 转换 |
| 80 | `metavision_file_cutter` | 文件裁剪 |
| 81 | `metavision_raw_evt_encoder` | RAW 编码 |
| 82 | `metavision_file_info` | 文件元信息 |
| 83 | `metavision_camera_stream_slicer` | 相机流切片 |
| 84 | `metavision_hal_showcase` | HAL 全功能展示 |
| 85 | `metavision_active_pixel_detection` | 活跃像素检测 |
| 86 | `metavision_hal_raw_cutter` | HAL 层裁剪 |
| 87 | `metavision_hal_ls` | 设备列表 |
| 88 | `metavision_platform_info` | 平台信息 |
| 89 | `metavision_synced_camera_streams_slicer` | 多相机同步切片 |

### 1.6 jAER 算法源码清单与借鉴对照表

> 本节系统列出 [`ref/jaer`](file:///home/justin/GUI_for_openEB/GUI-for-openEB/ref/jaer) 源码中事件相机相关算法文件，并对照本项目 `algo/` 自研模块，标注借鉴决策。jAER 是 iniLabs/Prophesee 早期开源 Java 实现，含大量经典事件相机算法（虽然部分已过时，但算法思路具有直接借鉴价值）。

#### 1.6.1 噪声过滤算法源码（`net/sf/jaer/eventprocessing/filter/`）

| jAER 类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|----------|----------|----------------|
| `AbstractNoiseFilter` | 降噪器抽象基类，定义 `correlationTimeS`(25ms)/`subsampleBy`/`filterHotPixels` 等共享参数 + `NoiseFilterControl` 自适应相关时间 | ✅ 借鉴基类设计 | `algo/cv/noise_filter.h` |
| `BackgroundActivityFilter` | 经典 BA 滤波（Delbruck 2008）：3×3 邻域在过去 `dt` 内有事件则放行 | ✅ 移植 | `algo/cv/noise_filter.h` (BAF 模式) |
| `SpatioTemporalCorrelationFilter` | STCF（Guo & Delbruck 2022 T-PAMI）：3×3 邻域至少 N 个相关事件，可选极性匹配 + 散粒噪声检测 | ✅ 移植为主力 | `algo/cv/noise_filter.h` (STCF 模式) |
| `OrderNBackgroundActivityFilter` | O(W+H) 低内存 BA（Khodamoradi 2018） | ✅ 可选移植 | `algo/cv/noise_filter.h` (低内存模式) |
| `DoubleWindowFilter` | 双窗口 FIFO（信号窗+噪声窗），内存与分辨率无关 | ✅ 移植 | `algo/cv/noise_filter.h` (DWF 模式) |
| `RefractoryFilter` | 逐像素不应期滤波，同像素间隔 > 阈值才放行 | ✅ 移植 | `algo/cv/noise_filter.h` (Refractory 模式) |
| `AgePolarityDenoiser` | 软评分版 STCF（age=1-dt/tau），≥ 阈值放行 | ✅ 移植 | `algo/cv/noise_filter.h` (AgePolarity 模式) |
| `MedianDtFilter` | 3×3 邻域 dt 排序后取前 N 个求和，对噪声鲁棒 | ✅ 可选移植 | `algo/cv/noise_filter.h` (MedianDt 模式) |
| `MultiEventAgePolarityDenoiser` | 每像素维护 N 个历史事件，长上下文评分 | ⚠️ 可选（内存高） | — |
| `LinearCorrelationDenoiser` | 方向直方图 + 线性年龄 + 负熵组合 | ⚠️ 高级模式 | — |
| `DensityFilter` | 5×5 邻域事件密度统计 | ⚠️ 与 STCF 重叠 | — |
| `HarmonicFilter` | 全局谐波振荡器抑制 50/60 Hz 灯光闪烁 | ✅ 移植 | `algo/cv/noise_filter.h` (Harmonic 模式) |
| `RepetitiousFilter` | 滤除周期性重复事件（屏幕闪烁等） | ✅ 可选移植 | `algo/cv/noise_filter.h` (Repetitious 模式) |
| `SpatialBandpassFilter` | 空间带通（中心/外围时间戳图抑制），小目标增强 | ✅ 可选移植 | `algo/cv/noise_filter.h` (SpatialBP 模式) |
| `SubSamplingBandpassFilter` | 子采样带通（更高效） | ✅ 可选移植 | 同上 |
| `ProbabalisticPassageFilter` | 概率下采样（负载削减） | ⚠️ 工具，可选 | — |
| `ProbFPNCorrectionFilter` | 概率 FPN 校正（逐像素 ISI 自适应通过率） | ✅ 移植 | `algo/cv/hot_pixel_filter.h` (FPN 模式) |
| `AdaptiveInstantaneousSpikeRateNoiseFilter` | 自适应瞬时脉冲率（按场景活动率调阈值） | ⚠️ 仅借鉴思路 | — |
| `EventRateEstimator` / `TypedEventRateEstimator` | 事件率估计（含按类型） | ✅ 移植 | `algo/common/event_rate_estimator.h` |
| `ISIHistogrammer` | ISI 直方图分析 | ✅ 移植 | `algo/analytics/isi_analyzer.h` |
| `CellStatsProber` | 鼠标框选区域率/ISI/频率直方图 | ⚠️ 已移除（`gui/widgets/pixel_probe.h` 从未实例化，已删除） | — |
| `Oscilloscope` | 实时事件示波器（触发+捕获+回放） | ⚠️ 可选 | — |
| `NoiseTesterFilter` + `SignalNoiseEvent/Packet` | 降噪评测框架（注入泊松+漏噪声，统计 TP/FP/TN/FN） | ⚠️ 已移除（移植的 `algo/tests/noise_tester.h`/`signal_noise_event.h` 从未被任何测试使用，已删除） | — |

#### 1.6.2 跟踪算法源码（`net/sf/jaer/eventprocessing/tracking/`）

| jAER 类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|----------|----------|----------------|
| `ClusterInterface` / `ClusterTrackerInterface` | 聚类/跟踪器抽象接口 | ✅ 移植 | `algo/cv/cluster_interface.h` |
| `ClusterPathPoint` | 轨迹点（位置+时间+速度+视差+半径） | ✅ 移植为 POD struct | `algo/cv/cluster_path_point.h` |
| `BasicCluster` | 朴素质心聚类（教学基线） | ⚠️ 仅参考 | — |
| `RectangularClusterTracker` ★ | jAER 旗舰紧凑物体跟踪器（位置/速度/半径/角度 IIR 更新，质量衰减，可见性判定，合并/剪枝） | ✅ 移植为核心 | `algo/cv/object_tracker.h` (RCT 模式) |
| `RectangularClusterTrackerWithUDPClusterOutput` | RCT + UDP 输出（x,y,主机时间戳） | ✅ 借鉴输出模式 | `algo_bridge` 网络输出 |
| `MedianTracker` ★ | 单目标中位数跟踪（鲁棒于离群事件） | ✅ 移植 | `algo/cv/object_tracker.h` (Median 模式) |
| `ParticleTracker` | 粒子滤波多目标跟踪 | ⚠️ 实现老旧，仅参考 | — |
| `KalmanFilter` | 每聚类绑定 4 维 KF（x,y,vx,vy）+ HashMap 管理 | ✅ 移植架构 | `algo/cv/object_tracker.h` (Kalman 模式) |
| `HoughCircleTracker` | 增量霍夫圆变换（瞳孔/球类） | ✅ 移植自 jAER（3D 累加器 a,b,r + 指数时间衰减 + 局部极大值 + NMS + 最近邻关联） | `algo/cv/hough_circle_tracker.h` |
| `HoughLineTracker` ★ | 增量霍夫直线跟踪（巡线/车道线） | ✅ 移植自 jAER（2D 累加器 ρ,θ + 指数时间衰减 + 局部极大值 + NMS + 最近邻关联） | `algo/cv/hough_line_tracker.h` |
| `LineDetector` | 直线检测器接口 (rho, theta) | ✅ 借鉴 | 同上 |
| `OpticalGyro` | 基于 RCT 的全局平移/旋转估计（EIS） | ✅ 移植 | `algo/cv/optical_gyro.h` |
| `ClusterBasedOpticalFlow` ★ | 基于 RCT 速度场的网格化光流（带通滤波） | ✅ 移植 | `algo/cv/sparse_optical_flow.h` (ClusterOF 模式) |
| `TargetLabeler` ★ | 鼠标标注工具（监督学习数据集构建） | ⚠️ 已移除（`gui/widgets/target_labeler.h` 从未实例化，已删除） | — |
| `LIFOEventBuffer` | LIFO 环形事件缓冲（最近优先遍历） | ✅ 移植为通用工具 | `algo/common/lifo_event_buffer.h` |
| `FlyingBlobGenerator` | 合成飞行物体事件（测试用） | ⚠️ 借鉴思路 | — |

#### 1.6.3 朝向/方向标签算法源码（`net/sf/jaer/eventprocessing/label/`）

| jAER 类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|----------|----------|----------------|
| `AbstractOrientationFilter` ★ | 4 朝向边缘检测（时空相关性 + WTA/多输出 + 朝向历史） | ✅ 移植为朝向基类 | `algo/cv/orientation_filter.h` |
| `SimpleOrientationFilter` | 稳定版 4 朝向滤波（兼容 DVS/DAVIS/Binocular） | ✅ 移植 | 同上 |
| `AbstractDirectionSelectiveFilter` ★ | 8 方向运动估计 + 全局平移/旋转/膨胀 | ✅ 移植 | `algo/cv/direction_selective_filter.h` |
| `EndStoppedOrientationLabeler` | 端止细胞仿真，检测线段端点/角点 | ✅ 移植 | `algo/cv/corner_detector.h` (EndStopped 模式) |
| `TypeCoincidenceFilter` | 正交朝向巧合角点检测 | ✅ 移植 | `algo/cv/corner_detector.h` (Coincidence 模式) |
| `NearestEventMotionComputer` | 邻域最近事件运动方向（粒子级） | ⚠️ 仅粒子场景 | — |

#### 1.6.4 FREME 体系（`net/sf/jaer/eventprocessing/freme/` 与 `FremeExtractor.java`）

| jAER 类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|----------|----------|----------------|
| `Freme<O>` | 泛型 2D 事件驱动状态图容器 | ✅ 移植为模板 | `algo/common/freme.h` |
| `FremeExtractor` | "事件流→2D 图→RGB→GL 纹理"流水线抽象基类 | ✅ 借鉴架构 | `algo/common/freme.h` |
| `OrientationFreme` | 梯度时间法连续朝向图 | ⚠️ 与 SimpleOrientationFilter 重叠 | — |

#### 1.6.5 项目算法源码（`ch/unizh/ini/jaer/projects/`）

| jAER 项目/类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|--------------|----------|----------|----------------|
| `rbodo/opticalflow/LocalPlanesFlow` ★ | 局部平面拟合光流（Benosman 2013，4 种估计器） | ✅ 移植为稀疏光流主力 | `algo/cv/sparse_optical_flow.h` (LocalPlanes 模式) |
| `rbodo/opticalflow/LucasKanadeFlow` | Lucas-Kanade 事件光流（Benosman 2012，4 种差分估计器） | ✅ 移植 | `algo/cv/sparse_optical_flow.h` (LK 模式) |
| `rbodo/opticalflow/MotionFlowStatistics` | 光流评估（AE/EPE/密度/全局运动） | ✅ 移植 | `algo/analytics/flow_statistics.h` |
| `rbodo/opticalflow/DirectionSelectiveFlow` | 基于方向事件的光流 | ⚠️ 与 1.6.3 重叠 | — |
| `minliu/PatchMatchFlow` ★ | ABMOF 块匹配光流（BMVC2018，多尺度 SAD + 钻石搜索） | ✅ 移植为备选 | `algo/cv/sparse_optical_flow.h` (BlockMatch 模式) |
| `minliu/KMeans` | 通用 KMeans 聚类 | ✅ 移植 | `algo/common/kmeans.h` |
| `minliu/OpenCVFlow` | OpenCV Farneback 稠密光流（事件累积帧间） | ⚠️ 未移植（非 GUI 核心功能） | — |
| `labyrinthkalman/KalmanFilter` ★ | 通用 2D 点目标常加速度 KF（6 维状态） | ✅ 移植为通用 KF | `algo/common/kalman_filter.h` |
| `labyrinthkalman/LabyrinthBallKalmanFilter` | 球跟踪专用 KF（基于前者） | ✅ 借鉴 | 同上 |
| `labyrinthkalman/KalmanEventFilter` | 多假设 KF 池 + Mahalanobis gating | ✅ 移植 | `algo/cv/object_tracker.h` (MultiHypothesis 模式) |
| `labyrinth/LabyrinthBallTracker` | RCT + 速度中值 + 静态球定位 | ⚠️ 借鉴思路 | — |
| `labyrinth/LabyrinthDavisTrackFilter` | APS 帧掩码过滤 DVS 事件（DVS+APS 融合去背景） | ✅ 移植 | `algo/cv/background_mask_filter.h` |
| `elised/ELiSeD` ★ | Sobel 时间戳梯度 + 角度聚类的线段检测（EBCCSP2016） | ✅ 移植 | `algo/cv/line_segment_detector.h` |
| `elised/LineSupport` | 线段支持区（图像矩增量更新） | ✅ 一并移植 | 同上 |
| `raindrops/RaindropCounter` ★ | 团块跟踪 + 物理量换算 + 统计（颗粒/雨滴计数） | ✅ 移植为通用计数器 | `algo/analytics/particle_counter.h` |
| `rccar/PerspecTransform` | 透视 + 镜头畸变校正（LUT 预计算） | ⚠️ 已移除（`algo/cv/perspective_undistort.h` 注册项与后端永不生效，已删除；去畸变由共享预处理 undistort 阶段取代，见 §4.5.3） | — |
| `rccar/OrientationCluster` | 邻域方向共识过滤 | ✅ 移植 | `algo/cv/orientation_cluster.h` |
| `virtualslotcar/Histogram2DFilter` ★ | 2D 事件直方图背景建模 + 掩码学习 | ✅ 移植 | `algo/cv/background_mask_filter.h` |
| `virtualslotcar/PeriodicSpline` | 周期三次样条（闭合轨迹建模） | ✅ 移植 | `algo/common/periodic_spline.h` |
| `virtualslotcar/NearbyTrackEventFilter` | 基于轨迹模型的近邻事件过滤 | ⚠️ 与赛道耦合 | — |
| `ahuber/filter/BandpassEventFilter` | 事件驱动 IIR 带通滤波 | ✅ 可选移植 | `algo/cv/bandpass_filter.h` |
| `ahuber/filter/BandpassIIREventFilter` | 多阶 IIR 带通 | ✅ 可选移植 | 同上 |
| `einsteintunnel/BlurringTunnelFilter` | LIF 神经元网格聚类（替代 RCT 的范式） | ✅ 移植 | `algo/cv/cluster_lif.h` |
| `eyetracker/EyeTracker` | 圆环模型瞳孔跟踪 | ⚠️ 未移植（瞳孔专用） | — |
| `eyetracker/EllipseTracker` | 椭圆霍夫跟踪 | ⚠️ 同上 | — |
| `laser3d/HistogramData` | 通用环形历史直方图 | ✅ 移植 | `algo/common/histogram_ring_buffer.h` |
| `laser3d/FilterSyncedEvents` | 触发同步事件过滤 | ✅ 移植 | `algo/cv/trigger_synced_filter.h` |
| `laser3d/FilterLaserline` | 激光线检测（周期直方图打分） | ⚠️ 激光专用 | — |
| `davis/calibration/SingleCameraCalibration` ★ | OpenCV 标定 + 事件去畸变 LUT | ✅ 移植 LUT 思路 | `algo/calibration/intrinsic.h` (增强) |
| `davis/frames/ApsFrameExtractor` | APS 帧提取（双缓冲） | ✅ 借鉴架构 | `algo/common/frame_generator.h` |
| `tobi/goalie/Goalie` | 反应式跟踪+速度预测拦截 | ⚠️ 借鉴思路 | — |
| `apsdvsfusion/LeakyIntegrateAndFire` + `IntegerDecayModel` | LIF 神经元 + 整数近似指数衰减 | ✅ 移植为通用积分器 | `algo/common/lif_integrator.h` |

#### 1.6.6 芯片/渲染器/显示方法源码

| jAER 类 | 路径 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|------|----------|----------|----------------|
| `DavisChip` | `eu/seebetter/ini/chips/` | DVS/APS/IMU 事件位编码（YSHIFT=22, XSHIFT=12, POLSHIFT=11） | ✅ 借鉴位编码 | `openeb/` 解析层 |
| `HotPixelFilter` ★ | `eu/seebetter/ini/chips/davis/` | 学习模式（统计热像素）+ 过滤模式（地址查表） | ✅ 移植 | `algo/cv/hot_pixel_filter.h` |
| `AEChipRenderer` | `net/sf/jaer/graphics/` | 6 种 ColorMode（GrayLevel/RedGreen/ColorTime/GrayTime/HotCode/WhiteBackground）+ fading/sliding/accumulate 三模式 | ✅ 借鉴枚举 | `gui/display/event_display_widget.h` |
| `DavisRenderer` | `net/sf/jaer/graphics/` | 三 buffer 分离（APS pixBuffer + DVS dvsEventsMap + annotateMap） | ✅ 借鉴架构 | 同上 |
| `DavisColorRenderer` | `eu/seebetter/ini/chips/davis/` | RGBW Bayer demosaic + 3x4 颜色校正矩阵 | ⚠️ 仅彩色 DAVIS | — |
| `AdaptiveIntensityRenderer` ★ | `net/sf/jaer/graphics/` | 用 dt（距上次事件时间）作为灰度 + 校准矩阵 | ✅ 移植 | `gui/display/adaptive_renderer.h` |
| `SpaceTimeRollingEventDisplayMethod` ★ | `net/sf/jaer/graphics/` | 滚动 XYT 3D 显示（VBO + GLSL，时间窗剔除） | ✅ 强烈移植 | `gui/display/space_time_display.h` |
| `SpaceTimeEventDisplayMethod` | `net/sf/jaer/graphics/` | 静态 XYT 3D（cube spike list） | ✅ 已借鉴（坐标轴+标签） | `gui/display/space_time_display.cpp` `draw_axes_overlay()` |
| `TimestampImage3DDisplayMethod` | `net/sf/jaer/graphics/` | 时间戳 3D 显示 | ⚠️ 与 SpaceTime 重叠 | — |
| `ChipRendererDisplayMethodRGBA` | `net/sf/jaer/graphics/` | 三 GL 纹理叠加（APS+DVS+标注） | ✅ 借鉴 | `gui/display/event_display_widget.h` |
| `FrameAnnotater` 接口 | `net/sf/jaer/graphics/` | 算法绘制叠加层抽象 | ✅ 借鉴接口 | `gui/display/frame_annotator.h` |
| `MultilineAnnotationTextRenderer` | `net/sf/jaer/graphics/` | 多行文本渲染 | ⚠️ 已移除（`gui/widgets/multiline_text.h` 无任何引用，已删除） | — |

#### 1.6.7 传感器/Tweak/标定算法源码

| jAER 类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|----------|----------|----------------|
| `DVSTweaks` ★ | 4 高级 bias 抽象（threshold/bandwidth/maxFiringRate/onOffBalance），含 Nozaki & Delbruck 2018 物理量估计 | ✅ 移植接口 | `gui/panels/bias_tweaks_panel.h` |
| `DVSBiasController` | 自适应 bias 控制（状态机+滞回+命令节流，目标 event rate/SNR/噪声） | ✅ 移植 | `algo/analytics/auto_bias_controller.h` |
| `DavisVideoContrastController` | APS 对比度/亮度/gamma（含自动模式） | ✅ 移植 | `gui/display/contrast_controller.h` |
| `BadRetKicker` | 静止时复位像素阵列 | ⚠️ 仅 DVS128 调试 | — |
| `SingleCameraCalibration` ★ | OpenCV 标定 + 事件去畸变 LUT + SwingWorker 后台 | ✅ 移植 LUT | `algo/calibration/intrinsic.h` (增强) |

#### 1.6.8 工具/事件类型/Filter 框架源码

| jAER 类 | 算法原理 | 借鉴决策 | 对应本项目模块 |
|---------|----------|----------|----------------|
| `BasicEvent` / `PolarityEvent` / `TypedEvent` | 事件 POD 基类 | ✅ 借鉴字段 | `algo/common/event.h` |
| `ApsDvsEvent` + `ReadoutType` 枚举 | DVS/APS/IMU/SOF/EOF/SOE/EOE 统一事件 | ✅ 借鉴枚举 | 同上 |
| `OpticalFlowEvent` | 携带 `optFlowVelPPS` 的事件 | ✅ 借鉴字段 | `algo/common/event.h`（纯事件 (x,y,p,t)，光流速度由 `algo/cv/sparse_optical_flow.h` 按需计算） |
| `OrientationEvent` / `DvsOrientationEvent` | 4 朝向事件 | ✅ 借鉴字段 | `algo/common/event.h`（朝向由 `algo/cv/orientation_filter.h` 按需计算） |
| `DvsMotionOrientationEvent` | 8 方向运动事件（含 delay/distance/speed/velocity） | ✅ 借鉴字段 | `algo/common/event.h`（运动方向由 `algo/cv/direction_selective_filter.h` 按需计算） |
| `EventPacket` | 后端数组事件包 + 双迭代器 + filteredOut 计数 | ✅ 借鉴 | `algo/common/event_packet.h` |
| `EventFilter2D` / `FilterChain` | Filter 框架（含 ProcessingMode 双模式 + per-filter 性能剖析） | ✅ 借鉴双模式 | `gui/algo_bridge/filter_chain.h` (增强) |
| `EventFilter2DMouseROI` ★ | 多矩形 ROI + freezeRoi + 持久化 | ✅ 移植（升级 ROI） | `gui/panels/roi_panel.h` (增强) |
| `EventFilter2DMouseAdaptor` | 鼠标→芯片坐标转换 | ⚠️ 已移除（`gui/widgets/mouse_adaptor.h` 从未实例化，已删除） | — |
| `EventProcessingPerformanceMeter` | 每 filter ns/event + eps + 占比 | ✅ 移植 | `algo/common/performance_meter.h` |
| `TimeLimiter` | 单帧处理超时丢弃 | ✅ 移植 | `algo/common/time_limiter.h` |
| `LowpassFilter` / `HighpassFilter` / `BandpassFilter` | 一阶 IIR 滤波器工具 | ✅ 移植为头文件 | `algo/common/filter/lowpass.h` 等 |
| `AngularLowpassFilter` | 角度低通（处理回绕） | ✅ 移植 | `algo/common/filter/angular_lowpass.h` |
| `MedianLowpassFilter` | 滑动中位数滤波 | ✅ 移植 | `algo/common/filter/median_lowpass.h` |
| `ParticleFilter` + `Particle` + `ParticleEvaluator` | 通用粒子滤波框架 | ✅ 移植模板 | `algo/common/particle_filter.h` |
| `DvsFramer` | DVS→帧序列（3 种切片模式） | ✅ 借鉴 | `algo/common/dvs_framer.h` |

---

## 二、系统总体架构

### 2.1 分层架构

```
┌──────────────────────────────────────────────────────┐
│                  GUI Layer (gui/ — C++ / Qt 6)       │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────────┐  │
│  │ 主窗口    │ │ 设置面板  │ │ 分析工具窗口          │  │
│  │(显示区)   │ │(控制区)   │ │(XYT 3D/Algo)         │  │
│  └──────────┘ └──────────┘ └──────────────────────┘  │
├──────────────────────────────────────────────────────┤
│                Application Layer                     │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐        │
│  │Camera  │ │Record/ │ │Export  │ │Config  │        │
│  │Ctrl    │ │Playback│ │Convert │ │Mgr     │        │
│  └────────┘ └────────┘ └────────┘ └────────┘        │
│  ┌──────────────────────────────────────────────┐    │
│  │           algo_bridge (算法桥接层)             │    │
│  │          gui/  ↔  algo/ 接口调用              │    │
│  └──────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────┤
│             Algorithm Layer (algo/ — C++)            │
│  ┌──────┐ ┌──────┐ ┌──────────┐ ┌────────────┐      │
│  │  cv  │ │analytics│ │calibration│ │  common   │      │
│  │22自研│ │ 8自研 │ │  1自研  │ │  20公共   │      │
│  └──────┘ └──────┘ └──────────┘ └────────────┘      │
│  ┌──────┐                                           │
│  │tests│  (GTest 单元测试)                            │
│  └──────┘                                           │
│  (openEB 8 个滤波/变换阶段由 algo_bridge 封装复用)     │
├──────────────────────────────────────────────────────┤
│                OpenEB SDK Layer                      │
│  ┌─────────┐ ┌─────────┐ ┌────────┬ ┌───────────┐   │
│  │ Stream  │ │  HAL    │ │  Core  │ │ Core ML   │   │
│  └─────────┘ └─────────┘ └────────┘ └───────────┘   │
├──────────────────────────────────────────────────────┤
│                Hardware Layer                        │
│  ┌──────────────────────────────────────────────┐    │
│  │ Prophesee 事件相机 (Gen4.1/IMX636/GenX320)    │    │
│  └──────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────┘
```

### 2.2 模块交互关系

```
                        ┌─────────────┐
                        │   主窗口     │
                        │  (MainWin)  │
                        └──────┬──────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
    ┌─────▼──────┐     ┌──────▼──────┐     ┌───────▼──────┐
    │  显示面板   │     │  设置面板    │     │  算法面板     │
    │ (Display)  │     │ (Settings)  │     │ (Algo Panel)  │
    └─────┬──────┘     └──────┬──────┘     └───────┬──────┘
          │                    │                    │
          └────────────────────┼────────────────────┘
                               │
                    ┌──────────▼──────────┐
                    │    Application      │
                    │    Controller       │
                    │  (app_controller)   │
                    └──────────┬──────────┘
                               │
     ┌─────────────┬───────────┼───────────┬─────────────┐
     │             │           │           │             │
┌────▼────┐ ┌──────▼───┐ ┌─────▼────┐ ┌───▼────┐ ┌─────▼────┐
│CameraCtrl│ │Recorder  │ │Exporter │ │Config  │ │algo_bridge│
│(相机控制)│ │(录制回放) │ │(导出)   │ │Mgr     │ │(算法调用) │
└────┬────┘ └──────┬───┘ └─────┬────┘ └───┬────┘ └─────┬────┘
     │             │           │           │             │
     └─────────────┴─────┬─────┴───────────┴─────────────┘
                         │
                    ┌────▼────┐
                    │  algo/  │
                    │ cv /    │
                    │analytics│
                    │/calib   │
                    └────┬────┘
                         │
                    ┌────▼────┐
                    │ OpenEB  │
                    │ Stream/ │
                    │ HAL     │
                    └─────────┘
```

---

## 三、模块详细需求

### 3.1 相机控制模块 (Camera Controller)

**职责**：管理相机生命周期（发现、连接、断开），控制传感器参数。

#### 3.1.1 相机发现与连接

| 功能 | 描述 |
|------|------|
| 设备枚举 | 扫描系统中所有已连接的 Prophesee 兼容相机 |
| 相机选择 | 从已发现相机列表中选择目标相机并建立连接 |
| 连接状态 | 实时显示连接状态（已连接/断开/错误） |
| 自动重连 | 可选：相机意外断开后自动重连 |

#### 3.1.2 传感器 Bias 控制

Bias 是传感器内部参数，控制像素对光照变化的响应特性。不同传感器型号（Gen4.1、IMX636、GenX320）的参数集和取值范围不同，有符号方向也可能不同。

| 参数 | 类别 | 功能描述 |
|------|------|----------|
| **bias_diff_on** | ON 对比度阈值 | 产生 ON 事件（变亮）所需的最小对比度增量。值越大 → 灵敏度越低，ON 事件越少 |
| **bias_diff_off** | OFF 对比度阈值 | 产生 OFF 事件（变暗）所需的最小对比度降低量。方向因传感器而异 |
| **bias_diff** | 基准对比度参考 | 内部参考电平，**强烈建议不要修改** |
| **bias_fo** | 低通滤波器（时域） | 设定像素低通滤波器截止频率，决定多快的光照波动会被衰减。降低截止 → 减少闪烁但增加延迟 |
| **bias_hpf** | 高通滤波器（时域） | 设定像素高通滤波器截止频率，决定多慢的波动会被抑制。扩宽 → 对慢速物体更敏感，但背景噪声增加 |
| **bias_refr** | 不应期（死时间） | 像素产生事件后的盲区时间。缩短 → 更多事件/大信号；延长 → 一步变化只产生一个事件 |
| **bias_pr** | 光电管带宽 | **（Gen4.1/IMX636/GenX320 上已弃用）** 不建议修改 |

**设计要点**：
- 支持传感器型号自动检测，动态加载对应的可用 Bias 列表
- 每个 Bias 显示参数名、当前值、范围、单位（如有）
- 支持滑块 + 精确数值输入两种交互方式
- 支持 reset to default
- 支持实时生效（修改即时下发到传感器）

#### 3.1.3 感兴趣区域（ROI）

| 参数 | 描述 | 取值范围 |
|------|------|----------|
| roi_enabled | 是否启用 ROI | true / false |
| roi_x | 起始 X 坐标（列） | 0 ~ 传感器宽度-1 |
| roi_y | 起始 Y 坐标（行） | 0 ~ 传感器高度-1 |
| roi_width | ROI 宽度 | 1 ~ 传感器宽度 |
| roi_height | ROI 高度 | 1 ~ 传感器高度 |

**设计要点**：在主显示区支持鼠标拖拽绘制 ROI 矩形，并在设置面板同步数值。

#### 3.1.4 数字事件掩码（Digital Event Mask）

| 功能 | 描述 |
|------|------|
| 掩码启用 | 开启/关闭像素掩码 |
| 掩码配置 | 选择特定像素/区域在数字阶段屏蔽事件输出 |

---

### 3.2 可视化与显示模块 (Display)

**职责**：事件流的实时可视化渲染，帧累积显示，色彩配置。

#### 3.2.1 事件帧显示

| 参数 | 描述 | 默认值 |
|------|------|--------|
| accumulation_time_ms | 帧累积时间（ms），控制一帧内累积多少事件 | 33.3（≈30fps） |
| color_theme | 色彩主题（暗色/亮色） | 暗色 |
| background_color | 背景色 | 黑色 |
| on_event_color | ON 事件颜色 | 白色（暗色主题） |
| off_event_color | OFF 事件颜色 | 蓝色（暗色主题） |
| display_mode | 显示模式（累积帧/下采样连续帧） | 累积帧 |

**显示核心理念**：将异步事件流通过时间窗口累积转换为可视化帧——这个转换只影响显示，不影响底层事件数据。

#### 3.2.2 统计信息面板

| 指标 | 描述 |
|------|------|
| 事件率 | 实时事件速率（Kev/s 或 Mev/s） |
| 传感器信息 | 型号、分辨率、序列号 |
| 帧率 | 当前显示帧率（fps） |
| ON/OFF 比例 | ON 事件与 OFF 事件数量比 |

#### 3.2.3 3D 事件点云（XYT 3D Point Cloud）

| 参数 | 描述 |
|------|------|
| time_window_ms | 显示时间窗口（ms），仅显示最近 N ms 的事件 |
| polarity_color | 极性着色模式（ON=红/OFF=绿 / 年龄渐变 蓝→绿→红） |
| point_size | 点大小（px） |
| depth_shade | 是否启用深度遮蔽（近大远小） |
| rotate_speed | 自动旋转速度（可暂停手动拖拽） |

**功能**：在独立窗口中以 3D 点云形式显示事件流——X、Y 为像素坐标，T 为时间轴（深度方向），每个事件为一个彩色点。相比原 Temporal Plot（2D x-t/y-t 散点图）保留完整空间信息，可旋转、缩放、拖拽观察事件时空分布，用于分析运动轨迹、噪声分布、传感器调优。底层由 `gui/display/space_time_display.h`（VBO+GLSL 渲染）实现，算法层由 `algo/cv/xyt_visualizer.h` 提供数据切片与颜色映射。

---

### 3.3 数据录制与回放模块 (Recording & Playback)

**职责**：实时录制、文件回放、录制裁剪。

#### 3.3.1 实时录制

| 参数 | 描述 | 默认值 |
|------|------|--------|
| output_format | 输出格式（RAW） | RAW |
| output_path | 输出文件路径 | 用户指定 |
| max_file_size_mb | 最大文件大小（MB），0=无限制 | 0 |
| record_duration_s | 录制时长（秒），0=持续录制 | 0 |
| auto_split | 到达最大尺寸时自动分片 | false |

**设计要点**：
- 录制按钮（●）与计时器显示
- 录制中禁用可能导致中断的操作
- 支持快捷键（如 R 开始/停止录制）

#### 3.3.2 文件回放

| 参数 | 描述 | 默认值 |
|------|------|--------|
| playback_speed | 回放倍速 | 1.0 |
| loop | 循环播放 | false |
| start_at_s | 起始播放位置（秒） | 0 |
| pause / resume | 暂停/继续 | - |
| seek | 跳转到指定位置 | - |

**设计要点**：
- 播放进度条 + 时间显示
- 支持倍速切换（正常、慢动作、高速）
- 支持帧步进
- 播放中切换文件/相机后新文件自动播放
- EOF 判定以文件加载完成标志为准：加载完成前游标追上已缓冲进度时暂停等待，不误判 EOF
- 打开大文件时给出内存预估提示；事件缓冲硬上限 3 亿事件（约 4.8 GB），超出部分截断并提示

#### 3.3.3 录制裁剪

| 参数 | 描述 |
|------|------|
| start_time_us | 裁剪起始时间（微秒） |
| end_time_us | 裁剪结束时间（微秒） |

**设计要点**：在回放进度条上支持拖拽选择裁剪区间，配合预览。

#### 3.3.4 支持的文件格式

| 格式 | 读取 | 写入 | 描述 |
|------|------|------|------|
| RAW | ✓ | ✓ | Prophesee 原生事件录制格式 |
| HDF5 | ✓ | ✓ | 开放标准 HDF5 事件文件格式 |
| DAT | ✓ | - | 兼容旧版 .dat 事件文件格式 |

---

### 3.4 事件信号处理模块（ESP）

**职责**：配置传感器的事件信号处理（Event Signal Processing）功能。

ESP 滤波器会影响传感器输出的事件流。**如果不启用任何滤波器且录制不受限，可能因事件率过高导致丢帧或数据损坏。**

#### 3.4.1 Anti-Flicker（抗闪烁）

消除人工光源（PWM 调光 LED 等）引起的周期性闪烁事件。

| 参数 | 描述 | 默认值 |
|------|------|--------|
| afk_enabled | 是否启用 | false |
| afk_frequency_hz | 电网频率（Hz） | 50（中国/欧洲）或 60（北美） |

**适用范围**：Gen4.1、IMX636、GenX320 传感器。

#### 3.4.2 Event Trail Filter（事件轨迹滤波器）

用于抑制或整形连续事件序列。

| 参数 | 描述 | 默认值 |
|------|------|--------|
| trail_enabled | 是否启用 | false |
| trail_type | 滤波类型（STC 等） | 依传感器 |
| trail_threshold_us | 时间阈值（微秒） | 依传感器 |

#### 3.4.3 Event Rate Controller（ERC，事件率控制器）

限制总体事件率，防止下游处理被事件峰值淹没。**注意：可能引入水平伪影、影响信号质量。**

| 参数 | 描述 | 默认值 |
|------|------|--------|
| erc_enabled | 是否启用 | false |
| erc_target_rate_evs | 目标事件率（events/sec） | 1000000 |

---

### 3.5 触发接口模块 (Trigger Interfaces)

**职责**：配置相机的 Trigger In / Trigger Out 硬件同步接口。

#### 3.5.1 Trigger In（外部触发输入）

| 参数 | 描述 |
|------|------|
| trigger_in_enabled | 是否启用 |
| trigger_in_mode | 触发模式（上升沿/下降沿/电平） |
| trigger_in_polarity | 极性 |

#### 3.5.2 Trigger Out（触发输出）

| 参数 | 描述 |
|------|------|
| trigger_out_enabled | 是否启用 |
| trigger_out_mode | 输出模式（事件触发/帧触发/周期性） |
| trigger_out_frequency_hz | 周期性触发频率 |
| trigger_out_duty_cycle | 占空比 |

---

### 3.6 数据导出模块 (Exporter)

**职责**：将事件录制文件导出为 HDF5 或 AVI 格式。

| 导出格式 | 参数 |
|----------|------|
| **HDF5** | 压缩级别、分块大小 |
| **AVI** | 帧率（fps）、累积时间（ms）、编码器、分辨率、质量（1-100）、色彩模式 |

| 参数 | 描述 | 默认值 |
|------|------|--------|
| export_fps | 视频帧率 | 30 |
| export_accumulation_ms | 每帧累积时间 | 33.3 |
| export_codec | 编码器（H.264 / MJPEG） | H.264 |
| export_quality | 编码质量（1-100） | 90 |

**设计要点**：
- AVI 导出使用 `process_all_frames=true` 的 `CDFrameGenerator`，不再丢帧（含文件尾部事件）；注意长文件导出时生成器内部缓冲随"读取快于编码"增长，内存占用显著
- 取消导出/转换时删除半成品文件，不残留不完整输出

---

### 3.7 配置序列化模块 (Config Manager)

**职责**：保存和加载相机完整配置（Bias、ROI、ESP、Trigger 等），兼容 JSON 格式（与 Metavision Studio 互通）。

| 功能 | 描述 |
|------|------|
| 保存配置 | 将当前所有相机参数序列化为 JSON 文件 |
| 加载配置 | 从 JSON 文件反序列化并应用到相机 |
| 预设管理 | 内置典型场景的预设配置（如"高灵敏度"、"低噪声"） |
| 自动检测 | 加载配置时校验传感器型号兼容性 |

**配置范围**：
- Bias 参数
- ROI 设置
- ESP 设置（Anti-Flicker, Trail Filter, ERC）
- Trigger 设置

> **注**：数字事件掩码（Digital Event Mask）暂未在 ConfigManager 中实现，后续版本补充。

---

### 3.8 算法桥接模块 (gui/algo_bridge/)

**职责**：`gui/` 中负责调用 `algo/` 中 C++ 算法模块的桥接层。只做参数传递和结果回传，不实现算法逻辑。所有算法实现统一在 `algo/` 目录（详见第四章）。桥接层**实际实例化并调用** `algo/` 中的真实算法类（非 Phase 1 占位 stub）：对自研算法直接构造 `algo/` 中的类实例并驱动其处理接口；对 openEB 内置能力则实例化 `openeb/sdk/modules/` 下的算法类并转发参数。`algo_bridge.h` 头部注释须反映此事实（"实际实例化并调用真实 algo/ 类"），不得保留 "Phase 1 stub" / "pass-through" 字样。

**桥接接口**：

```cpp
// algo_bridge.h — 统一算法调用接口
class AlgoBridge {
public:
    // 查询可用算法列表
    std::vector<AlgoInfo> list_algos() const;

    // 创建算法实例（按名称）
    std::shared_ptr<AlgoInstance> create(const std::string& name);

    // 推送事件到算法实例
    void push_events(const std::shared_ptr<AlgoInstance>& inst,
                     const Metavision::EventCD* begin,
                     const Metavision::EventCD* end);

    // 拉取算法处理结果
    AlgoResult pull_result(const std::shared_ptr<AlgoInstance>& inst);
};
```

---

## 四、algo/ 算法模块详细设计

> **技术栈**：全部 C++17 实现，通过 `gui/algo_bridge/` 被 Qt GUI 调用。  
> **模块来源**：基础算法 + 原 [Metavision SDK5 PRO](https://www.prophesee-cn.com/metavision-sdk-pro/) 中 CV/Analytics/Calibration 的核心模块自主重实现。  
> **参考**：自研算法实现方案只是尝试性的，当前算法设计还有改进空间，后续会持续优化和扩展。

### 4.1 模块总览

| 子目录 | 自研模块数 | 内容 |
|--------|-----------|------|
| **algo/common/** | 20 | 事件 POD/包、环形/LIFO 缓冲、帧生成器与事件分帧器、FREME 模板、数据加载器（HDF5/RAW）、IIR 滤波器集、Kalman/KMeans/粒子滤波、周期样条、环形直方图、LIF 积分器、性能剖析、时间限制器、事件率估计 |
| **algo/cv/** | 22 | 🆕 全部自研：噪声过滤（8 模式）、热像素过滤、4 朝向边缘检测、8 方向运动估计、光流估计（4 模式）、团块检测、目标跟踪（4 模式）、角点检测（3 模式）、ELiSeD 线段、霍夫直线/圆跟踪、方向共识、LIF 神经元聚类、背景掩码、触发同步、带通滤波、电子稳定 EIS、超高速回放、XYT 3D 事件点云、Time Surface 窗口、可视化叠加（详见 4.3；其中注册为算法的是 20 个，noise_filter 为共享预处理、cluster_interface/cluster_path_point 为支撑类） |
| **algo/analytics/** | 8 | 🆕 主动标记跟踪（滑动窗口聚类）、事件→灰度重建（2 种非 DL + DL 可选）、光流评估、ISI 直方图、颗粒计数器、闪烁频率检测、自适应 Bias、Sensor 自检（坏点 + 不应期） |
| **algo/calibration/** | 0 (Tools 向导) | 🆕 单相机内参标定向导（CalibrationWizard：闪烁棋盘 + 1ms/50ms 滑窗自动抓拍），不注册为算法；事件去畸变由共享预处理 undistort 阶段承担（§4.5.3） |
| **algo/tests/** | — | 🆕 GTest 单元测试（test_phase6_common / test_phase7_cv / test_phase8_10 / test_raw_algos） |
| **gui/algo_bridge/** | （封装层） | 🔄 openEB 8 个事件滤波/变换阶段（polarity_filter/polarity_invert/flip_x/flip_y/rotate/transpose/rescale/roi_filter）经 FilterChain 暴露并保留注册项；原 frame_*/preproc_*/util_* 共 20 个注册项与 4 个 openeb_*_backends.cpp 已删除（无 GUI 入口的不可达死代码，见 doc/systematic_audit.md §三-B6/7） |

> **总计**：`AlgoBridge` 注册 28 个自研算法（cv 20 + analytics 8，含 sensor_self_test；intrinsic_calibration 已移至 Tools 菜单向导，perspective_undistort 注册项已删除被预处理 undistort 阶段取代；common/tests 模块为工具库不注册，noise_filter v1.0.9 起改为共享预处理阶段）+ 8 项 openEB 滤波/变换阶段，共 **36 项**可通过 `list_algos()` 枚举。
> **数据约束**：仅支持单相机纯事件流 (x,y,p,t)，不含 DAVIS APS 帧、IMU、双目立体。所有模块均在纯事件输入下工作。
> **算法来源**：噪声过滤、跟踪、光流、朝向、霍夫等核心思路借鉴 jAER（见 1.6 节对照表），全部以 C++17 重写并集成到 openEB 事件流回调中，不引入 Java 依赖。

---

### 4.2 algo/common/ — 公共工具

| 模块 | 功能 | 关键设计 / 借鉴 |
|------|------|----------|
| `event.h` 🆕 | EventCD POD 包装，提供极性/坐标/时间戳访问器 | 零开销抽象，对应 jAER `BasicEvent` |
| `event_packet.h` 🆕 | 事件包（span 风格），零拷贝视图 | `std::span<EventCD>` 包装 |
| `event_buffer.h/.cpp` | 环形缓冲区，无锁读写，支持多线程并发推送/拉取事件 | `std::atomic`, cache-line padding |
| `lifo_event_buffer.h/.cpp` 🆕 | LIFO 事件栈，用于回溯最近 N 个事件 | 借鉴 jAER `AEStack` |
| `frame_generator.h/.cpp` | 将事件流按累积时间窗口生成帧（`cv::Mat`），支持多窗口并行 | 模板化窗口策略 |
| `dvs_framer.h` 🆕 | 事件分帧器，纯事件 ON/OFF 计数累积 → 灰度帧（非 DL 重建基础） | 借鉴 jAER `DvsFramer` |
| `freme.h` 🆕 | FREME 频率表示模板，存储事件频谱 | 借鉴 jAER `Freme`/`FremeExtractor` |
| `data_loader.h/.cpp` | 读取 RAW/HDF5 文件，提供 `EventIterator` 接口 | 内存映射加速大文件 |
| `event_rate_estimator.h` 🆕 | 事件率估计（IIR 平滑，Mev/s） | 借鉴 jAER `EventRateEstimator` |
| `performance_meter.h` 🆕 | 性能剖析（FPS、延迟、事件率、丢帧率） | 借鉴 jAER `PerformanceMeter` |
| `time_limiter.h` 🆕 | 时间限制器，防止单帧渲染超时 | 借鉴 jAER `TimeLimiter` |
| `kalman_filter.h` 🆕 | 2D 位置/速度 Kalman 滤波器 | 借鉴 jAER `KalmanFilter` |
| `kmeans.h` 🆕 | KMeans 聚类（事件颜色量化/轨迹分簇） | 标准 OpenCV 实现 |
| `particle_filter.h` 🆕 | 粒子滤波（蒙特卡洛目标跟踪） | 借鉴 jAER `ParticleFilter` |
| `periodic_spline.h` 🆕 | 周期样条（轨迹平滑/插值） | 借鉴 jAER `PeriodicSpline` |
| `histogram_ring_buffer.h` 🆕 | 环形直方图（ISI/方向/极性统计） | 用于 `isi_analyzer` 等 |
| `lif_integrator.h` 🆕 | LIF 神经元积分器（聚类膜电位更新） | 借鉴 jAER `LIFNeuron` |
| `filter/lowpass.h` `highpass.h` `bandpass.h` 🆕 | 一阶 IIR 低通/高通/带通滤波器 | 借鉴 jAER `LowPassFilter` 等 |
| `filter/angular_lowpass.h` 🆕 | 角度低通（处理 0/2π 边界） | 借鉴 jAER `AngleLowPassFilter` |
| `filter/median_lowpass.h` 🆕 | 中位数低通（去脉冲噪声） | 借鉴 jAER `MedianLowPassFilter` |

### 4.3 algo/cv/ — 计算机视觉与运动分析

> **符号说明**：🔄 = 由 `gui/algo_bridge/` 直接调用 openEB 已有类（`openeb/sdk/modules/core/`），不重复实现，不在 `algo/cv/` 建单独文件；🆕 = openEB 未提供，在 `algo/cv/` 自研实现

#### 4.3.1 🔄 事件过滤与预处理（algo_bridge 封装 openEB Core 算法）

以下 8 个事件滤波/变换阶段由 openEB 完整提供，经 `gui/algo_bridge/filter_chain` 实例化，在侧栏 **Preprocessing 面板**启用/调参，并保留注册项（用于配置捕获与稳定命名空间），无需在 `algo/cv/` 重复实现：

| GUI 选项名 | 调用的 openEB 类 | 参数 |
|-----------|-----------------|------|
| ROI Filter | `RoiFilterAlgorithm` | `x0, y0, x1, y1` |
| Polarity Filter | `PolarityFilterAlgorithm` | `polarity`（ON/OFF） |
| Polarity Invert | `PolarityInverterAlgorithm` | 无 |
| Flip X / Flip Y | `FlipXAlgorithm` / `FlipYAlgorithm` | 传感器尺寸 |
| Rotate | `RotateEventsAlgorithm` | 角度（0/90/180/270） |
| Transpose | `TransposeEventsAlgorithm` | 无 |
| Rescale | `EventRescalerAlgorithm` | `scale_x, scale_y` |

> 注：`RoiMaskAlgorithm` 与 `AdaptiveRateEventsSplitterAlgorithm` 的注册项已删除（无 GUI 入口）。

#### 4.3.2 🔄 帧生成（openEB Frame Generation 算法，内部使用）

> **注册项已删除**：7 个 `frame_*` 注册项与 `openeb_frame_backends.cpp` 已移除（GUI 无帧模式选择入口，属不可达死代码，见 doc/systematic_audit.md §三-B6）。以下 openEB 帧生成能力仍被显示渲染（`CDFrameGenerator`）与 AVI 导出内部使用，此处仅作记录。

| 内部使用者 | 调用的 openEB 类 | 参数 |
|-----------|-----------------|------|
| Integration Frame | `EventsIntegrationAlgorithm` | `accumulation_time_us` |
| Diff Frame | `EventFrameDiffGenerationAlgorithm` | `accumulation_time_us` |
| Histogram Frame | `EventFrameHistoGenerationAlgorithm` | `accumulation_time_us` |
| Time Decay Frame | `TimeDecayFrameGenerationAlgorithm` | `decay_time_us`, `accumulation_time_us` |
| Contrast Map | `ContrastMapGenerationAlgorithm` | `accumulation_time_us` |
| Periodic Frame | `PeriodicFrameGenerationAlgorithm` | `period_us` |
| On-Demand Frame | `OnDemandFrameGenerationAlgorithm` | 触发策略 |

#### 4.3.3 🔄 事件张量预处理器（openEB Preprocessors，内部使用）

> **注册项已删除**：7 个 `preproc_*` 注册项（含 `preproc_hw_diff`/`preproc_hw_histo` 两个连后端工厂分支都没有的空注册）与 `openeb_preproc_backends.cpp` 已移除。`TimeSurfaceProcessor` 仍被 §4.3.27 Time Surface 内部使用，其余仅作记录。

这些是事件→张量的转换器，可输出给模型或作为中间表示：

| 内部使用者 | 调用的 openEB 类 | 参数 |
|-----------|-----------------|------|
| Diff Preprocessor | `DiffProcessor` | `accumulation_time_us` |
| Histo Preprocessor | `HistoProcessor` | `max_events_per_pixel` |
| Time Surface | `TimeSurfaceProcessor` | `decay_time_us` |
| Event Cube | `EventCubeProcessor` | `num_bins` |
| Preprocessor Factory | `EventPreprocessorFactory` | JSON 配置文件路径 |

#### 4.3.4 🔄 工具（openEB Utils，内部使用）

> **注册项已删除**：6 个 `util_*` 注册项与 `openeb_util_backends.cpp` 已移除。`VideoWriter`/`CvVideoRecorder` 仍被 AVI 导出内部使用，其余仅作记录。

| 内部使用者 | 调用的 openEB 类 | 功能 |
|-----------|-----------------|------|
| Rate Estimator | `RateEstimator` | 实时事件率估计（Mev/s） |
| Frame Composer | `FrameComposer` | 多帧叠加合成 |
| Rolling Buffer | `RollingEventBuffer` | 滑动窗口事件缓冲 |
| Video Writer | `VideoWriter` / `CvVideoRecorder` | AVI/MP4 录制 |
| Data Synchronizer | `DataSynchronizerFromTriggers` | 外部触发数据同步 |
| Timing Profiler | `TimingProfiler` | 性能剖析 |

#### 4.3.5 🆕 噪声过滤 (NoiseFilter) — 叠加型

> **v1.0.9 变更**：`noise_filter` 已从独立注册算法移除，改为通过 AlgorithmsPanel 的共享预处理控件暴露。8 种滤波模式（BAF/STCF/Refractory/DWF/AgePolarity/Harmonic/Repetitious/SpatialBP）仍由 `algo/cv/noise_filter.h` 实现，但通过 `preproc_filter_mode` 等参数键配置，作为 "ROI → filter → downsample" 预处理链的一环。Noise filter 与主算法不互斥，可叠加使用。

openEB 未提供专用事件级噪声滤波器，需自研。作用于事件流，过滤后事件送显示/下游算法。借鉴 jAER `net/sf/jaer/eventprocessing/filter/`（见 1.6.1），提供 8 种模式以适配不同场景：

| 模式 | 算法方案 | 借鉴 jAER 类 | 关键参数与合法范围 |
|------|----------|-------------|------|
| **BAF**（背景活动过滤） | Delbruck 2008：当前事件在 3×3 邻域过去 `dt` 内有事件则放行 | `BackgroundActivityFilter` | `baf_dt_us`：int，`[1000, 100000]`，默认 `1000`；`baf_subsample_by`：int，`[0, 4]`，默认 `0` |
| **STCF**（时空相关滤波，主力） | Guo & Delbruck 2022 T-PAMI：3×3 邻域至少 N 个相关事件，可选极性匹配 + 散粒噪声检测 | `SpatioTemporalCorrelationFilter` | `correlation_time_s`：float，`[0.001, 0.1]`，默认 `0.005`；`min_neighbors`：int，`[1, 8]`，默认 `2`；`require_polarity_match`：bool，默认 `false`；`allow_coincidence`：bool，默认 `false` |
| **Refractory**（不应期） | 逐像素不应期：同像素间隔 > 阈值才放行，去除高频热噪 | `RefractoryFilter` | `refractory_us`：int，`[100, 100000]`，默认 `1000` |
| **DWF**（双窗口） | 双 FIFO（信号窗 + 噪声窗），内存与分辨率无关 | `DoubleWindowFilter` | `dwf_window_length`：int，`[1, 1024]`，默认 `2`（上限放宽至 1024，可恢复 jAER 512 工作点）；`dwf_dist_threshold`：int，`[1, 1024]`，默认 `2`；`dwf_min_correlated`：int，`[1, 8]`，默认 `2`；`dwf_double_mode`：bool，默认 `false` |
| **AgePolarity**（软评分） | age = 1 - dt/tau，加权评分 ≥ 阈值放行 | `AgePolarityDenoiser` | `agep_tau_us`：int，`[1000, 100000]`，默认 `3000`；`age_threshold`：float，`[0, 8]`，默认 `2.0`；`agep_radius`：int，`[1, 5]`，默认 `2` |
| **Harmonic**（谐波） | 全局谐波振荡器抑制 50/60 Hz 灯光闪烁 | `HarmonicFilter` | `line_freq_hz`：枚举（50/60），默认 `50`；`notch_q`：float，`[0.1, 100]`，默认 `5.0`；`harmonic_threshold`：float，`[0, 1]`，默认 `0.1` |
| **Repetitious**（重复过滤） | 滤除周期性重复事件（屏幕闪烁等）；ISI 在 `avg/ratio_shorter` 与 `avg*ratio_longer` 之间且 ≥ `min_dt_to_store_us` 才判定为重复 | `RepetitiousFilter` | `rep_ratio_shorter`：int，`[1, 100]`，默认 `10`；`rep_ratio_longer`：int，`[1, 100]`，默认 `10`；`rep_min_dt_to_store_us`：int，`[0, 1000000]`，默认 `1000`（原 `rep_period_us`/`rep_tolerance_us` 为算法端死参数，已从 GUI 移除） |
| **SpatialBP**（空间带通） | 中心/外围时间戳图差分，小目标增强 | `SpatialBandpassFilter` | `sbp_center_radius_px`：int，`[1, 10]`，默认 `2`；`sbp_surround_radius_px`：int，`[5, 30]`，默认 `10`；`sbp_dt_surround_us`：int，`[100, 1000000]`，默认 `10000` |

**通用参数**：`filter_hot_pixels`：bool，默认 `false`（与 4.3.6 联动）；`adaptive_correlation_time`：bool，默认 `false`（借鉴 jAER `NoiseFilterControl`，按事件率自适应调整 `dt`）

**GUI 参数暴露**：全部 8 种模式的专用参数已在 GUI 面板暴露（`algo_bridge.cpp` 注册），按所选 `mode` 调节对应参数即可。

**输出**：过滤后事件流（透传给下游算法/显示），并统计过滤率（filtered/total）。

#### 4.3.6 🆕 热像素过滤 (HotPixelFilter) — 叠加型

借鉴 jAER `HotPixelFilter`（见 1.6.6），学习并标记固定位置的热像素，可选 FPN 概率校正。

| 子模块 | 方案 | 借鉴 |
|--------|------|------|
| 热像素学习 | 学习窗口内在线统计每像素事件数，显著高于全局水平的像素标记为热像素（周期性重学） | `HotPixelFilter` |
| 查表过滤 | 维护 HxW 的 `uint8` 热像素掩码，过滤时直接查表 | — |
| FPN 概率校正 | 借鉴 jAER `ProbFPNCorrectionFilter`：逐像素 ISI 自适应通过率 | `ProbFPNCorrectionFilter` |

**参数与合法范围**：`learning_window_s`：float，`[0.1, 60]`，默认 `5.0`；`enable_fpn_correction`：bool，默认 `false`；`fpn_target_rate_hz`：float，`[1, 1000]`，默认 `100`（原 `n_sigma` 为算法端死参数——头文件自标 "deprecated, unused"，已从 GUI 移除）。启用后主显示改为渲染过滤后的事件流（`filtered_events` 回注重渲染，见 §5.6.1）。

#### 4.3.7 🆕 朝向边缘检测 (OrientationFilter) — 叠加型

✅ 移植自 jAER `SimpleOrientationFilter` / `AbstractOrientationFilter`（见 1.6.3）。采用 jAER **min-dt WTA（Winner-Take-All）方法**：每个事件先更新 `lastTimesMap` 时间表面，然后对 4 朝向（0°/45°/90°/135°）分别沿感受野偏移收集同极性 delta-times，计算每朝向的平均 dt（或最大 dt），选最小 dt 的朝向作为该事件的朝向标签。仅当该朝向 dt 低于 `min_dt_threshold_us` 时才输出。可选 `oriHistory` 时序 IIR 平滑与 `dtRejectThreshold` 离群值剔除。

| 子模块 | 方案 |
|--------|------|
| 朝向计算 | jAER min-dt WTA：4 朝向 RF（半长=3）收集 delta-times → 平均/max dt → WTA 选最小 dt |
| oriHistory 平滑 | per-pixel 朝向历史 IIR 平滑（`oriHistoryMixingFactor` 默认 0.25） |
| 全局朝向向量 | 从芯片中心绘制全局朝向向量（jAER `computeGlobalOriVector`） |
| 朝向渲染 | 4 朝向映射为 4 种颜色（Fixed4 调色板或 HSV），per-event 像素着色 |

**参数与合法范围**：`min_dt_threshold_us`：int，`[1, 1000000]`，默认 `100000`；`use_average_dt`：bool，默认 `true`（平均 vs 最大 dt）；`ori_history_enabled`：bool，默认 `false`；`dt_reject_threshold_us`：int，`[1, 10000000]`，默认 `100000`（原 `tau_us`/`min_neighbors`/`multi_ori_output`/`pass_all_events` 为算法端死参数或未实现，已从 GUI 移除）

**GUI 显示**：per-event 彩色像素（朝向着色）+ 全局朝向向量线（从芯片中心）+ 直方图文字

#### 4.3.8 🆕 方向选择性滤波 (DirectionSelectiveFilter) — 叠加型

✅ 移植自 jAER `AbstractDirectionSelectiveFilter`（见 1.6.3），8 方向运动估计（E/NE/N/NW/W/SW/S/SE）。每个事件在 8 邻域中搜索最近时间戳，最小正 recency（在 `min_dt_us` 与 `time_window_us` 之间）的方向即为运动方向。全局模式下还计算平移/旋转/膨胀运动量（低通滤波）。

| 子模块 | 方案 |
|--------|------|
| 8 方向计算 | 8 邻域时间戳差分，最小 recency 方向 = 运动方向（jAER `computeDirection`） |
| 全局模式 | mass-weighted 平移 + small-angle LS 旋转 + 膨胀（jAER `updateMotion`） |
| 朝向渲染 | 8 方向映射为 8 种颜色（红/橙/黄/绿/青/蓝/紫/白），per-event 像素着色 |
| 运动向量 | 平移箭头（从芯片中心）+ 旋转弧段 + 运动文字 |

**参数与合法范围**：`tau_us`：int，`[1000, 50000]`，默认 `10000`（时间窗口）；`min_dt_us`：int，`[0, 1000000]`，默认 `100`；`search_distance`：int，`[1, 12]`，默认 `3`；`tau_low_ms`：int，`[1, 100000]`，默认 `100`（全局运动低通时间常数）；`enable_global_mode`：bool，默认 `true`

**GUI 显示**：per-event 8 色像素 + 平移箭头 + 旋转弧段 + 直方图文字 + 运动量文字

#### 4.3.9 🆕 光流估计 (OpticalFlow) — 叠加型

openEB 未提供光流算法，需自研。结果以箭头/颜色图叠加到主显示帧。借鉴 jAER `ch/unizh/ini/jaer/projects/rbodo/` 光流套件与 `ClusterBasedOpticalFlow`（见 1.6.5、1.6.2），提供 4 种模式：

| 模式 | 算法方案 | 借鉴 jAER 类 | 参数与合法范围 |
|------|----------|-------------|------|
| **LocalPlanes**（局部平面拟合） | 局部事件 (x,y,t) 时空平面拟合，法向量 = 运动矢量 | `LocalPlanesFlow` (Benoit 2015) | `time_window_us`：int，`[1000, 100000]`，默认 `10000`；`spatial_radius_px`：int，`[3, 30]`，默认 `8`；`min_events_per_cluster`：int，`[3, 100]`，默认 `10` |
| **LucasKanade**（LK 光流） | 时空梯度 + 最小二乘求解 | `LucasKanadeFlow` | `block_size`：int，`[4, 64]`，默认 `16`；`step`：int，`[1, 32]`，默认 `8`；`time_window_us`：同 LocalPlanes |
| **BlockMatch**（ABMOF 块匹配） | 滑动窗口累积张量 + 块匹配，硬件友好 | `PatchMatchFlow` (minliu) | `downsample_factor`：int，`[1, 8]`，默认 `2`；`time_window_us`：`[1000, 200000]`，默认 `20000`；`search_radius_px`：int，`[1, 16]`，默认 `4` |
| **ClusterOF**（聚类光流场） | 跟踪聚类质心轨迹，估计每簇光流；质心 EMA 系数 `cluster_ema_alpha`（对应 jAER RCT `locationMixingFactor`，默认 0.05） | `ClusterBasedOpticalFlow` | 复用 4.3.11 跟踪参数；`cluster_ema_alpha`：float，`[0.001, 1.0]`，默认 `0.05` |

**通用参数**：`search_radius`：int，`[3, 30]`，默认 `4`；`time_window_us`：int，`[1000, 100000]`，默认 `20000`

**输出**：`vector<FlowVector>` — (x, y, vx, vy, confidence)；可选颜色图（HSV 编码方向 + 饱和度编码幅度）

#### 4.3.10 🆕 团块检测 (BlobDetector) — 叠加型

结果以 bbox 叠加到主显示帧。修正原方案（原"RollingEventBuffer + 背景减除"不准确——`RollingEventBuffer` 只是缓冲，不构成背景模型）。借鉴 jAER `Histogram2DFilter`（virtualslotcar 项目）实现真正的运动/静止分离：

| 模式 | 算法方案 | 借鉴 |
|------|----------|------|
| 连通域分析 | openEB 帧 → `cv::threshold` → `cv::connectedComponents` → 过滤面积 | — |
| **Histogram2DFilter 背景建模** | 维护 HxW 的事件计数直方图环形缓冲，长期低活动像素 = 背景；当前帧高活动像素 = 前景 | `Histogram2DFilter` |
| 帧差分 | 相邻累积帧差异 > 阈值即运动像素 | — |

**参数与合法范围**：`accumulation_ms`：float，`[1, 1000]`，默认 `33.3`；`threshold`：int，`[1, 254]`，默认 `50`；`min_area`：int，`[1, 100000]`，默认 `10`；`histogram_window_s`：float，`[0.1, 10]`，默认 `1.0`；`learning_rate`：float，`(0, 1]`，默认 `0.05`

#### 4.3.11 🆕 事件级目标跟踪 (ObjectTracker) — 叠加型

结果以 bbox+ID+轨迹叠加到主显示帧。**修正原方案**（原"DBSCAN+Kalman"不适合事件流——DBSCAN 是离线批处理，事件流需在线增量跟踪）。借鉴 jAER `RectangularClusterTracker`（RCT，Litzenberger 2006）等经典事件流跟踪器，提供 4 种模式：

| 模式 | 算法方案 | 借鉴 jAER 类 |
|------|----------|-------------|
| **RCT**（矩形聚类跟踪，主力） | 每个聚类维护矩形边界框，新事件落入邻近聚类则更新，否则新建；基于热度衰减分裂/合并 | `RectangularClusterTracker` |
| **MedianTracker**（中位数跟踪） | 用中位数（而非均值）更新聚类位置，对离群事件鲁棒 | `MedianTracker` |
| **KalmanTracker**（卡尔曼跟踪） | 每聚类 Kalman 滤波（位置+速度），缺失预测后衰减消失 | `KalmanFilter` (labyrinthkalman) |
| **MultiHypothesis**（多假设跟踪） | 维护多假设树，按似然剪枝，应对遮挡 | `MultiHypothesisTracker` |

| 子模块 | 方案 |
|--------|------|
| 聚类数据结构 | 借鉴 jAER `ClusterInterface`/`ClusterPathPoint`：位置、速度、bbox、轨迹、生命期 |
| 多目标关联 | 邻近聚类合并 / 远距聚类分裂；轨迹管理 + ID 稳定性 |
| **输出** | `vector<TrackedObject>` — ID, bbox, velocity, trajectory, age |

**参数与合法范围**：`cluster_size_px`：int，`[3, 50]`，默认 `10`；`cluster_time_us`：int，`[1000, 50000]`，默认 `5000`；`min_cluster_events`：int，`[10, 500]`，默认 `50`；`max_lost_age_s`：float，`[0.1, 5.0]`，默认 `1.0`；`enable_velocity_prediction`：bool，默认 `true`；`location_mixing_factor`：float，`[0, 1]`，默认 `0.05`（jAER `locationMixingFactor`）；`predictive_velocity_factor`：float，`[0, 10]`，默认 `1.0`（jAER `predictiveVelocityFactor`）；`mass_decay_tau_us`：int，`[1, 1000000]`，默认 `10000`（jAER RCT 默认值）；`threshold_mass_for_visible`：float，`[0, 1000000]`，默认 `10.0`

**GUI 显示**：bbox + ID + 速度箭头（从质心指向预测位置）+ 轨迹折线（jAER `ClusterPath`）+ 速度文字

#### 4.3.12 🆕 角点检测与跟踪 (CornerDetector) — 叠加型

**修正原方案**（原"Harris+SAE"是帧基算法，不适用事件流——SAE 仅辅助时间戳查询，无法直接检测事件角点）。借鉴 jAER `label/` 中的事件原生角点检测器，提供 3 种模式：

| 模式 | 算法方案 | 借鉴 jAER 类 |
|------|----------|-------------|
| **EndStopped**（端止角点） | 双正交方向朝向滤波器组合，检测端止响应（角点/端点） | `EndStoppedOrientationLabeler` |
| **TypeCoincidence**（极性重合） | ON/OFF 极性事件在时空邻域同时出现 = 角点特征 | `TypeCoincidenceFilter` |
| **Harris**（帧基 Harris） | 累积帧 → Harris 角点 → 映射回事件位置（兼容传统方法） | `Harris` |

| 子模块 | 方案 |
|--------|------|
| 角点跟踪 | 最近邻匹配 + `periodic_spline` 平滑 → 轨迹过滤 |
| **输出** | `vector<Corner>` — (x, y, strength, track_id, trajectory) |

**参数与合法范围**：`mode`：枚举（`0=EndStopped` / `1=TypeCoincidence` / `2=Harris`——标签与 `algo/cv/corner_detector.h` 的 Mode 枚举严格一一对应），默认 `0`；`min_score`：float，`[0, 1]`，默认 `0.1`。其余算法端参数（`accumulation_ms`/`track_radius_px`/`min_track_len`/`output_hz` 等）未在 GUI 暴露。

#### 4.3.13 🆕 ELiSeD 线段检测 (LineSegmentDetector) — 叠加型

借鉴 jAER `ELiSeD`（Brandli et al. EBCCSP 2016，见 1.6.5），实时事件级线段检测与跟踪。结果以线段叠加。

| 子模块 | 方案 |
|--------|------|
| 朝向滤波 | 复用 4.3.7 朝向滤波（4 朝向），将事件归类 |
| 线段累积 | 朝向一致的事件沿主方向累积为线段候选 |
| 线段跟踪 | RANSAC 直线拟合 + ID 关联 |

**参数与合法范围**：`min_length`：int，`[3, 100]`，默认 `20`；`gap`：int，`[1, 20]`，默认 `5`（原 `orientation_threshold` 为算法端死参数，已从 GUI 移除）

#### 4.3.14 🆕 霍夫直线跟踪 (HoughLineTracker) — 叠加型

✅ 移植自 jAER `HoughLineTracker`（见 1.6.2）。事件驱动增量霍夫直线变换：维护 2D 累加器 (ρ, θ)，逐事件对每个 θ 累加 ρ = x·cos(θ) + y·sin(θ)；累加器按 **per-packet 乘法因子** `hough_decay_factor` 衰减（jAER `applyDecay` 每包乘以固定因子，默认 0.6）；寻找局部极大值并经 NMS 抑制后输出直线段（按 θ 斜率将 (ρ,θ) 转为图像满跨线段），按 (θ,ρ) 邻近度最近邻关联持久航迹。

**ROI 处理（关键）**：`HoughLineBackend` 采用 `ProcessRegion` + `crop_to_roi` 模式（非 `RoiFilter`），按 **ROI 尺寸**（非传感器尺寸）构造 `HoughLineTracker`，事件先裁剪到 ROI 相对坐标再投票。检测到的线段端点按 ROI 原点 `(x0, y0)` 反向平移回传感器坐标用于叠加渲染。ROI 变更（坐标/尺寸/启停）触发 `rebuild()` 重建累加器。

**参数与合法范围**：`threshold`：int，`[2, 500]`，默认 `50`；`num_theta_bins`：int，`[8, 360]`，默认 `90`；`num_rho_bins`：int，`[0, 4000]`，默认 `0`（0 = 按图像对角线自动 1px 分辨率）；`hough_decay_factor`：float，`[0, 1]`，默认 `0.6`（jAER per-packet 衰减因子。原 `accumulator_decay_us` 为算法端死参数，已从 GUI 移除）

**GUI 显示**：检测线段叠加 + Hough θ-ρ 累加器空间伪彩图（JET colormap，路由到 AlgoWindow 独立窗口显示）

#### 4.3.15 🆕 霍夫圆跟踪 (HoughCircleTracker) — 叠加型

✅ 移植自 jAER `HoughCircleTracker`（见 1.6.2）。事件驱动增量霍夫圆变换：维护 3D 累加器 (a, b, r)，逐事件对每个候选半径 r，在以事件 (x,y) 为中心、半径为 r 的圆上对所有候选圆心 (a, b) 投票（a = x + r·cos(θ), b = y + r·sin(θ)）；累加器按 jAER 的非指数公式 `1/(0.0001·decay·dt)` 衰减（事件过期）；在累加器中寻找局部极大值，经跨半径 NMS 抑制后输出圆，并按最近邻关联持久航迹。峰值扫描按 20Hz 节流（节流期间事件仍入累加器，只跳过 find_peaks）。

**ROI 处理（关键，防崩溃）**：3D 累加器尺寸为 `width*height*num_radii`。若按传感器尺寸（如 1280×720, max_radius=50）构造，累加器将达 ~42M cells (168 MB) 且 `find_peaks` 每帧扫描全部 cells，导致内存爆炸 + GUI 卡死/崩溃。`HoughCircleBackend` 采用 `ProcessRegion` + `crop_to_roi` 模式，按 **ROI 尺寸**（默认 128×128 → ~750K cells）构造算法实例，事件先裁剪到 ROI 相对坐标再投票。检测到的圆心按 ROI 原点 `(x0, y0)` 反向平移回传感器坐标用于叠加渲染。ROI/半径变更触发 `rebuild()` 重建累加器。

**参数与合法范围**：`max_radius`：int，`[5, 500]`，默认 `30`；`threshold`：int，`[2, 500]`，默认 `50`；`decay`：float，`[0, 10]`，默认 `1.0`；`buffer_length`：int，`[100, 100000]`，默认 `4000`；`nr_max`：int，`[1, 20]`，默认 `1`；`decay_mode`：bool，默认 `true`；`loc_depression`：bool，默认 `true`（原 `min_radius`/`accumulator_decay_us` 为算法端死参数，已从 GUI 移除）

**GUI 显示**：检测圆叠加 + Hough per-pixel 累加器伪彩图（JET colormap，路由到 AlgoWindow 独立窗口显示）

#### 4.3.17 🆕 方向共识过滤 (OrientationCluster) — 叠加型

✅ 移植自 jAER `rccar/OrientationCluster`（见 1.6.5，已取代早期自研的 4-bin 量化 + 连通域聚类版本）。逐事件朝向滤波：每像素维护连续方向向量 (vx, vy)，由感受野 `(2·rf_width+1)×(2·rf_height+1)` 内过去邻居的贡献累加而成（每个邻居贡献其归一化偏移 (xx, yy)，按 `factor/dt` 加权，dt 为事件间隔，越老的邻居权重越低）；反极性邻居旋转 90°（对比度梯度垂直于边缘），同极性下半部分邻居做点反演使所有向量 y 分量为正。可选 `ori_history_enabled` 对方向向量做时序平滑（`history_factor`）。事件通过条件（无连通域聚类）：朝向 θ 与邻域方向在 `tolerance`（度）内一致、与竖直方向夹角在 `ori`（度）内、且邻域向量长度超过随 y 缩放的阈值（`neighbor_thr` 按 `thr_gradient` 缩放）。通过的事件以方向线段叠加渲染（`display_length`）。

**参数与合法范围**：`dt`：float，`[100, 1000000]`，默认 `10000`（us）；`factor`：float，`[1, 100000]`，默认 `1000`；`rf_width`/`rf_height`：int，`[1, 64]`，默认 `1`；`tolerance`：float，`[0, 180]`，默认 `10`（度）；`ori`：float，`[0, 180]`，默认 `45`（度）；`neighbor_thr`：float，`[0, 1000]`，默认 `10`；`thr_gradient`：float，`[0, 100]`，默认 `0`；`history_factor`：float，`[0, 10]`，默认 `1`；`use_opposite_polarity`：bool，默认 `true`；`ori_history_enabled`：bool，默认 `false`；`display_length`：int，`[1, 500]`，默认 `10`（原 `min_events`/`min_cluster_size`/`coherence_threshold` 已不存在或为死参数，已从 GUI 移除）

#### 4.3.18 🆕 LIF 神经元网格聚类 (ClusterLIF) — 叠加型

借鉴 jAER `ClusterBubbles` / `IFSignedNeuronArray`（见 1.6.1、1.6.5），将像素映射为 LIF 神经元阵列，事件触发膜电位更新，超过阈值即形成聚类。

**参数与合法范围**：`tau_ms`：float，`[1, 1000]`，默认 `22`（= jAER 22000us）；`threshold`：float，`[0.1, 1000]`，默认 `15`；`receptive_field_size_pixels`：int，`[2, 128]`，默认 `8`；`initial_potential_percent`：float，`[0, 100]`，默认 `50`；`jump_after_firing_percent`：float，`[0, 100]`，默认 `10`

#### 4.3.19 🆕 背景掩码学习 (BackgroundMaskFilter) — 叠加型

借鉴 jAER `BackgroundActivityFilter` + `Histogram2DFilter`（见 1.6.1），长期学习场景背景（静止/慢动区域），输出运动前景掩码，供下游跟踪使用。

**参数与合法范围**：`learning_window_s`：float，`[0.1, 60]`，默认 `5.0`（jAER 风格 5s 工作点；该控件映射到算法的 `set_learning_window_s`，是学习窗口秒数而非"学习率"，原误名 `learning_rate` 已修正）；`threshold`：float，`[1, 100]`，默认 `10`；`erosion_size`：int，`[0, 20]`，默认 `0`

#### 4.3.20 ~~透视去畸变 (PerspectiveUndistort)~~ — 已移除

注册项、后端与 `algo/cv/perspective_undistort.h` 已删除：该后端从不调用 `set_calibration()`，启用后永远无效果，且构造参数绑定错误（见 doc/systematic_audit.md §三-B8）。事件去畸变由共享预处理 **undistort 阶段**承担（见 §4.5.3，在侧栏 Algorithms 面板 Preprocessing 组勾选启用）。

#### 4.3.21 🆕 触发同步过滤 (TriggerSyncedFilter) — 叠加型

借鉴 jAER `DataSynchronizerFromTriggers`（见 1.5.3），仅保留外部触发窗口内事件，去除窗口外噪声。用于多传感器同步采集。

**参数与合法范围**：`window_us`：int，`[1000, 1000000]`，默认 `10000`；`t0_us`：int，`[0, 1000]`，默认 `500`；`trigger_channel`：int，`[0, 7]`，默认 `0`

> 注：当前 GUI 未接入外部触发源（无代码调用 `add_trigger`），该算法输出恒为空；面板中已标注"Requires an external trigger source"（见 doc/systematic_audit.md §五-G3）。

#### 4.3.22 🆕 IIR 带通滤波 (BandpassFilter) — 叠加型

借鉴 jAER `LowPassFilter` / `HighPassFilter` 组合（见 1.6.8），对事件率信号进行 IIR 带通滤波，提取特定频率范围内的运动（如旋转机械的振动频率）。

**参数与合法范围**：`low_cutoff_hz`：float，`[0.01, 100]`，默认 `1.0`；`high_cutoff_hz`：float，`[0.01, 1000]`，默认 `10.0`

#### 4.3.23 🆕 电子稳定 EIS (OpticalGyro) — 叠加型

✅ 移植自 jAER `OpticalGyro`（见 1.6.5），估计全局相机运动（平移 + 旋转），反向补偿事件坐标，实现电子稳定。内部维护聚类跟踪器，从聚类位移计算 mass-weighted 平移 + small-angle LS 旋转，低通滤波后反向变换事件坐标。

**参数与合法范围**：`stabilize`：bool，默认 `true`；`rotation_enabled`：bool，默认 `false`（仅平移 vs 平移+旋转）；`smoothing_window_ms`：float，`[10, 1000]`，默认 `100`

**GUI 显示**：平移向量箭头（从芯片中心，缩放显示累积位移）+ 旋转弧段（指示旋转方向与幅度）+ 运动量文字（trans/rot）

#### 4.3.24 🆕 超高速等效回放 (UltraSlowMotion) — 主显示替换型

时间戳膨胀重渲染，等效 ≥200,000 fps。替换主显示区帧（不叠加）。

**参数与合法范围**：`factor`：int，`[1, 1000]`，默认 `10`（时间膨胀因子）

#### 4.3.25 🆕 XYT 3D 事件点云 (XYTVisualizer) — 独立窗口型

3D x-y-t 事件点云可视化，替代原 Temporal Plot（2D x-t/y-t 散点图）。每个事件为 3D 空间中的一个点（X=像素列, Y=像素行, T=时间轴/深度），支持极性着色（ON=红/OFF=绿）或事件年龄渐变（蓝→绿→红）。底层为 `gui/display/space_time_display.h`（VBO+GLSL），严格对齐 jAER `SpaceTimeRollingEventDisplayMethod`。

**坐标系**（对齐 jAER）：X=像素列（水平），Y=像素行（垂直，不翻转），T=时间（深度轴， newest 在前/靠近相机，oldest 在后/远离相机）。空间轴按 `smax = max(sensor_w, sensor_h)` 归一化，保持传感器宽高比（例如 640×480 → X∈[0,1.0], Y∈[0,0.75]）；时间轴按 `time_aspect_ratio`（默认 4，匹配 jAER）缩放。模型矩阵 `scale(sw, sh, time_aspect_ratio)` 一次性应用所有比例，着色器和标签投影使用同一 MVP，避免双重缩放。

**3D 包围盒与标签**（对齐 jAER `maybeRegenerateAxesDisplayList()`）：绘制 12 条边的 3D 长方体线框（非 3 条穿过原点的轴线）。前截面（z=1, newest）= 蓝色，后截面（z=0, oldest）= 暗红色，深度边 = 蓝→暗红渐变。标签位置匹配 jAER 角点：`x=sx` 在 (1.05, 0, 1) 蓝色，`y=sy` 在 (0, 1.05, 1) 绿色，`t=0` 在 (-0.05, 0, 1) 蓝色，`t=-Xms` 在 (1.05, 0, 0) 红色。使用独立 GLSL line shader 渲染 GL_LINES + QPainter 2D 文字叠加。左上角显示事件数、时间窗、着色模式等信息。

**着色**（对齐 jAER fragment shader）：Age 模式下，`colorize()` 仅产生原始 r/g/b 分量（不乘亮度），亮度由 fragment shader 统一应用 `brightness = 0.75*tn + 0.25`（tn=1 newest → 亮度 1.0，tn=0 oldest → 亮度 0.25），避免双重亮度。颜色公式：`f=1-tn`，`b=max(1-2f,0)`，`r=max(2(f-0.5),0)`，`g=f if f≤0.5 else 1-f`。点大小 `gl_PointSize = pointSize*tn + 1`（新事件更大）。

**tn 归一化**（对齐 jAER 动态窗口行为）：`render()` 使用缓冲区中事件的**实际时间跨度**（`buffer_.front().t` → `latest_t_`）归一化 tn 到 [0,1]，而非理论 `time_window_ms`。jAER 根据帧率动态设置时间窗口使事件始终填满窗口；我们使用固定窗口，若按理论窗口归一化，当事件实际跨度远小于窗口（如 1ms 事件 vs 50ms 窗口）时所有 tn≈1.0（全蓝、全在前平面、呈平面状）。按实际范围归一化确保 tn 始终跨越 [0,1]，无论事件密度如何。GUI `time_window_us` 参数（默认 500000us=500ms）通过 `on_open_xyt_view()` 初始同步和事件推送回调持续同步到 `SpaceTimeDisplay::set_time_window_ms()`，确保用户在 AlgoWindow 中的参数修改实时生效。

**渲染性能**（对齐 jAER 流畅度）：事件推送间隔 16ms（≈60 FPS），每批最多 5000 事件（降采样），配合 `kMaxBuffer=200000` 硬上限可保留约 40 批 ≈ 640ms 的时间跨度，确保点云时间连续。`SpaceTimeDisplay` 内置 60 FPS `QTimer` 渲染定时器，将渲染节奏与事件推送解耦——即使事件以突发方式到达，显示仍保持稳定 60 FPS。`render()` 接受预分配 `std::vector<XYTPoint>&` 输出参数（避免每帧堆分配），`rebuild_vbo()` 复用成员 `vbo_data_` 缓冲区（避免每帧 vector 分配），确保大数据量下无卡顿。

**参数与合法范围**：`time_window_us`：int，`[10000, 10000000]`，默认 `500000`（500ms，显示最近 N µs 事件）。着色/点大小/自动旋转/深度遮蔽为 `SpaceTimeDisplay` 内部设置（Age 着色、`point_size=2.5`、深度亮度对齐 jAER shader），不在算法参数面板暴露；原 `max_points` 参数后端只存不用，已从 GUI 移除。

**ROI 处理区**（默认启用，详见 §5.6.6）：`roi_enabled`：bool，默认 `true`；`roi_x`/`roi_y`：int，`-1` 表示自动居中，默认 `-1`；`roi_w`/`roi_h`：int，`0` 表示全幅，默认 `128`（即默认 128×128 中心区域）。启用时仅向 3D 点云推送 ROI 内事件，主显示帧同步绘制黄色 ROI 边框。

#### 4.3.26 🆕 可视化叠加 (Overlay)

将叠加型算法结果叠加到渲染帧上：光流箭头/颜色图、目标 bbox+ID、检测线、轨迹线、角点标记、线段、圆环、统计文字等。

#### 4.3.27 🆕 Time Surface 窗口 (TimeSurfaceWindow) — 独立窗口型

一键打开独立窗口，实时显示 Time Surface（每像素最近事件时间衰减编码图）。底层调用 openEB `TimeSurfaceProcessor`，按时间衰减将最近事件时间戳编码为伪彩图像，可用于观察运动轨迹残留、分析事件时序分布。

| 子模块 | 方案 |
|--------|------|
| Time Surface 计算 | openEB `TimeSurfaceProcessor<CHANNELS>`（合并极性/分离极性） |
| 时间衰减渲染 | 将 `MostRecentTimestampBuffer` 通过 LUT 转换为伪彩 `cv::Mat`，叠加时间衰减效果 |
| 窗口独立刷新 | 与主显示区共享事件流，独立累积窗口与刷新率 |

**参数与合法范围**：
- `channels`：枚举（1=合并极性 / 2=分离极性），默认 `1`
- `decay_time_us`：int（μs），范围 `[10000, 5000000]`，默认 `100000`（100ms）
- `palette`：色彩映射枚举（Gray/Hot/Plasma/Turbo 等），默认 `Hot`
- `refresh_rate_hz`：int，范围 `[10, 120]`，默认 `30`

**ROI 处理区**（默认启用，详见 §5.6.6）：`roi_enabled`：bool，默认 `true`；`roi_x`/`roi_y`：int，`-1` 表示自动居中，默认 `-1`；`roi_w`/`roi_h`：int，`0` 表示全幅，默认 `128`（即默认 128×128 中心区域）。启用时仅向 Time Surface 推送 ROI 内事件并按 ROI 尺寸重建 MostRecentTimestampBuffer，主显示帧同步绘制黄色 ROI 边框。

**一键操作**：菜单 Algorithm → Time Surface 或工具栏按钮，单击即弹出独立 Time Surface 窗口；窗口可拖拽、停靠、缩放。Algorithm 菜单项为可勾选状态，再次点击或关闭窗口即可手动停用该功能。

### 4.4 algo/analytics/ — 分析模块

#### 4.4.1 主动标记跟踪 (ActiveMarker)

**原理**：借鉴 [`ref/Lighthouse/doc/d_AlgorithmDesign_zh.md`](file:///home/justin/GUI_for_openEB/GUI-for-openEB/ref/Lighthouse/doc/d_AlgorithmDesign_zh.md) 滑动窗口方案，对每个事件聚类在时间窗口内的事件数进行统计，根据事件数多少赋予颜色——越少越蓝，越多越红。支持闪烁频率 ID 识别（可选）。

| 子模块 | 方案 |
|--------|------|
| 滑动窗口缓冲 | 事件触发式窗口（T_w 默认 20ms，ΔT ≤ 2ms），窗口内事件集合 E_w(t) |
| 事件聚类 | 热图阈值化 + 连通域分析 → 聚类；每个聚类统计窗口内事件数 |
| 颜色映射 | 事件数 → colormap（蓝→绿→红），事件少=蓝（冷），事件多=红（热） |
| 频率检测（可选） | 聚类质心 3×3 区域分箱 FFT → 闪烁频率（参考 `freq_analyzer`） |
| ID 识别（可选） | 频率→ID 映射（需预设频率表） |
| 输出 | 热图叠加 + 聚类标注（颜色+事件数+频率） |

**参数与合法范围**：`window_us`：int，`[1000, 100000]`，默认 `10000`；`min_events`：int，`[5, 500]`，默认 `20`

**可视化**：主显示帧叠加热图（半透明）+ 聚类圆圈标注（颜色编码事件数）。

#### 4.4.2 事件→灰度图像重建 (EventToVideo)

提供三种重建路径——两种 **非 DL 路径**（纯事件，无外部依赖）与一种 **DL 路径**（可选，需模型权重）。

| 模式 | 算法方案 | 参考文献 | 外部依赖 |
|------|----------|----------|----------|
| **InteractingMaps**（非 DL） | 事件累积 → V 图（对数亮度变化）→ V 钳位到 `[-1, 1]` → **六图交替松弛**（Cook 2011 §II，I/G/V/F/C/R）→ 灰度帧。三关系松弛迭代：(i) `G=∇I`、(ii) `-V=F·G` → `F=-V·G/|G|²`、(iii) `F=m32(R×C)` → 旋转最小二乘估计 R 后回代 F；I 更新采用 Poisson 梯度积分（`∇²I=∇·G`）+ 数据保真（有事件处 I→V）。严格复现论文，仅加入最小稳定性修复：V/F/R/I 钳位、`|G|²` 阈值跳过平坦区、首帧 I=V 温启动（避免 F·G 死锁） | [Interacting Maps for Fast Visual Interpretation](file:///home/justin/GUI_for_openEB/GUI-for-openEB/ref/Interacting_maps_for_fast_visual_interpretation.pdf)（Cook et al. 2011 IJCNN） | 无 |
| **BardowVariational**（非 DL） | 滑动窗口变分优化：联合估计连续速度场 u（光流）与对数亮度 L，使用 `lambda1`-`lambda6` 六个正则化权重（λ1 光流 TV、λ2 光流时间平滑、λ3 亮度 TV、λ4 光流约束一致性、λ5 无事件死区 h_θ、λ6 先验图保持）。Chambolle-Pock 原始-对偶交替优化（biconvex split）。适配实时 2D 框架：以单时间步滑动窗口（当前帧 ↔ 上一帧）近似时空体素，保留全部 6 项和光流-亮度联合估计 | [Simultaneous Optical Flow and Intensity Estimation from an Event Camera](file:///home/justin/GUI_for_openEB/GUI-for-openEB/ref/Simultaneous_Optical_Flow_and_Intensity_Estimation_from_TQEb.pdf)（Bardow et al. 2016 CVPR） | 无 |
| **E2VID**（DL，可选，默认） | E2VID / UNet-Recurrent 神经网络推理，从事件流重建灰度帧。已从 [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) 完整移植：事件体素网格 → ONNX Runtime 推理（含 UNetRecurrent 状态管理）→ Unsharp Mask → 强度重缩放（auto-HDR）→ 裁剪 → 双边滤波。无模型时自动回退到启发式重建（体素求和 + Sigmoid） | [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) | ONNX Runtime（可选） |

**子模块**：
- `interacting_maps`（非 DL）：**滑动时间窗** `[t - window_ms, t]` 内事件求和 → V 图（对数亮度变化）→ V 钳位 `[-1, 1]` → **六图交替松弛**（Cook 2011 §II）→ 灰度帧。窗口外事件完全丢弃（无指数衰减），对齐 Cook 2011 的短时间窗分箱方案。松弛迭代三关系：(i) `G=∇I`（梯度图松弛趋近 I 的梯度）、(ii) `-V=F·G` → `F=-V·G/|G|²`（光流约束，`|G|²` 阈值跳过平坦区，|F| 钳位 5.0）、(iii) `F=m32(R×C)`（旋转几何——先由 F、C 最小二乘解 R 正则方程 `M·R=b`，R 钳位 0.5 弧度，再回代得 F）；I 更新为 Poisson 梯度积分 `I_target=avg(neighbors)−div(G)/4` 与数据保真 `0.5·Vc+0.5·poisson`（有事件处）的加权和，钳位 `[-1,1]`。首帧 I=V 温启动避免 F·G 死锁（I=G=F=0 时 V 无法影响系统）
- `bardow_variational`（非 DL）：**滑动时间窗** `[t - window_ms, t]` 内事件求和 → 对数亮度数据项 f → Chambolle-Pock 交替优化 L（亮度，含 λ3 TV + λ4 光流约束 + λ5 死区 + λ6 先验）和 u（光流，含 λ1 TV + λ2 时间平滑 + 光流约束最小范数解）→ 灰度帧。窗口外事件完全丢弃（无指数衰减），对齐 Bardow 2016 的滑动窗口公式。λ6=1.0（论文值），但**仅应用于无新事件的像素**（`|f|≈0`）——论文先验项 `λ6·||L(x,t1)−L̂(x)||²` 仅约束窗口最老时间步，在我们的单步框架中等价于"补偿窗口滑动信息损失"，仅对窗口滑过后丢失事件的像素生效；有新事件的像素由数据项主导，不混入先验。这避免了之前 λ6=1.0 整帧混入导致 50% 旧帧残影的问题，同时保持论文权重
- `e2vid/`（DL，默认）：子目录，包含 `e2vid_inference.h`（ONNX Runtime 推理 + 启发式回退 + UNetRecurrent 状态管理）、`event_voxel_grid.h`（事件→体素网格，双线性时间插值）、`intensity_rescaler.h`（auto-HDR 强度重缩放）、`unsharp_mask.h`（Unsharp Mask + 双边滤波）。已从 [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) 完整移植

**参数与合法范围**（GUI 按当前 `mode` 仅显示对应模式的可调参数，由 `AlgoParamSpec::mode_filter` 控制行可见性）：
- `mode`：枚举（BardowVariational / InteractingMaps / E2VID），默认 `E2VID`（通用参数，始终可见）
- `output_fps`：int，`[1, 120]`，默认 `30`（通用参数，始终可见）
- `window_ms`：float，`[10, 500]`，默认 `50`（mode_filter `0,1`——BardowVariational / InteractingMaps 共享。**滑动时间窗**：`get_frame()` 时仅汇总 `[current_t - window_ms, current_t]` 内的事件作为数据项 f（Bardow）/ V（Cook），窗口外事件从环形缓冲区剪枝丢弃。对齐两篇论文的滑动窗口公式——无指数衰减，旧事件完全消失，残影不超过 window_ms。50ms 在 30fps 下覆盖 ~1.5 帧，提供足够事件又保证快速响应）
- BardowVariational 模式（mode=0）：`delta_t_ms`：float，`[1, 50]`，默认 `15`（快运动时减小）；`theta`：float，`[0.05, 0.5]`，默认 `0.22`（事件触发阈值）；`num_iterations`：int，`[10, 500]`，默认 `100`（Chambolle-Pock 交替优化外层迭代数）；`lambda1`：float，默认 `0.02`（光流 TV 空间平滑）；`lambda2`：float，默认 `0.05`（光流时间平滑）；`lambda3`：float，默认 `0.02`（亮度 TV 空间平滑）；`lambda4`：float，默认 `0.2`（光流约束一致性，亮度时间变化 = 光流·梯度）；`lambda5`：float，默认 `0.1`（无事件死区 h_θ 约束）；`lambda6`：float，`[0, 2]`，默认 `1.0`（**论文值**，先验图保持。仅应用于无新事件的像素 `|f|≈0`——论文先验项 `λ6·||L(x,t1)−L̂(x)||²` 仅约束窗口最老时间步，在我们的单步框架中等价于补偿窗口滑动信息损失，仅对窗口滑过后丢失事件的像素生效；有新事件的像素由数据项主导。这避免了之前 λ6=1.0 整帧混入导致 50% 旧帧残影的问题，同时保持论文权重）
- InteractingMaps 模式（mode=1）：`relaxation_step`：float，`[0.001, 0.5]`，默认 `0.1`（六图松弛欠松弛步长 δ，每张图按 `map=(1−δ)·map+δ·target` 更新）；`im_iterations`：int，`[10, 1000]`，默认 `50`（每帧六图松弛外层迭代数）；`fov_deg`：float，`[10, 170]`，默认 `60`（相机视场角，用于构建标定图 C——每像素 3D 单位方向向量，参与关系 (iii) `F=m32(R×C)` 的旋转估计）。注：`lambda3`/`num_iterations` 的 mode_filter 恢复为 `0`（仅 Bardow），InteractingMaps 不再复用 TV 去噪参数
- E2VID 模式（mode=2）：`model_path`：string（ONNX 模型路径，加载失败自动回退到启发式）；`num_bins`：int，`[1, 20]`，默认 `5`（时间体素分箱数；**加载 ONNX 模型后自动同步为模型的输入通道数**，对齐 rpg_e2vid 的 `model.num_bins` 行为——见 [run_reconstruction.py:55](file:///home/justin/GUI-for-openEB/ref/rpg_e2vid/run_reconstruction.py#L55)、[model/model.py:14](file:///home/justin/GUI-for-openEB/ref/rpg_e2vid/model/model.py#L14)，此时用户修改 num_bins 不再生效）；`auto_hdr`：bool，默认 `false`（自动 HDR 强度重缩放，对齐 README `--auto_hdr`）；`unsharp_amount`：float，`[0, 2]`，默认 `0.3`（锐化强度，对齐 `--unsharp_mask_amount`）；`unsharp_sigma`：float，`[0.1, 5]`，默认 `1.0`（高斯模糊 σ，对齐 `--unsharp_mask_sigma`）；`bilateral_sigma`：float，`[0, 10]`，默认 `0.0`（双边滤波 σ，0 = 禁用，对齐 `--bilateral_filter_sigma`）。`hot_pixel_mask`（uint8 向量，长度 = width × height）为程序内部接口，不在 GUI 暴露。rpg_e2vid 的 `--no-recurrent`/`--color`/`--flip` 等高级选项未暴露（默认值与 rpg_e2vid 一致：循环连接开启、灰度、不翻转）。`--no-normalize` 已默认启用（`normalize_input_=false`）以提升实时性能

**ROI 处理区**（默认启用，详见 §5.6.6）：`roi_enabled`：bool，默认 `true`；`roi_x`/`roi_y`：int，`-1` 表示自动居中，默认 `-1`；`roi_w`/`roi_h`：int，`0` 表示全幅，默认 `128`（即默认 128×128 中心区域）。启用时仅向重建算法推送 ROI 内事件并按 ROI 尺寸重建算法实例（避免全幅高延迟），主显示帧同步绘制黄色 ROI 边框。重建窗口以 ROI 大小独立显示灰度帧。**初次构建事件重建参数页时 GUI 自动设置模式对应的 ROI 与帧率**：三种模式均自动设为 128×128 中心区域（`roi_w`/`roi_h`=128、`roi_x`/`roi_y`=-1）+ `output_fps`=24（仅在初次构建时设置一次，用户后续手动切换模式不重置自定义 ROI/fps）。此自动设置在主面板（AlgorithmsPanel）和小窗（AlgoWindow）中均生效。三种模式下 128×128 的实时性能由以下优化保障：(1) ONNX Runtime 使用全部 CPU 核心（`SetIntraOpNumThreads(hardware_concurrency)`，上限 8）并行推理 Conv/MatMul；(2) `GraphOptimizationLevel::ORT_ENABLE_ALL` 启用全部图优化；(3) 默认禁用事件张量归一化（`normalize_input_=false`，对齐 rpg_e2vid README `--no-normalize`：「will improve speed a bit, but might degrade the image quality a bit」）；(4) 可在共享预处理中显式勾选 **1/4 Downsample**（默认 OFF）：仅保留 x、y 坐标均为偶数的事件并映射到 `(x/2, y/2)`，对 128×128 ROI 产生 64×64 有效分辨率送入重建（计算量降至 ~1/4），输出后 `INTER_NEAREST` 上采样回 128×128。原每算法 `downsample` 参数已移除，改为共享预处理阶段 `preproc_downsample`（默认 OFF——对多数后端它只是抽稀事件而非降分辨率，默认开启会静默损失 3/4 输入，见 doc/systematic_audit.md §五-F1）。

**E2VID 后台推理**：E2VID 模式在独立 worker 线程中执行 ONNX 推理（事件经有界队列投递，GUI 线程不再冻结）；推理慢于事件到达率时丢弃最旧的批（状态行显示 `dropped=N`），模型加载成功/启发式回退在状态行以 `model=loaded` / `model=heuristic` 标注。

**E2VID 模型配置**（DL 路径，可选）：DL 重建依赖 ONNX 格式的预训练模型。模型权重文件（`.pth.tar`/`.onnx`/`.onnx.data`）体积较大，不入版本库（见 `.gitignore`），用户按以下步骤本地准备：

1. **安装 ONNX Runtime C++ 运行时**（一次性）。将官方预编译包（1.19.2 Linux x64 CPU）解压到 `third_party/onnxruntime/`，目录结构需为 `third_party/onnxruntime/{include,lib}`。CMake（`algo/CMakeLists.txt`）会自动检测该路径并启用 `GUI_ALGO_HAS_ONNXRUNTIME` 编译宏；缺失时 DL 路径编译为空实现，自动回退到启发式重建。
2. **创建 Python 转换环境**（一次性）。在仓库根目录执行 `python3 -m venv .venv && . .venv/bin/activate && pip install torch --index-url https://download.pytorch.org/whl/cpu onnx onnxscript onnxruntime numpy`（CPU 版 torch 即可）。
3. **下载 PyTorch 预训练权重**。从 [rpg_e2vid 模型页](http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar) 下载 `E2VID_lightweight.pth.tar`（约 41 MB）到 `models/`。
4. **导出 ONNX 模型**。执行 `python models/convert_to_onnx.py`，脚本会读取 `models/E2VID_lightweight.pth.tar`，使用 `E2VIDRecurrentONNXWrapper`（将 ConvLSTM 的 `[(h0,c0),(h1,c1),(h2,c2)]` 列表状态展平为 6 个独立张量）导出为 `models/e2vid_lightweight.onnx`。导出使用 `dynamo=False`（传统 TorchScript 导出器）以避免 PyTorch 2.x 新导出器的 `Split` op 在 opset 17 下的兼容性问题；动态轴为每编码层级使用独立维度名（`H/W`、`H2/W2`、`H4/W4`、`H8/W8`），`batch` 维度也设为动态。
5. **配置 GUI 模型路径**。`model_path` 参数默认值为 `models/e2vid_lightweight.onnx`（相对工作目录），通常无需修改。如使用其他模型（如 `E2VID.pth.tar`、`E2VID_firenet.pth.tar`），重复步骤 3-4 即可，然后在 GUI 的 `EventToVideo` 算法参数中将 `model_path` 指向新生成的 `.onnx` 文件。

**ONNX 模型 I/O 约定**（由 `algo/analytics/e2vid/e2vid_inference.h` 自动适配）：
- 输入：`event_tensor` `[1, num_bins, H, W]` + 6 个 ConvLSTM 状态张量（`h0/c0` `[1, 64, H/2, W/2]`、`h1/c1` `[1, 128, H/4, W/4]`、`h2/c2` `[1, 256, H/8, W/8]`）。共 7 个输入，对应 `num_encoders=3`。
- 输出：`image` `[1, 1, H, W]`（灰度图，sigmoid 后值域 `[0,1]`） + 6 个新状态张量（结构同输入状态，循环传递给下一帧）。
- `num_bins` 从模型第一输入的通道维自动读取；`num_encoders` 从输入总数推断（`n_inputs = 1 + 2 * num_encoders`），自动重算 `CropParameters`（2^num_encoders 对齐）。两者均对齐 rpg_e2vid 中 `model.num_bins` / `model.num_encoders` 的行为——加载模型后用户修改不再生效。
- 零状态张量首次推理时按 `crop_h / 2^(level+1)` 计算各层级空间维度，动态维度（ONNX 报告为 `-1`）全部替换为具体值（含 batch 维）。

**用途**：事件数据可视化、帧基算法输入、视频导出。Algorithm 菜单项为可勾选状态，可手动停用。

#### 4.4.3 🆕 光流评估 (FlowStatistics)

借鉴 jAER `NoiseTesterFilter`（TP/FP/TN/FN 框架，见 1.6.1）思路，对光流算法（4.3.9）输出进行质量评估。

| 子模块 | 方案 |
|--------|------|
| Ground truth | 合成数据（已知运动的仿真事件流）或手动标注 |
| 误差指标 | EPE（端点误差）、PE（百分比误差）、角度误差 |
| 统计输出 | 直方图、均值、中位数、90 百分位 |

**参数**：无（注册为 Passive，实时路径不产生评估输出；统计/渲染 API 供离线/GT 流程使用）

#### 4.4.4 🆕 ISI 直方图分析 (ISIAnalyzer)

借鉴 jAER `IntegrateAndFire` / `ISI` 工具（见 1.6.8），统计每像素（或全图）事件间隔分布（Inter-Spike Interval），用于场景运动频率分析、噪声特性评估。

| 子模块 | 方案 |
|--------|------|
| ISI 统计 | `histogram_ring_buffer` 维护每像素最近 N 次 ISI |
| 直方图渲染 | 在独立窗口显示全图 ISI 直方图 |
| 异常检测 | ISI 异常短/长 → 标记噪声像素 |

**参数与合法范围**：`max_isi_ms`：float，`[1, 1000]`，默认 `100`；`per_pixel`：bool，默认 `false`（全图统计）。`bin_count` 固定 32，不在 GUI 暴露。

**ROI 处理区**（默认启用，详见 §5.6.6）：`roi_enabled`：bool，默认 `true`；`roi_x`/`roi_y`：int，`-1` 表示自动居中，默认 `-1`；`roi_w`/`roi_h`：int，`0` 表示全幅，默认 `128`（即默认 128×128 中心区域）。启用时仅统计 ROI 内事件的 ISI。Algorithm 菜单项为可勾选状态，可手动停用。

#### 4.4.5 🆕 通用颗粒计数器 (ParticleCounter)

借鉴 jAER `particlecounter` 项目（见 1.6.5），统计传送带/管道上的颗粒流（颗粒/秒）。

| 子模块 | 方案 |
|--------|------|
| 颗粒检测 | 复用 4.3.10 团块检测 |
| 计数线 | 用户定义虚拟检测线 |
| 颗粒跟踪 | 复用 4.3.11 跟踪，避免重复计数 |
| 输出 | 颗粒/秒、累计计数、尺寸分布直方图 |

**参数与合法范围**：`line_y`：int，`[-1, sensor_h]`，默认 `-1`（-1 = 自动取传感器高度中点；硬编码固定值在非 720p 传感器上会错位）；`min_area`：int，`[1, 10000]`，默认 `10`

#### 4.4.6 🆕 自适应 Bias 控制 (AutoBiasController)

借鉴 jAER `AutoExposureController` 的自动曝光反馈思路（原版基于 APS 直方图统计，本模块改用事件率作为反馈信号），按事件率/动态范围反馈自动调整相机 Bias，保持最优曝光。

| 子模块 | 方案 |
|--------|------|
| 反馈信号 | 事件率（Mev/s）、在/不在范围事件比、噪声率 |
| 控制律 | PID 控制器 → 调整 `bias_diff`/`bias_on`/`bias_off` |
| 边界保护 | Bias 上下限钳位、变化率限制 |

**参数与合法范围**：`target_event_rate_mev`：float，`[0.1, 50]`，默认 `5.0`（PID 系数与步长限制为算法内置，不在 GUI 暴露）

#### 4.4.7 🆕 闪烁光源频率检测 (FreqDetector)

借鉴 [`ref/Lighthouse/tools/freq_analyzer`](file:///home/justin/GUI_for_openEB/GUI-for-openEB/ref/Lighthouse/tools/freq_analyzer/main.cpp)，检测视野内相对静止闪烁光源（LED 等）的事件频率。

| 子模块 | 方案 |
|--------|------|
| 累积热图 | 累积窗口事件 → HxW 像素级热图（事件计数） |
| 聚类定位 | 阈值化 + `cv::connectedComponentsWithStats` → 闪烁光源聚类 |
| 频率分析 | 每个聚类质心周围 3×3 像素区域提取事件时间戳 → 分箱 FFT（Hann 窗 + 抛物线插值） → 频率峰值 |
| 谐波确认 | 最低显著峰 + 2× 谐波确认（方波信号半频误判消除） |
| 输出 | 每个光源的 (u, v, event_freq_hz, blink_freq_hz) + 热图叠加标注 |

**频率定义**：事件频率 = 2 × LED 物理闪烁频率（LED 每次亮/灭各触发一个事件）。

**参数与合法范围**：`update_interval_s`：float，`[0.1, 10]`，默认 `1.0`；`min_events`：int，`[1, 1000]`，默认 `3`（`f_min`/`f_max`/`bin_dt_us`/`heatmap_threshold` 等为算法内置默认值，不在 GUI 暴露）

**ROI 处理区**（默认启用，详见 §5.6.6）：`roi_enabled`：bool，默认 `true`；`roi_x`/`roi_y`：int，`-1` 表示自动居中，默认 `-1`；`roi_w`/`roi_h`：int，`0` 表示全幅，默认 `128`（即默认 128×128 中心区域）。启用时仅分析 ROI 内事件。Algorithm 菜单项为可勾选状态，可手动停用。

**可视化**：热图 colormap（Inferno）+ 光源位置圆圈标注 + 频率文本（Hz）。

#### 4.4.8 🆕 Sensor 自检 (SensorSelfTest)

对相机 sensor 进行坏点检测与不应期（refractory period）估计的自检算法。用户主动摇晃相机以刺激全 sensor 事件生成；算法对每个像素独立统计两次事件之间的历史最小间隔时间，作为该像素不应期估计。若某像素从未触发事件，则判定为疑似坏点。

| 子模块 | 方案 |
|--------|------|
| 逐像素追踪 | 两个 HxW 数组：`last_ts_`（上次事件时间戳，哨兵 `kNeverSeen`）+ `min_interval_`（历史最小间隔，哨兵 `kNoInterval`）。每个事件 O(1)：读上次时间戳 → 计算间隔 → 与当前最小值比较 → 更新 |
| 指数 LUT 渲染 | 预计算 `v = 255 * exp(-dt / 2000.0)` 的 0~10000us 查找表，`render()` 时按 `min_interval_` 查表生成灰度热图；未触发像素染红 `(0,0,255)`，仅 1 次事件像素染黑 |
| 统计报告 | `compute_stats()` 一次性 O(N log N) 排序计算 min/max/mean/median/p90；`bad_pixel_coords()` 收集坏点坐标（上限 200） |

**复杂度**：事件处理 O(1)/事件（两次数组读 + 一次写 + 一次比较）；渲染 O(N)（N = 像素数）；统计 O(N log N)（仅报告时调用，每 30 帧缓存一次）。

**参数与合法范围**：无参数 —— 完全自动，覆盖全 sensor（不应用 ROI/预处理，确保坏点检测覆盖每个像素）。

**触发方式**：硬件面板（DevicesPanel）"Sensor Self-Test" 按钮，需先连接相机。点击后弹出独立窗口显示热图；每次（重新）打开窗口自动 `reset()`，确保每次会话从零开始。

**可视化**：HxW `CV_8UC3` 灰度热图 —— 红色 `(0,0,255)` = 未触发（疑似坏点），黑色 = 仅 1 次事件（数据不足），灰度 = 指数映射的不应期（越亮 = 不应期越短，范围 1us~10000us 映射到 0~255）。

**关闭行为**：用户关闭显示窗口时自动停用算法实例，并弹出 `QMessageBox` 报告：不应期统计（min/max/mean/median/p90，单位 us）+ 触发/测量/坏点像素数 + 疑似坏点坐标列表（上限 50 个，超出则显示"还有 N 个"）。

### 4.5 algo/calibration/ — 相机标定（Tools 菜单向导）

标定不再注册为算法，而是通过 **Tools → Intrinsic Wizard** 菜单启动 `CalibrationWizard` 对话框完成。整个流程针对纯事件相机优化（无 APS 帧），核心思路：屏幕上绘制 20 Hz 闪烁棋盘，每次翻转在棋盘边缘产生事件突发，1 ms 累积窗口对齐翻转即可"免费"捕获棋盘图案。

#### 4.5.1 内参标定向导 (CalibrationWizard)

| 子模块 | 方案 | 借鉴 |
|--------|------|------|
| 标定板显示 | `ChessboardDisplay`：全屏黑白棋盘，20 Hz **全板取反**（50 ms 周期，每个格子黑白交替）；内嵌半透明 HUD（进度条 N/目标、状态行、Start/Stop 按钮、规格显示），位于棋盘区外的底部黑带，快捷键 **S**=开始/停止、**F**=全屏切换、**Esc**=关闭 | — |
| 极性分离渲染 | 1 ms 窗口渲染为灰度图：bg=128 / ON=255 / OFF=0。保留极性是**必要的**——全板取反时每个像素都会发放，若忽略极性所有格子同亮，得到实心矩形，角点在信息层面不存在（见 doc/systematic_audit.md §9.2-A） | — |
| 事件订阅 | `CameraController::cd_events_ready` 信号（cross-thread queued），`CalibrationEventTap` 缓冲批次（`Qt::UniqueConnection` 防重复注入） | — |
| 自动抓拍 | 每 50 ms 周期在整个缓冲上做**滑窗最大值搜索**：按 100 µs 细桶计数（500 桶）+ 长度为 10 的滑动和取最大（O(N)+O(桶数)），允许最优 1 ms 窗口落在任意偏移，跨边界翻转爆发也能完整捕获 | — |
| 内角点约定 | UI 输入、显示、检测统一为 **OpenCV 内角点数**（默认 9×6，棋盘格数为 10×7） | — |
| 标定板检测 | `cv::findChessboardCorners`，检测前先对渲染图做 3×3 `cv::medianBlur` 去椒盐噪声；检测不出则弃图 | — |
| 去重 | **角点位移查重**：本次与上次合格帧对应角点的平均位移 < 阈值（默认 2.0 px）判重复弃图——对翻转相位天然免疫（取代了被 20Hz 翻转击败的 MSE 查重，静止相机无法再刷满帧数） | — |
| 线程模型 | drain/渲染/检测/标定全部在 **worker 线程**执行（有界任务队列，检测跟不上时整周期丢弃），GUI 线程只保留翻转定时器与 HUD 进度显示——20 Hz 闪烁稳定、UI 不冻结 | — |
| 自动结束 | 达到目标帧数（默认 30）自动停止抓拍并在 worker 线程运行标定 | — |
| 内参计算 | Zhang 法（`cv::calibrateCamera`） | — |
| 畸变模型 | k1,k2,p1,p2,k3 | — |
| square_mm | **可编辑** spinbox（默认取屏幕物理 DPI 计算值——X11 上 DPI 常不可靠，用户可用尺子量一个格子直接填）；仅影响平移向量的米制尺度，不影响 fx/fy/cx/cy/畸变系数 | — |
| 导出 | `QFileDialog` 弹窗，默认路径 `~/Documents/EBplus/calibration/intrinsic.yml`（与去畸变预处理默认路径一致），写入前自动 `mkpath` 创建目录，写入 image_width/image_height/camera_matrix/distortion_coefficients/rms | — |

#### 4.5.2 锐度工具 (SharpnessDialog)

**Tools → Sharpness** 菜单启动。不再在显示帧上测量（显示帧被调色板/ROI 框/算法标注污染且 8-bit 饱和丢失计数信息，Laplacian 方差主要反映噪声密度——见 doc/systematic_audit.md §9.3），改为：

1. **数据源**：自带滚动累积窗（可选 50/100/200 ms），从 `CameraController::cd_events_ready` 广播的原始事件流直接构建**计数图**（每像素事件计数，忽略极性、不饱和、无调色板/叠加）。
2. **去噪**：孤立像素去除——计数 ≤1 且 8 邻域全空的像素置零（专杀热像素散点）。
3. **双指标**（`gui/calibration/sharpness_metrics.h` 纯函数，可单测）：
   - **归一化对比度** σ²(I)/μ(I)²（CMax 风格锐度代理，对事件率/窗长一阶不变）——**越高越锐**；
   - **平均线宽**（px）：计数图阈值化 → 距离变换 → 边缘像素距离均值 ×2——**越小越锐**（直接回答"线条锐利程度"）。
4. **参考量**：同时显示事件率（Mev/s）。

指标以滚动折线图实时绘制。用途：镜头对焦（调对焦环，线宽先降后升、对比度先升后降，焦点处同时取极值）与 bias 调节（先看噪声不干扰对比度）。

#### 4.5.3 事件去畸变预处理阶段 (Preprocessor undistort stage)

可叠加预处理阶段，在侧栏 **Algorithms** 面板的 Preprocessing 组中勾选 "Undistort (apply calibration)" 启用。执行顺序：ROI → filter → downsample → **undistort**。

| 子模块 | 方案 | 借鉴 |
|--------|------|------|
| 参数加载 | `load_intrinsics_yml()` 读取向导导出的 YAML（camera_matrix / distortion_coefficients / image_size） | — |
| LUT 预计算 | `cv::undistortPoints` 正向映射（distorted → undistorted），K 按 ROI 原点 + downsample 因子调整（fx/fy /= factor, cx = (cx-roi_x)/factor, cy = (cy-roi_y)/factor） | `SingleCameraCalibration` LUT (L700-718) |
| 运行时 | 每个事件查 LUT 得到去畸变后的坐标，越界事件丢弃 | JAER `undistortEvent` |
| 默认路径 | `~/Documents/EBplus/calibration/intrinsic.yml`（与向导默认输出目录一致） | — |

---

### 4.6 algo/tests/ — 自研测试基建

#### 4.6.0 GTest 单元测试框架 (已完成)

基于 Google Test + CTest 的单元测试基建，编译参数 `-Wall -Wextra -Werror`：

| 测试文件 | 覆盖模块 | 状态 |
|----------|----------|------|
| `test_phase6_common.cpp` | event / event_packet / lifo_event_buffer / dvs_framer / freme / event_rate_estimator / performance_meter / time_limiter / kalman_filter / kmeans / particle_filter / periodic_spline / histogram_ring_buffer / lif_integrator / filter/(lowpass, highpass, bandpass, angular_lowpass, median_lowpass) | ✅ |
| `test_phase7_cv.cpp` | Phase 7 CV 模块 | ✅ |
| `test_phase8_10.cpp` | Phase 8-10 analytics + E2VID | ✅ |
| `test_raw_algos.cpp` | 真实 RAW 数据回归 | ✅ |

GUI 侧另有 `gui/tests/` 6 个测试可执行文件（algo_bridge / config_manager / display_strategy / layout_manager / sharpness_metrics / theme_tokens）。

#### 4.6.1 ~~降噪评测框架 (NoiseTester)~~ — 已移除

`algo/tests/noise_tester.h` 与 `signal_noise_event.h`（jAER `NoiseTesterFilter` 移植的 TP/FP/TN/FN 评测框架）从未被任何测试使用，已删除（见 doc/systematic_audit.md §三-A5）。

## 五、GUI 布局设计

### 5.1 主窗口布局

v1.0.9 起 GUI 采用 VSCode 风格布局：自定义标题栏（CustomTitleBar，下拉菜单替代 QMenuBar）、左侧 ActivityBar + 设置面板、中央事件显示区、右侧 AlgoWindow 停靠区。工具栏已移除，录制/导出等功能迁入侧栏 File Tools 面板。

```
┌──────────────────────────────────────────────────────────┐
│  CustomTitleBar: [EB plus]  File|View|Theme|Tools|Help  [_□×]│
├────┬─────────────────────────┬───────────────────────────┤
│ A  │  Settings Panel          │                           │
│ c  │  (QStackedWidget)        │                           │
│ t  │ ┌──────────────────────┐ │     Event Display         │
│ i  │ │ Camera:               │ │     (中央事件显示区)       │
│ v  │ │   Devices, Info       │ │     OpenGL 渲染           │
│ i  │ ├──────────────────────┤ │     1280×720 / 640×480    │
│ t  │ │ Display & Stats:      │ │                           │
│ y  │ │   Display, Statistics │ │                           │
│ B  │ ├──────────────────────┤ │                           │
│ a  │ │ Hardware:             │ │                           │
│ r  │ │   Biases, ROI, ESP,   │ │                           │
│    │ │   Trigger             │ │                           │
│ 48 │ ├──────────────────────┤ │                           │
│ px │ │ Algorithms:           │ │     AlgoWindow (右侧)     │
│    │ │   Preprocessing,      │ │     (算法显示窗口)         │
│ ▼  │ │   Algorithms          │ │                           │
│    │ ├──────────────────────┤ │                           │
│    │ │ Tools:                │ │                           │
│    │ │   File Tools          │ │                           │
│    │ └──────────────────────┘ │                           │
├────┴─────────────────────────┴───────────────────────────┤
│  Status Bar: 连接状态 │ 事件率 │ 时间戳 │ 录制状态        │
├──────────────────────────────────────────────────────────┤
│  Playback Controls (回放时出现): 进度条                   │
└──────────────────────────────────────────────────────────┘
```

- **ActivityBar**（左侧 48px 图标列）：点击图标切换 QStackedWidget 页面；底部 chevron 按钮切换侧栏内容可见性（收起后 dock 缩至 48px）；空白区域可拖拽移动 dock 到对侧
- **Settings Panel**（左侧停靠，默认宽 380px）：5 个分组页面，无内部滚动条，依赖父滚动区
- **AlgoWindow**（右侧停靠）：算法显示窗口标签页叠加

### 5.2 设置面板各区域说明

v1.0.9 起设置面板采用 VSCode 风格分组，通过 ActivityBar 图标切换 QStackedWidget 页面。11 个面板归入 5 个分组：

| 分组 | 包含面板 | 内容 |
|------|---------|------|
| **Camera** | Devices, Information | 设备列表/连接/断开；传感器型号、分辨率、序列号、固件版本 |
| **Display & Stats** | Display, Statistics | 累积时间、帧率、FPS 上限、色彩主题；事件率（Mev/s）、ON/OFF 比例、数据速率、算法丢帧率 |
| **Hardware** | Biases, ROI, ESP, Trigger | Bias 滑块+预设；ROI 开关+坐标+掩码；Anti-Flicker/Trail Filter/ERC；Trigger In/Out |
| **Algorithms** | Preprocessing, Algorithms | OpenEB 预处理链（Polarity/Flip/ROI Filter）；自研算法选择+启停+参数+共享预处理（Noise/Downsample） |
| **Tools** | File Tools | 文件转换（→HDF5/CSV/DAT）、录制开始/停止、导出 AVI/HDF5 |

> 注：Calibration（内参标定向导）通过 Tools 菜单 → Intrinsic Wizard 打开，不在侧栏常驻。

### 5.3 其他窗口

- **XYT 3D 事件点云窗口**：3D x-y-t 事件点云（替代 Temporal Plot），可旋转/缩放/拖拽
- **算法参数对话框**：每个算法的独立参数调整面板
- **标定助手窗口**：内参标定引导
- **转换进度对话框**：RAW→HDF5/CSV/DAT 转换进度
- **导出进度对话框**：AVI / HDF5 导出
- **HAL 控制台**：`metavision_hal_showcase` 全功能 HAL 面板

### 5.4 菜单栏

v1.0.9 起菜单栏由 CustomTitleBar 实现（下拉菜单替代 QMenuBar）。Camera/Preprocess/Frame Mode/Algorithm/Calibration 菜单已移除，相关功能迁入侧栏对应面板。新增 Theme 顶级菜单。

| 菜单 | 选项 |
|------|------|
| **File** | Open File (RAW/HDF5/DAT), Save Config, Load Config, Save Biases, Load Biases, Algorithm Params (导出/导入), Exit |
| **View** | Toggle Playback Panel (Ctrl+Shift+P), Reset Layout, Save Layout, Load Layout, Fullscreen (F11) |
| **Theme** | Color: LightGray / LightBlue; Mode: Follow System / Always Light / Always Dark |
| **Tools** | Intrinsic Wizard (内参标定向导), Sharpness (锐度工具) |
| **Help** | About, About Qt |

> 注：侧栏切换由 ActivityBar 底部 chevron 按钮控制，不再通过 View 菜单。Camera Connect/Disconnect 在 Devices 面板，Preprocess 在 Preprocessing 面板，Algorithm 选择在 Algorithms 面板，录制/导出在 File Tools 面板。

### 5.5 快捷键

> **更新（v1.0.9）**：v1.0.9 重构后仅保留以下实际注册的快捷键。早期文档列出的 Ctrl+C/Space/←→/S/L/Ctrl+Shift+T/X/E/F/Ctrl+W 等均未实现，已从文档移除。

| 快捷键 | 功能 | 注册位置 |
|--------|------|----------|
| Ctrl+O | 打开文件 | main_window.cpp (`QKeySequence::Open`) |
| Ctrl+Q | 退出应用 | main_window.cpp (`QKeySequence::Quit`) |
| Ctrl+Shift+P | 命令面板（预留） | main_window.cpp |
| F11 | 全屏切换 | main_window.cpp |
| R | 开始/停止录制 | file_tools_panel.cpp |
| Ctrl+R | 切换 ROI 叠加显示 | roi_panel.cpp |

### 5.6 多窗口布局与并行显示

**核心需求**：GUI 支持多窗口可拖拽、可并行显示不同功能。新增窗口时自动重新布局，使各窗口并排可见；用户可手动拖拽调整位置与大小。

#### 5.6.1 算法显示模式分类

算法结果按显示方式分为三类，决定其窗口行为：

| 显示模式 | 说明 | 代表算法 | 行为 |
|----------|------|----------|------|
| **被动型 (Passive)** | 原地过滤事件，不绘制叠加层 | 噪声过滤、热像素过滤、超高速回放、带通滤波 | 不开新窗口；启用后主显示改为渲染其**输出事件流**（`filtered_events` 回注重渲染，单帧上限 ~50k 事件），过滤效果直接可见；AlgoWindow 打开时显示状态文字 |
| **叠加型 (Overlay)** | 结果叠加到主显示帧上 | 朝向/方向滤波、光流、团块检测、目标跟踪、角点、计数、霍夫直线/圆、EIS（电子稳定） | 不开新窗口，直接绘制在主显示区 |
| **主显示替换型 (Replace)** | 替换主显示区内容 | 事件重建（EventToVideo） | 占用主显示区 |
| **独立窗口型 (Standalone)** | 在新窗口独立显示 | Time Surface、XYT 3D 可视化、频率检测（FreqDetector）、ISI 直方图（ISI Analyzer）、Sensor 自检（SensorSelfTest） | 弹出独立子窗口 |

#### 5.6.2 多窗口布局行为

| 场景 | 布局行为 |
|------|----------|
| 仅主显示区 | 全屏显示主事件帧 |
| 新增 1 个独立窗口（如 Time Surface） | 自动一左一右二分：左=主显示，右=新窗口 |
| 新增第 2 个独立窗口（如 XYT） | 自动三分（左/中/右 或 上/下+右），保持主显示始终可见 |
| 新增第 N 个窗口（N≥3） | 切换为网格布局（grid），主显示区固定在左上角 |
| 用户拖拽窗口 | 可将任意子窗口拖出为浮动窗口，或停靠到边缘 |
| 用户调整大小 | 各窗口分隔条可拖动调整占比 |
| 关闭某窗口 | 剩余窗口自动重排填满空间 |

#### 5.6.3 窗口管理

| 功能 | 描述 |
|------|------|
| 窗口标题 | 每个子窗口显示其算法名 + 参数摘要（如 "Time Surface (decay=100ms)"） |
| 统一参数面板 | 所有算法参数仅在侧栏（AlgorithmsPanel）调节，显示窗口只展示算法输出，避免两处参数面板不同步的歧义 |
| 独立启停 | 每个子窗口可独立启停，不影响主显示与其他窗口 |
| 共享事件源 | 所有窗口共享同一事件流（实时相机或回放文件），各自独立处理 |
| 窗口列表 | View 菜单显示当前所有打开的子窗口，可勾选显示/隐藏 |
| 布局保存/恢复 | 可保存当前多窗口布局为预设，下次启动恢复 |

#### 5.6.4 布局示例

```
示例 A：主显示 + Time Surface（一键新增后自动二分）
┌──────────────────────┬──────────────────────┐
│                      │                      │
│   主显示区            │   Time Surface 窗口   │
│   (事件累积帧 +       │   (时间衰减伪彩图)    │
│    光流叠加)          │                      │
│                      │                      │
└──────────────────────┴──────────────────────┘

示例 B：主显示 + Time Surface + XYT（三分）
┌──────────────────────┬──────────┬───────────┐
│                      │  Time    │           │
│   主显示区            │  Surface │   XYT     │
│   (光流叠加)          │  窗口    │  窗口     │
│                      │          │           │
└──────────────────────┴──────────┴───────────┘

示例 C：4 个窗口（网格）
┌──────────────┬──────────────┐
│  主显示       │  Time Surface│
├──────────────┼──────────────┤
│  XYT 窗口     │  事件重建     │
└──────────────┴──────────────┘
```

#### 5.6.5 实现技术

基于 Qt 6 的 `QDockWidget` 实现可停靠/浮动子窗口；主窗口采用 `QMainWindow`，中央区域为主显示（EventDisplayWidget）。设置面板（SettingsPanel）停靠在**左侧**，包含 ActivityBar（48px 图标列）和 QStackedWidget（分组页面切换）；算法窗口（AlgoWindow）停靠在**右侧**并可标签页叠加。标题栏由 CustomTitleBar 实现（下拉菜单替代 QMenuBar）。工具栏已移除。侧栏收起时由 ActivityBar 底部 chevron 按钮控制可见性（收起后 dock 缩至 48px 仅显示图标列）。ActivityBar 空白区域支持拖拽移动 dock 到对侧（OpenHandCursor/ClosedHandCursor 反馈）。布局自动重排通过 `QMainWindow.resizeDocks` / `tabifyDockWidget` 实现。

#### 5.6.6 🆕 算法 ROI 处理区（全算法支持，128×128 中心默认）

**核心需求**：全部 28 个自研算法注册（20 个 CV + 8 个 Analytics）均支持"算法 ROI"处理区；noise_filter 模块仍存在于 algo/cv/ 但 v1.0.9 起从独立算法改为共享预处理阶段。GUI 默认启用 ROI 且默认区域为中心 128×128 内侧，使算法只处理 ROI 内事件、GUI 只渲染 ROI 区域的输出。用户可在侧栏"算法模块"面板顶部的全局 ROI 选择器中调节 ROI 坐标/尺寸，并在每个算法的展开参数编辑器中调节全部算法参数（AlgoWindow 显示窗口仅展示输出，不含参数控件）。**例外**：§4.4.8 Sensor 自检（SensorSelfTest）必须覆盖全 sensor 以检测每个像素，注册时不追加 ROI/预处理参数。

**适用算法**：全部 28 个自研算法（包括 Overlay / Replace / Standalone / Passive 四种显示模式）。除 SensorSelfTest 外，每个算法在 `algo_bridge.cpp` 注册时由 `add()` lambda 自动追加 5 个 ROI 参数；每个 `AlgoBackend` 通过 `RoiFilter` helper（或等价的 `ProcessRegion` 成员）实现事件过滤。

**两种后端过滤模式**：
- **In-place compaction**（用于输出事件向量的滤波器，Group A/F/G）：在原缓冲区上将 ROI 外事件覆盖式压缩，算法仅处理压缩后的事件子集；`filtered_events` 输出仅含 ROI 内事件
- **Apply / keep-coords**（用于 Overlay 检测器、分析器、帧生成器，Group B/C/D/E/H）：将 ROI 内事件拷贝到独立缓冲区供算法处理，`filtered_events` 输出全部事件（passthrough），叠加层坐标保持传感器尺度

**参数（28 个算法共用同一组参数键，自动追加；SensorSelfTest 例外，无参数）**：

| 参数 | 类型 | 默认 | 范围 | 说明 |
|------|------|------|------|------|
| `roi_enabled` | bool | `true` | — | 是否启用算法 ROI |
| `roi_x` | int | `-1` | `[-1, sensor_w]` | ROI 左上角 x，`-1` 表示自动居中 |
| `roi_y` | int | `-1` | `[-1, sensor_h]` | ROI 左上角 y，`-1` 表示自动居中 |
| `roi_w` | int | `128` | `[0, sensor_w]` | ROI 宽度，`0` 表示全幅 |
| `roi_h` | int | `128` | `[0, sensor_h]` | ROI 高度，`0` 表示全幅 |

**计算规则**（`ProcessRegion::compute`）：
1. `rw = (roi_w ≤ 0) ? sensor_w : min(roi_w, sensor_w)`，`rh` 同理
2. `rx = (roi_x < 0) ? (sensor_w - rw) / 2 : clamp(roi_x, 0, sensor_w - rw)`
3. `ry` 同理
4. 钳位后 `x0=rx, y0=ry, x1=min(rx+rw, sensor_w), y1=min(ry+rh, sensor_h)`，最终 `rw=x1-x0, rh=y1-y0`

**事件过滤**：每个事件 `(x, y, p, t)` 若 `x0 ≤ x < x1 && y0 ≤ y < y1` 则保留（坐标平移与否取决于过滤模式），否则丢弃。

**GUI 行为（侧栏 AlgorithmsPanel）**：
1. **算法选择**：侧栏 Algorithms 组的 AlgorithmsPanel 提供算法列表，每个算法有 "Enable" 复选框和参数编辑器。勾选 "Enable" 时启用 AlgoInstance 并自动打开 AlgoWindow（右侧停靠）；取消勾选时关闭 AlgoWindow 并停用算法
2. **全局 ROI 选择器**：AlgorithmsPanel 顶部提供全局 ROI 控件（x/y/w/h + enabled 复选框），默认中心 128×128。ROI 参数仅应用于 `source == "self"` 的自研算法；OpenEB wrapper 算法（`source == "openeb"`）的 ROI 由 OpenEB filter_chain 处理，跳过以避免污染其 param_values_ 映射（BUG-13 修复）
3. **共享预处理**：AlgorithmsPanel 提供 "Preprocessing (ROI > filter > downsample > undistort)" 控件，包含 Noise filter（8 模式，默认 STCF）、1/4 Downsample（**默认 OFF**——对多数后端它只是抽稀事件（坐标不变），默认开启会静默损失 3/4 输入；仅 E2VID/ISI/TimeSurface/Hough 后端真正坐标减半，见 doc/systematic_audit.md §五-F1）和 Undistort（默认 OFF，见 §4.5.3）。预处理参数仅推送到自研算法实例，与主算法不互斥（可叠加）
4. **使用 `toggled(bool)` 信号 + `QSignalBlocker`** 避免程序化 `setChecked` 触发重入（v1.0.9 实际实现）
5. **算法互斥**：AlgorithmsPanel 对自研算法（`source == "self"`）实行互斥模式——同时仅允许启用一个算法实例。启用新算法时自动停用前一个；OpenEB wrapper 算法不受此限制
6. **AlgoWindow**（`gui/widgets/algo_window.h`，继承 `QDockWidget`）：
   - 停靠在主窗口**右侧**，多个算法窗口标签页叠加；用户可拖拽至任意边缘、浮出或重新排列
   - 内部 content widget 仅为显示区域，**不含参数控件**——所有参数调节统一在侧栏 AlgorithmsPanel 完成
   - Standalone 帧类算法安装 `EventDisplayWidget` 显示算法输出帧；自研 Overlay 算法也安装 `EventDisplayWidget`，作为 ROI 放大视图
   - 关闭 AlgoWindow → `closeEvent` 显式 accept → 触发 `closing` 信号 → `set_enabled(false)` + 侧栏算法面板取消勾选 + `WA_DeleteOnClose` 自动回收
7. **主显示帧 ROI 叠加**：`process_algo_results()` 中调用 `draw_roi_overlays()`，遍历所有启用的自研算法，对每个 `roi_enabled=true` 的算法在主显示帧上绘制黄色矩形框 + 算法名标注
8. **ROI 放大视图**（Overlay 算法）：Overlay 分支在主帧上绘制完算法图元后，若该算法 `roi_enabled=true` 且其 AlgoWindow 已开，则从已标注的主帧裁剪 ROI 区域并推送到 AlgoWindow 的 `EventDisplayWidget`
9. **模式切换保护**：`first_init_` 标志确保 ROI/fps 仅在初始构建时自动设置，用户手动切换算法模式时不重置自定义 ROI/fps（BUG-14 修复）

**手动停用方式**：
- 侧边栏"算法模块"面板 → 取消勾选算法的 "Enable" 复选框
- 关闭对应 AlgoWindow（dock 标签页 X 按钮）
- 取消勾选 "算法ROI"（保留全幅处理）或在侧栏全局 ROI 选择器中调整 ROI 尺寸为 `0`（全幅）

---

#### 5.6.7 🆕 事件涌入保护（Flood Guard / 背压）

**核心需求**：当事件率瞬时飙升（如 10–100 Mev/s，常见于强光闪烁、相机抖动、bias 失配等场景），慢算法（如 `event_to_video` 的 Chambolle-Pock 重建、`hough_circle` 的 3D 累加器扫描）无法跟上事件速率；若无背压，算法内部缓冲区将无界增长，最终导致 GUI 线程阻塞、内存耗尽或进程崩溃。

**保护机制**（`AlgoInstance`，`gui/algo_bridge/algo_bridge.h`，**事件速率制**）：

1. **滑窗测速**：`push_events()` 按墙钟维护 1 秒滑动窗口，累计窗口内事件数；窗口满 1 秒时计算事件速率（events/s）。批次不再截断——在保护触发前每个事件都完整投递。用墙钟（而非批大小或事件时间戳）使文件回放与实况流适用同一阈值（原按"每批 5 万事件"的实现会在文件播放时误杀，见 doc/systematic_audit.md §五-E1）。
2. **连续超限判定**：窗口速率 ≤ `kMaxEventRateEvPerSec`（30 Mev/s）时重置 `flood_strikes_ = 0`（偶发尖峰不触发停用）。
3. **自动停用阈值**：连续 `kFloodStrikes`（默认 `4`，即持续约 4 秒超 30 Mev/s）个窗口超限，设置 `overloaded_ = true` 并将 `enabled_ = false`，算法实例停止处理事件，并通过 overload 回调通知 AlgorithmsPanel 同步取消侧栏复选框勾选。
4. **用户重新启用**：用户在侧边栏"算法模块"面板重新勾选 "Enable" 时，`set_enabled(true)` 同时清零 `overloaded_`、`flood_strikes_` 与速率窗口，算法从干净状态重启（后端内部缓冲/累加器状态保留，维持暂停-恢复语义）；禁用时调用后端 `release_resources()` 释放重量级资源（E2VID ONNX session、大帧缓冲），重新启用后懒重建。

**GUI 反馈**：
- `is_overloaded()` 暴露停用原因，`process_algo_results()` 在遍历 live 实例时检测到该状态，于状态栏显示 "auto-disabled: event rate too high (re-enable to retry)"，并在对应 AlgoWindow 的状态标签显示 "AUTO-DISABLED: event flooding detected. Re-enable from the sidebar."
- 侧边栏 "Enable" 复选框因 `enabled_=false` 自动取消勾选，与用户手动停用行为一致。

**XYT 缓冲区硬上限**：`XYTVisualizer`（`algo/cv/xyt_visualizer.h`）额外维护 `kMaxBuffer = 200000` 的环形缓冲区硬上限，即使 flood guard 仍允许事件通过，缓冲区超出上限时也会丢弃最旧事件（`pop_front`），防止 3D 点云内存无界增长。

**保护范围**：所有 28 个自研算法均通过 `AlgoInstance` 统一获得 flood guard 保护；OpenEB 滤波/变换阶段在 FilterChain 内处理，不受此机制约束。

---

## 六、数据流设计

### 6.1 实时相机数据流

```
相机硬件
  → HAL (硬件抽象层)
    → Stream 模块 (事件缓冲区)
      → Event Callback → gui/ 显示模块 (OpenGL 累积帧渲染)
      → Event Callback → gui/ 统计模块 (事件率计算)
      → Event Callback → gui/algo_bridge/ → algo/cv (噪声过滤→光流→跟踪…)
      → Event Callback → gui/algo_bridge/ → algo/analytics (主动标记/事件重建)
      → Event Callback → gui/ 录制模块 (写入 RAW 文件)
```

### 6.2 回放数据流

```
RAW/HDF5 文件
  → algo/common/data_loader (EventIterator)
    → gui/ 显示模块 (帧渲染 + 回放控制)
    → gui/algo_bridge/ → algo/ 算法模块 (离线分析)
    → gui/ 统计模块
```

### 6.3 导出数据流

```
RAW/HDF5 文件
  → algo/common/data_loader
    → algo/common/frame_generator (累积窗口)
      → AVI Encoder → 输出 .avi
      → HDF5 Writer → 输出 .h5
```

---

## 七、非功能需求

### 7.1 性能需求

| 指标 | 要求 |
|------|------|
| 事件处理吞吐量 | ≥ 10 Mev/s（典型 EVK3 HD 事件率） |
| 显示帧率 | ≥ 30 fps |
| GUI 响应延迟 | 参数修改到反馈 < 100ms |
| 录制无丢帧 | 在目标事件率下 100% 事件写入 |

### 7.2 平台支持

| 平台 | 支持 |
|------|------|
| Ubuntu 20.04 | ✓ |
| Ubuntu 22.04 | ✓ |
| Ubuntu 24.04 | ✓ |
| Windows 10/11 | 计划中 |

### 7.3 国际化

界面语言为纯英文（English-only）。v1.0.9 起不再提供多语言切换，所有 GUI 文本、面板分组名、菜单项均为英文。Qt `tr()` 调用保留为标准实践，但未配置 `.ts`/`.qm` 翻译文件。

### 7.4 容错处理

| 场景 | 处理 |
|------|------|
| 相机断开 | 提示用户，停止录制，保留已录制数据 |
| 磁盘空间不足 | 录制前检查可用空间，录制中空间不足时警告并停止 |
| 传感器不支持的特性 | 灰度禁用不支持的 Bias/ESP 参数，显示传感器型号提示 |
| 无效配置文件 | 加载时校验并提示具体错误 |

---

## 八、开发路线图

### Phase 1：核心框架 (gui/ + algo/ MVP)
- [ ] CMake 项目骨架搭建（openeb/ + gui/ + algo/）
- [ ] 相机发现与连接（Qt + HAL）
- [ ] 实时事件显示（OpenGL 累积帧渲染）
- [ ] 基础显示参数（累积时间、色彩主题）

### Phase 2：相机控制 (gui/)
- [ ] Bias 控制面板（全部参数）
- [ ] ROI 设置（拖拽 + 数值输入）
- [ ] ESP 模块（Anti-Flicker、Trail Filter、ERC）
- [ ] Trigger 接口配置

### Phase 3：录制与回放 (gui/)
- [ ] 实时录制（RAW 格式）
- [ ] 文件回放（多倍速、循环、进度条）
- [ ] 录制裁剪
- [ ] 统计信息面板

### Phase 4：数据导出与配置 (gui/)
- [ ] 导出 HDF5
- [ ] 导出 AVI
- [ ] 配置保存/加载（JSON）
- [ ] 预设管理

### Phase 5：openEB 能力封装 (gui/algo_bridge/)
- [ ] 事件过滤链（ROI/极性/翻转/旋转/转置/缩放/速率分割）
- [ ] 7 种帧生成模式封装与切换
- [ ] 5 种事件张量预处理器封装
- [ ] 工具封装（RateEstimator/FrameComposer/VideoWriter/DataSync/TimingProfiler）
- [ ] 文件转换工具集成（→HDF5/CSV/DAT、裁剪、编码、信息）

### Phase 6：公共工具与基础设施 (algo/common/)
借鉴 jAER 公共数据结构与滤波器框架，建立自研算法的共享基建（详见 4.2，共 20 个模块）：
- [x] `event.h` / `event_packet.h`：事件 POD 与零拷贝包
- [x] `event_buffer.h` / `lifo_event_buffer.h`：环形缓冲 + LIFO 栈
- [x] `frame_generator.h` / `dvs_framer.h`：帧生成与事件分帧（纯事件 ON/OFF 计数）
- [x] `freme.h`：FREME 频率表示模板
- [x] `data_loader.h`：HDF5/RAW 数据加载（内存映射加速）
- [x] `event_rate_estimator.h` / `performance_meter.h` / `time_limiter.h`：性能监测
- [x] `kalman_filter.h` / `kmeans.h` / `particle_filter.h` / `periodic_spline.h`：滤波/聚类/平滑
- [x] `histogram_ring_buffer.h` / `lif_integrator.h`：直方图与 LIF 神经元
- [x] `filter/`：IIR 低通/高通/带通/角度/中位数滤波器集

### Phase 7：基础噪声过滤与运动分析 (algo/cv/ 8 个，对应 4.3.5–4.3.12)
借鉴 jAER `filter/`、`label/`、`tracking/`、`projects/rbodo/`：
- [x] 4.3.5 噪声过滤（8 模式：BAF/STCF/Refractory/DWF/AgePolarity/Harmonic/Repetitious/SpatialBP）
- [x] 4.3.6 热像素过滤（学习 + 查表 + FPN 概率校正）
- [x] 4.3.7 朝向边缘检测（4 朝向，`AbstractOrientationFilter`）
- [x] 4.3.8 方向选择性滤波（8 方向 + 全局模式，`AbstractDirectionSelectiveFilter`）
- [x] 4.3.9 光流估计（4 模式：LocalPlanes/LucasKanade/BlockMatch/ClusterOF）
- [x] 4.3.10 团块检测（连通域 + Histogram2DFilter 背景建模）
- [x] 4.3.11 事件级目标跟踪（4 模式：RCT/Median/Kalman/MultiHypothesis）
- [x] 4.3.12 角点检测（3 模式：EndStopped/TypeCoincidence/Harris）

### Phase 8：高级几何与检测 (algo/cv/ 10 个，对应 4.3.13–4.3.23)
借鉴 jAER `ELiSeD`、`HoughLineTracker`、`HoughCircleTracker`、`OpticalFlowGyroTracker` 等：
- [x] 4.3.13 ELiSeD 线段检测
- [x] 4.3.14 霍夫直线跟踪
- [x] 4.3.15 霍夫圆跟踪
- [x] 4.3.17 方向共识过滤
- [x] 4.3.18 LIF 神经元网格聚类
- [x] 4.3.19 背景掩码学习
- [x] 4.3.20 透视去畸变（LUT 查表）
- [x] 4.3.21 触发同步过滤
- [x] 4.3.22 IIR 带通滤波
- [x] 4.3.23 电子稳定 EIS（OpticalGyro）

### Phase 9：显示工具 (algo/cv/ 4 个，对应 4.3.24–4.3.27)
- [x] 4.3.24 超高速等效回放（时间膨胀，≥200,000 fps）
- [x] 4.3.25 XYT 3D 事件点云（独立窗口，替代 Temporal Plot）
- [x] 4.3.26 可视化叠加（Overlay）
- [x] 4.3.27 Time Surface 窗口（独立窗口，openEB `TimeSurfaceProcessor` 封装）

### Phase 10：分析模块 (algo/analytics/ 8 个)
借鉴 jAER `DvsFramer`、`particlecounter`、`freq_analyzer` 等：
- [x] 4.4.1 主动标记跟踪（滑动窗口聚类 + 事件数颜色映射）
- [x] 4.4.2 事件→灰度图像重建（非 DL：BardowVariational 完整复现（联合光流+亮度估计，λ1-6，Chambolle-Pock，λ6=1.0 论文值 + 条件先验仅作用于无新事件像素）/ InteractingMaps 严格复现（六图交替松弛 I/G/V/F/C/R + 旋转最小二乘 + Poisson 梯度积分，带 V/F/R/I 钳位与首帧温启动稳定性修复）/ DL：E2VID 已移植且为默认模式，含 ONNX Runtime 推理 + 启发式回退）
- [x] 4.4.3 光流评估（EPE/PE/角度误差）
- [x] 4.4.4 ISI 直方图分析
- [x] 4.4.5 通用颗粒计数器
- [x] 4.4.6 自适应 Bias 控制（PID 反馈）
- [x] 4.4.7 闪烁光源频率检测（热图 + FFT，参考 `freq_analyzer`）
- [x] 4.4.8 Sensor 自检（坏点检测 + 不应期估计，硬件面板触发，关闭弹报告）

### Phase 11：标定模块 (algo/calibration/ 1 个)
- [x] 4.5.1 内参标定（含事件去畸变 LUT 预计算，借鉴 jAER `SingleCameraCalibration`）

### Phase 12：测试基建 (algo/tests/)
- [x] 4.6.0 GTest + CTest 单元测试框架（覆盖 Phase 6–10 模块 + 真实 RAW 回归；4.6.1/4.6.2 降噪评测框架因零引用已移除）

### Phase 13：完善
- [x] 3D XYT 事件点云窗口（替代 Temporal Plot，VBO+GLSL）
- [x] 多窗口可拖拽并行显示布局（QDockWidget/QMdiArea）
- [ ] 窗口布局保存/恢复
- [ ] 多语言支持（中/英）
- [ ] 性能优化（≥10 Mev/s 吞吐）
- [ ] 算法参数保存/加载

---

> 参考资料：  
> [Prophesee Metavision SDK Docs 5.3.1](https://docs.prophesee.ai/stable/)  
> [Metavision Studio](https://docs.prophesee.ai/stable/metavision_studio/)  
> [SDK Modules](https://docs.prophesee.ai/stable/modules.html)  
> [Applications](https://docs.prophesee.ai/stable/applications.html)  
> [Biases](https://docs.prophesee.ai/stable/hw/manuals/biases.html)  
> [Metavision SDK 5 PRO - 产品页](https://www.prophesee-cn.com/metavision-sdk-pro/)
