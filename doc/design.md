# GUI-for-openEB 需求分析文档

> 版本：1.0  
> 日期：2026-06-30  
> 技术栈：C++17 + Qt 6  
> 基于：OpenEB SDK v5.2.0（Apache 2.0）  
> 参考：Metavision Studio（Prophesee Docs 5.3.1）、Metavision SDK Pro 产品页

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
├── openeb/                   # openEB 子树（Apache 2.0，v5.2.0）
├── gui/                      # GUI 应用代码（C++ + Qt 6）
│   ├── CMakeLists.txt
│   ├── main.cpp              # 应用入口
│   ├── main_window.h / .cpp  # 主窗口
│   ├── widgets/              # 通用 GUI 控件
│   ├── panels/               # 设置面板（Biases/ROI/ESP/Trigger 等）
│   ├── display/              # 事件显示渲染（OpenGL 加速）
│   ├── recorder/             # 录制/回放控制
│   ├── exporter/             # 数据导出
│   ├── config/               # 配置序列化（JSON）
│   ├── stats/                # 统计面板
│   ├── temporal/             # Temporal Plot 窗口
│   └── algo_bridge/          # 算法桥接（调用 algo/ 模块）
│       ├── algo_bridge.h
│       └── algo_bridge.cpp
├── algo/                     # 算法模块（C++，被 gui/ 调用）
│   ├── CMakeLists.txt
│   ├── common/               # 公共工具
│   │   ├── event_buffer.h / .cpp     # 高性能事件缓冲区
│   │   ├── frame_generator.h / .cpp  # 帧生成器（多窗口累积）
│   │   └── data_loader.h / .cpp      # HDF5/RAW 数据加载
│   ├── cv/                   # 计算机视觉与运动分析
│   │   ├── noise_filter.h / .cpp     # 噪声过滤（时空邻域/活动像素）
│   │   ├── sparse_optical_flow.h / .cpp   # 稀疏光流（平面拟合）
│   │   ├── dense_optical_flow.h / .cpp    # 密集光流
│   │   ├── blob_detector.h / .cpp     # 团块检测
│   │   ├── object_tracker.h / .cpp    # 事件级目标跟踪（DBSCAN+Kalman）
│   │   ├── corner_detector.h / .cpp   # 角点检测与跟踪（Harris+SAE）
│   │   ├── counter.h / .cpp           # 高速计数（检测线）
│   │   ├── ultra_slow_motion.h / .cpp # 超高速等效回放（时间膨胀）
│   │   ├── xyt_visualizer.h / .cpp    # XYT 时空连续可视化
│   │   ├── stereo_matcher.h / .cpp    # 立体匹配（半稠密深度图）
│   │   ├── time_surface.h / .cpp      # Time Surface 窗口（独立窗口型）
│   │   └── overlay.h / .cpp           # 算法结果可视化叠加
│   ├── analytics/            # 分析模块
│   │   ├── active_marker.h / .cpp     # 主动标记跟踪（调制光方案）
│   │   └── event_to_video.h / .cpp    # 事件→灰度图像重建（E2VID）
│   └── calibration/          # 相机标定
│       ├── intrinsic.h / .cpp         # 内参标定
│       └── extrinsic.h / .cpp         # 外参标定（多相机）
├── LICENSE
├── README.md
├── README_CN.md
└── doc/
    └── design.md
```

### 1.4 适用范围

- **gui/**：相机控制、事件可视化（OpenGL 渲染）、数据录制/回放/导出、配置管理（JSON）、ESP、Trigger
- **algo/cv/**（自研）：噪声过滤、稀疏/密集光流、团块检测、事件级跟踪、角点检测、高速计数、超高速回放、XYT 可视化、立体匹配、可视化叠加
- **algo/analytics/**（自研）：主动标记跟踪、事件→灰度图像重建
- **algo/calibration/**（自研）：内参标定、外参标定
- **gui/algo_bridge/**（封装复用）：直接调用 openEB 已有的 27 项算法/处理器/工具（ROI 过滤、极性过滤、7 种帧生成、5 种预处理器等，见 1.5 节），不重复实现
- **明确不在范围内**：ML 训练管线、ML 检测/分类/光流推理（Detection/Gesture/OpticalFlow Inference）、CV3D 边缘跟踪、企业级监测（颗粒/振动/飞溅）

> 注：`algo/analytics/event_to_video` 虽基于 E2VID 神经网络推理，但定位为"事件→灰度图像重建"分析工具（非通用 ML 应用），故保留在范围内，依赖项见 4.4.2。

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

---

## 二、系统总体架构

### 2.1 分层架构

```
┌──────────────────────────────────────────────────────┐
│                  GUI Layer (gui/ — C++ / Qt 6)       │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────────┐  │
│  │ 主窗口    │ │ 设置面板  │ │ 分析工具窗口          │  │
│  │(显示区)   │ │(控制区)   │ │(Temporal/Algo)       │  │
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
│  │12自研│ │ 2自研 │ │  2自研  │ │   3公共   │      │
│  └──────┘ └──────┘ └──────────┘ └────────────┘      │
│  (openEB 27项能力由 algo_bridge 直接封装复用)         │
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

#### 3.2.3 时间图（Temporal Plot）

| 参数 | 描述 |
|------|------|
| plot_axis | 绘制轴线（X 方向 / Y 方向） |
| axis_position | 选定轴的像素坐标位置 |
| accumulation_time_ms | 绘图累积时间（ms） |
| time_window_ms | 显示时间窗口（ms） |
| show_polarity | 是否区分 ON/OFF 极性着色 |

**功能**：在独立窗口中显示 x-t 或 y-t 散点图，横轴为时间，纵轴为像素列/行位置，用于分析事件时空分布、传感器调优。

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
- 数字事件掩码设置

---

### 3.8 算法桥接模块 (gui/algo_bridge/)

**职责**：`gui/` 中负责调用 `algo/` 中 C++ 算法模块的桥接层。只做参数传递和结果回传，不实现算法逻辑。所有算法实现统一在 `algo/` 目录（详见第四章）。

**桥接接口**：

```cpp
// algo_bridge.h — 统一算法调用接口
class AlgoBridge {
public:
    // 查询可用算法列表
    std::vector<AlgoInfo> listAlgos() const;

    // 创建算法实例（按名称）
    std::shared_ptr<AlgoInstance> create(const std::string& name);

    // 推送事件到算法实例
    void pushEvents(std::shared_ptr<AlgoInstance> inst,
                    const std::vector<EventCD>& events);

    // 拉取算法处理结果
    AlgoResult pullResult(std::shared_ptr<AlgoInstance> inst);
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
| **algo/common/** | 3 | 事件缓冲区、帧生成器、数据加载器（公共基础设施） |
| **algo/cv/** | 12 | 🆕 全部自研：噪声过滤、稀疏光流、密集光流、团块检测、事件级跟踪、角点检测、高速计数、超高速回放、XYT 可视化、立体匹配、Time Surface 窗口、可视化叠加 |
| **algo/analytics/** | 2 | 🆕 主动标记跟踪、事件→灰度图像重建 |
| **algo/calibration/** | 2 | 🆕 内参标定、外参标定 |
| **gui/algo_bridge/** | （封装层） | 🔄 直接调用 openEB 27 项已有能力（9 事件过滤 + 7 帧生成 + 5 预处理器 + 6 工具），不重复实现 |

> **总计**：自研 19 个算法模块 + 封装复用 openEB 27 项能力；openEB 全部 89 项内置功能（见 1.5 节）均需在 GUI 中可访问。

---

### 4.2 algo/common/ — 公共工具

| 模块 | 功能 | 关键设计 |
|------|------|----------|
| `event_buffer.h/.cpp` | 环形缓冲区，无锁读写，支持多线程并发推送/拉取事件 | `std::atomic`, cache-line padding |
| `frame_generator.h/.cpp` | 将事件流按累积时间窗口生成帧（`cv::Mat`），支持多窗口并行 | 模板化窗口策略 |
| `data_loader.h/.cpp` | 读取 RAW/HDF5 文件，提供 `EventIterator` 接口 | 内存映射加速大文件 |

---

### 4.3 algo/cv/ — 计算机视觉与运动分析

> **符号说明**：🔄 = 由 `gui/algo_bridge/` 直接调用 openEB 已有类（`openeb/sdk/modules/core/`），不重复实现，不在 `algo/cv/` 建单独文件；🆕 = openEB 未提供，在 `algo/cv/` 自研实现

#### 4.3.1 🔄 事件过滤与预处理（algo_bridge 封装 openEB Core 算法）

以下能力 openEB 已完整提供，由 `gui/algo_bridge/` 实例化并暴露参数给 GUI，无需在 `algo/cv/` 重复实现：

| GUI 选项名 | 调用的 openEB 类 | 参数 |
|-----------|-----------------|------|
| ROI Filter | `RoiFilterAlgorithm` | `x0, y0, x1, y1` |
| ROI Mask | `RoiMaskAlgorithm` | 掩码图像路径 |
| Polarity Filter | `PolarityFilterAlgorithm` | `polarity`（ON/OFF） |
| Polarity Invert | `PolarityInverterAlgorithm` | 无 |
| Flip X / Flip Y | `FlipXAlgorithm` / `FlipYAlgorithm` | 传感器尺寸 |
| Rotate | `RotateEventsAlgorithm` | 角度（0/90/180/270） |
| Transpose | `TransposeEventsAlgorithm` | 无 |
| Rescale | `EventRescalerAlgorithm` | `scale_x, scale_y` |
| Adaptive Rate Split | `AdaptiveRateEventsSplitterAlgorithm` | `target_rate`, `window_size` |

#### 4.3.2 🔄 帧生成（algo_bridge 封装 openEB Frame Generation 算法）

| GUI 选项名 | 调用的 openEB 类 | 参数 |
|-----------|-----------------|------|
| Integration Frame | `EventsIntegrationAlgorithm` | `accumulation_time_us` |
| Diff Frame | `EventFrameDiffGenerationAlgorithm` | `accumulation_time_us` |
| Histogram Frame | `EventFrameHistoGenerationAlgorithm` | `accumulation_time_us` |
| Time Decay Frame | `TimeDecayFrameGenerationAlgorithm` | `decay_time_us`, `accumulation_time_us` |
| Contrast Map | `ContrastMapGenerationAlgorithm` | `accumulation_time_us` |
| Periodic Frame | `PeriodicFrameGenerationAlgorithm` | `period_us` |
| On-Demand Frame | `OnDemandFrameGenerationAlgorithm` | 触发策略 |

注：每种帧生成算法产生不同的可视化效果，用户应在 GUI 中选择帧生成模式并调参。

#### 4.3.3 🔄 事件张量预处理器（algo_bridge 封装 openEB Preprocessors）

这些是事件→张量的转换器，可输出给模型或作为中间表示：

| GUI 选项名 | 调用的 openEB 类 | 参数 |
|-----------|-----------------|------|
| Diff Preprocessor | `DiffProcessor` | `accumulation_time_us` |
| Histo Preprocessor | `HistoProcessor` | `max_events_per_pixel` |
| Time Surface | `TimeSurfaceProcessor` | `decay_time_us` |
| Event Cube | `EventCubeProcessor` | `num_bins` |
| Preprocessor Factory | `EventPreprocessorFactory` | JSON 配置文件路径 |

#### 4.3.4 🔄 工具（algo_bridge 封装 openEB Utils）

| GUI 选项名 | 调用的 openEB 类 | 功能 |
|-----------|-----------------|------|
| Rate Estimator | `RateEstimator` | 实时事件率估计（Mev/s） |
| Frame Composer | `FrameComposer` | 多帧叠加合成 |
| Rolling Buffer | `RollingEventBuffer` | 滑动窗口事件缓冲 |
| Video Writer | `VideoWriter` / `CvVideoRecorder` | AVI/MP4 录制 |
| Data Synchronizer | `DataSynchronizerFromTriggers` | 外部触发数据同步 |
| Timing Profiler | `TimingProfiler` | 性能剖析 |

#### 4.3.5 🆕 噪声过滤 (NoiseFilter) — 叠加型

openEB 未提供专用事件级噪声滤波器，需自研。作用于事件流，过滤后事件送显示/下游算法。

| 算法 | 方案 | 参数与合法范围 |
|------|------|------|
| 时空邻域滤波 | 在 (Δx, Δy, Δt) 窗口内检查事件密度，滤除孤立事件 | `time_window_us`：int，范围 `[1000, 100000]`，默认 `5000`；`spatial_radius_px`：int，范围 `[1, 20]`，默认 `5`；`min_neighbors`：int，范围 `[1, 10]`，默认 `2` |
| 活动像素滤波 | 统计每像素历史事件率，利用 openEB `CounterMap` | `observation_window_s`：float，范围 `[0.1, 10.0]`，默认 `1.0`；`rate_threshold_hz`：float，范围 `[1, 10000]`，默认 `100` |

#### 4.3.6 🆕 光流估计 (OpticalFlow) — 叠加型

openEB 未提供光流算法，需自研。结果以箭头/颜色图叠加到主显示帧。

| 模式 | 算法方案 | 参数与合法范围 |
|------|----------|------|
| 稀疏 (SparseOF) | 局部事件簇 (x,y,t) 时空平面拟合，法向量=运动矢量 | `time_window_us`：int，`[1000, 100000]`，默认 `10000`；`spatial_radius_px`：int，`[3, 30]`，默认 `8`；`min_events_per_cluster`：int，`[3, 100]`，默认 `10` |
| 密集 (DenseOF) | 滑动窗口 + 局部平面拟合全覆盖 | `block_size`：int，`[4, 64]`，默认 `16`；`step`：int，`[1, 32]`，默认 `8`；`time_window_us`：同稀疏 |
| 快速 (FastOF) | 降采样 + 大时间窗口 | `downsample_factor`：int，`[1, 8]`，默认 `2`；`time_window_us`：`[1000, 200000]`，默认 `20000` |

#### 4.3.7 🆕 团块检测 (BlobDetector) — 叠加型

结果以 bbox 叠加到主显示帧。

| 算法 | 方案 | 参数与合法范围 |
|------|------|------|
| 连通域分析 | openEB 帧 → cv::threshold → cv::connectedComponents | `accumulation_ms`：float，`[1, 1000]`，默认 `33.3`；`threshold`：int，`[1, 254]`，默认 `50`；`min_area`：int，`[1, 100000]`，默认 `10` |
| 自适应背景 | 利用 openEB `RollingEventBuffer` + 背景减除 | `learning_rate`：float，`(0, 1]`，默认 `0.05`；`time_window_s`：float，`[0.1, 10]`，默认 `1.0` |

#### 4.3.8 🆕 事件级目标跟踪 (ObjectTracker) — 叠加型

结果以 bbox+ID+轨迹叠加到主显示帧。

| 子模块 | 方案 |
|--------|------|
| 事件聚类 | 时空密度在线聚类（DBSCAN 变体），利用 openEB `RollingEventBuffer` |
| 多目标关联 | 匈牙利算法 + 卡尔曼运动预测 |
| 轨迹管理 | 平滑滤波、ID 稳定性维护 |
| **输出** | `vector<TrackedObject>` — ID, bbox, velocity, trajectory |

**参数与合法范围**：`cluster_time_us`：int，`[1000, 50000]`，默认 `5000`；`cluster_radius_px`：int，`[3, 50]`，默认 `10`；`min_cluster_events`：int，`[10, 500]`，默认 `50`；`max_lost_age_s`：float，`[0.1, 5.0]`，默认 `1.0`

#### 4.3.9 🆕 角点检测与跟踪 (CornerDetector) — 叠加型

SAE + Harris 角点检测 → 最近邻匹配跟踪 → 轨迹过滤。结果以角点标记+轨迹叠加。

**参数与合法范围**：`accumulation_ms`：float，`[1, 100]`，默认 `10`；`harris_threshold`：float，`(0, 0.1]`，默认 `0.01`；`track_radius_px`：int，`[1, 30]`，默认 `5`；`min_track_len`：int，`[1, 100]`，默认 `10`；`output_hz`：int，`[10, 500]`，默认 `100`

#### 4.3.10 🆕 高速计数 (Counter) — 叠加型

检测线穿越计数，利用 openEB `RateEstimator` 辅助统计。检测线与计数文本叠加。

**参数与合法范围**：检测线由用户在主显示区两点拖拽定义（自动转为线段端点）；`direction`：枚举（双向/仅正向/仅反向）；`debounce_ms`：float，`[0, 1000]`，默认 `100`

**输出**：计数、计数率（objects/s）。

#### 4.3.11 🆕 超高速等效回放 (UltraSlowMotion) — 主显示替换型

时间戳膨胀重渲染，等效 ≥200,000 fps。替换主显示区帧（不叠加）。

**参数与合法范围**：`dilation_factor`：float，`[1, 10000]`，默认 `10`；`min_accumulation_us`：int，`[1, 1000]`，默认 `5`（=200,000 fps）

#### 4.3.12 🆕 XYT 时空可视化 (XYTVisualizer) — 独立窗口型

X-T / Y-T 散点图，时间缩放，分析线联动。在新窗口显示，不占用主显示区。

**参数与合法范围**：`axis`：枚举（X/Y）；`line_position_px`：int，`[0, max(width,height)-1]`；`time_window_ms`：float，`[10, 10000]`，默认 `1000`；`accumulation_ms`：float，`[1, 1000]`，默认 `100`

#### 4.3.13 🆕 立体匹配 (StereoMatcher) — 独立窗口型

极线搜索特征匹配 → 视差 → 半稠密深度图。在新窗口显示深度图。

**前置条件**：两台标定同步相机。

**参数与合法范围**：`disparity_range`：int，`[16, 256]`，默认 `64`；`block_size`：int，`[3, 21]`（奇数），默认 `7`

#### 4.3.14 🆕 可视化叠加 (Overlay)

将叠加型算法结果叠加到渲染帧上：光流箭头/颜色图、目标 bbox+ID、检测线、轨迹线、统计文字等。

#### 4.3.15 🆕 Time Surface 窗口 (TimeSurfaceWindow) — 独立窗口型

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

**一键操作**：菜单 Algorithm → Time Surface 或工具栏按钮，单击即弹出独立 Time Surface 窗口；窗口可拖拽、停靠、缩放。

---

### 4.4 algo/analytics/ — 分析模块

#### 4.4.1 主动标记跟踪 (ActiveMarker)

**原理**：检测调制光标记的特定闪烁频率（傅里叶/频率分析）→ 识别标记 ID → 2D 定位 → PnP 求解 3D 位姿。位姿估计速率可达 >1000 Hz。

| 子模块 | 方案 |
|--------|------|
| 频率检测 | 逐像素/区域 FFT 提取主频 |
| ID 识别 | 二进制闪烁编码序列匹配 |
| 2D 定位 | 标记区域质心 |
| 3D 位姿 | PnP（需要标记 3D 对应关系） |

**前置条件**：用户提供调制光源标记硬件。

#### 4.4.2 事件→灰度图像重建 (EventToVideo)

**算法**：E2VID / SSL-E2VID 神经网络推理，从事件流重建灰度帧。

| 子模块 | 方案 |
|--------|------|
| 模型加载 | ONNX Runtime / libtorch 加载 .onnx 或 .pt 模型 |
| 滑动窗口推理 | 连续事件片段 → 灰度帧输出 |
| 帧率控制 | 可配置输出帧率与累积时间 |

**参数**：`model_path`, `output_fps`(30), `accumulation_ms`(33.3)

**外部依赖**：需链接 ONNX Runtime 或 libtorch（C++ 推理库）；模型权重文件由用户自行提供，不随仓库分发。

**用途**：事件数据可视化、帧基算法输入。

---

### 4.5 algo/calibration/ — 相机标定

#### 4.5.1 内参标定 (IntrinsicCalibration)

| 子模块 | 方案 |
|--------|------|
| 标定板检测 | 棋盘格 / 圆形网格 / ArUco 标记板 |
| 多帧采集 | 从事件流自动/手动采集多姿态帧 |
| 内参计算 | Zhang 法（`cv::calibrateCamera`） |
| 畸变模型 | k1,k2,p1,p2,k3 |
| 输出 | K(3×3), distCoeffs, RMS |

#### 4.5.2 外参标定 (ExtrinsicCalibration)

| 子模块 | 方案 |
|--------|------|
| 多相机同步采集 | 基于事件时间戳对齐 |
| 外参计算 | 立体标定 / 多相机 BA |
| 输出 | R, T, E, F 矩阵 |

---

## 五、GUI 布局设计

### 5.1 主窗口布局

```
┌──────────────────────────────────────────────────────────┐
│  Menu: File | View | Camera | Preprocess | Frame Mode |  │
│        Algorithm | Calibration | Tools | Help            │
├───────────────────────────────────────┬──────────────────┤
│                                       │  Settings Panel   │
│                                       │                  │
│                                       │ ┌──────────────┐ │
│                                       │ │ Information   │ │
│         Event Display                 │ ├──────────────┤ │
│         (中央事件显示区)               │ │ Statistics    │ │
│         OpenGL 渲染                   │ ├──────────────┤ │
│         1280×720 / 640×480            │ │ Display       │ │
│                                       │ ├──────────────┤ │
│                                       │ │ Biases        │ │
│                                       │ ├──────────────┤ │
│                                       │ │ ROI           │ │
│                                       │ ├──────────────┤ │
│                                       │ │ ESP           │ │
│                                       │ ├──────────────┤ │
│                                       │ │ Trigger       │ │
│                                       │ ├──────────────┤ │
│                                       │ │ Algorithms    │ │
│                                       │ └──────────────┘ │
├───────────────────────────────────────┴──────────────────┤
│  Status Bar: 连接状态 │ 事件率 │ 时间戳 │ 录制状态        │
├──────────────────────────────────────────────────────────┤
│  Playback Controls (回放时出现): ▶ ⏸ ⏩ 进度条 ──●──       │
└──────────────────────────────────────────────────────────┘
```

### 5.2 设置面板各区域说明

| 区域 | 内容 | 适用场景 |
|------|------|----------|
| **Information** | 传感器型号、分辨率、序列号、固件版本、平台信息 | 始终可见 |
| **Statistics** | 事件率（Mev/s）、ON/OFF 比例、数据速率、温度/功耗 | 始终可见 |
| **Display** | 帧生成模式（7种）、累积时间滑块、色彩主题、叠加层开关 | 实时 / 回放 |
| **Biases** | 传感器 Bias 滑块 + 数值 + 预设 | 仅实时相机 |
| **ROI** | ROI 开关 + 坐标 + 掩码 | 仅实时相机 |
| **ESP** | Anti-Flicker / Trail Filter / ERC | 仅实时相机 |
| **Trigger** | Trigger In / Out | 仅实时相机 |
| **Preprocessing** | 事件过滤链：极性过滤、翻转、旋转、转置、缩放、速率分割 | 实时 / 回放 |
| **Algorithms** | algo/cv + algo/analytics 模块选择 + 启停 + 参数；标注显示模式（叠加/独立窗口） | 实时 / 回放 |
| **Calibration** | 内参/外参标定向导 | 实时相机 |
| **File Tools** | 文件转换（→HDF5/CSV/DAT）、裁剪、编码、信息查看 | 文件模式 |
| **Devices** | 设备列表、连接/断开、多相机同步、HAL 展示 | 始终可见 |

### 5.3 其他窗口

- **Temporal Plot 窗口**：x-t / y-t 散点图
- **算法参数对话框**：每个算法的独立参数调整面板
- **XYT 可视化窗口**：独立时空可视化
- **标定助手窗口**：内参/外参标定引导
- **转换进度对话框**：RAW→HDF5/CSV/DAT 转换进度
- **导出进度对话框**：AVI / HDF5 导出
- **HAL 控制台**：`metavision_hal_showcase` 全功能 HAL 面板

### 5.4 菜单栏

| 菜单 | 选项 |
|------|------|
| **File** | Open Camera, Open File (RAW/HDF5/DAT), Save/Load Settings, Export→AVI, Export→HDF5, File Tools→Convert to HDF5, File Tools→Convert to CSV, File Tools→Convert to DAT, File Tools→Cutter, File Tools→Info, Exit |
| **View** | Show/Hide Panels (Information/Statistics/Display/Biases/ROI/ESP/Trigger/Preprocessing/Algorithms), Fullscreen, Reset Layout |
| **Camera** | Connect/Disconnect, Device List, Platform Info, Monitor (Temperature/Power), Sync Multi-Camera, HAL Showcase |
| **Preprocess** | ROI Filter, Polarity Filter, Polarity Invert, Flip X/Y, Rotate (0°/90°/180°/270°), Transpose, Rescale, Adaptive Rate Split |
| **Frame Mode** | Integration, Diff, Histogram, Time Decay, Contrast Map, Periodic, On-Demand |
| **Algorithm** | Noise Filter, Optical Flow (Sparse/Dense), Blob Detect, Object Tracker, Corner Detect, Counter, Ultra Slow Motion, XYT View, **Time Surface**, Stereo Match, Active Marker, Event→Video |
| **Calibration** | Intrinsic Wizard, Extrinsic Wizard |
| **Tools** | Temporal Plot, Frame Composer, Data Synchronizer, Timing Profiler |
| **Help** | About, Documentation, Software Info |

### 5.5 快捷键建议

| 快捷键 | 功能 |
|--------|------|
| Ctrl+O | 打开文件 |
| Ctrl+C | 连接相机 |
| R | 开始/停止录制 |
| Space | 播放/暂停回放 |
| ← → | 回放步进 |
| S | 保存配置 |
| L | 加载配置 |
| F11 | 全屏 |
| Ctrl+Shift+T | 一键打开 Time Surface 窗口 |
| Ctrl+Shift+X | 一键打开 XYT 窗口 |
| Ctrl+W | 关闭当前活动子窗口 |

### 5.6 多窗口布局与并行显示

**核心需求**：GUI 支持多窗口可拖拽、可并行显示不同功能。新增窗口时自动重新布局，使各窗口并排可见；用户可手动拖拽调整位置与大小。

#### 5.6.1 算法显示模式分类

算法结果按显示方式分为三类，决定其窗口行为：

| 显示模式 | 说明 | 代表算法 | 行为 |
|----------|------|----------|------|
| **叠加型 (Overlay)** | 结果叠加到主显示帧上 | 光流、团块检测、目标跟踪、角点、计数、噪声过滤 | 不开新窗口，直接绘制在主显示区 |
| **主显示替换型 (Replace)** | 替换主显示区内容 | 超高速回放 | 占用主显示区 |
| **独立窗口型 (Standalone)** | 在新窗口独立显示 | Time Surface、XYT 可视化、立体匹配深度图 | 弹出独立子窗口 |

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
| 独立参数面板 | 每个独立窗口可在其侧边/底部展开自己的参数调整区 |
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
│  XYT 窗口     │  立体深度图   │
└──────────────┴──────────────┘
```

#### 5.6.5 实现技术

基于 Qt 6 的 `QDockWidget` 或 `QMdiArea`（多文档接口）实现可停靠/浮动子窗口；主窗口采用 `QMainWindow`，中央区域为主显示，子窗口作为 dock widget 可拖拽至四周或浮动。布局自动重排通过 `QMainWindow.resizeDocks` / `QMdiArea.tileSubWindows` 实现。

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

- 界面文字支持中文和英文（语言文件分离，可切换）

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

### Phase 6：基础算法 (algo/cv/ 前 6 个)
- [ ] 噪声过滤（时空邻域滤波）
- [ ] 光流估计（稀疏 + 密集，平面拟合法）
- [ ] 团块检测（连通域 + 背景建模）
- [ ] 事件级目标跟踪（DBSCAN + Kalman）
- [ ] 高速计数（检测线）
- [ ] 可视化叠加层（Overlay）

### Phase 7：高级计算机视觉 (algo/cv/ 后 5 个)
- [ ] 角点检测与跟踪（Harris + SAE）
- [ ] 超高速等效回放（时间膨胀）
- [ ] XYT 时空连续可视化
- [ ] Time Surface 窗口（独立窗口型）
- [ ] 立体匹配（半稠密深度图）

### Phase 8：分析模块 (algo/analytics/)
- [ ] 主动标记跟踪
- [ ] 事件→灰度图像重建（E2VID）

### Phase 9：标定模块 (algo/calibration/)
- [ ] 内参标定
- [ ] 外参标定（多相机）

### Phase 10：完善
- [ ] Temporal Plot 窗口
- [ ] 多窗口可拖拽并行显示布局（QDockWidget/QMdiArea）
- [ ] 窗口布局保存/恢复
- [ ] 多语言支持（中/英）
- [ ] 平台适配（Windows）
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
