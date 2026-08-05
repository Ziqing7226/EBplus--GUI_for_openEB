# 死代码审计与算法库储备分析报告

## 1. 概述

本文档记录了对项目全部死代码的系统性审计结果，包括：
- 已删除的死代码清单及删除理由
- 在线模式滞后事件丢弃失败的根因分析（基于已删除的 `time_limiter.h`）
- 保留的算法库储备文件的未来集成方案

---

## 2. 已删除的死代码（12 个文件）

### 2.1 删除清单

| # | 文件 | jAER 来源 | 删除理由 |
|---|------|-----------|----------|
| 1 | `algo/common/dvs_framer.h` | DvsFramer | SDK `CDFrameGenerator` 已提供更优的事件可视化；EventToVideo 三种算法各有独立重建 |
| 2 | `algo/common/freme.h` | Freme<O> | 仅是 `vector<O>` + width/height 的薄包装；所有算法直接管理自己的 2D 缓冲区 |
| 3 | `algo/common/kmeans.h` | — | OpenCV `cv::kmeans` 已可用且更功能丰富；`Point2D` 结构体已迁移至 `particle_filter.h` |
| 4 | `algo/common/event_rate_estimator.h` | EventRateEstimator | SDK `Metavision::RateEstimator` 已集成在 `statistics_controller.cpp`；功能已内联到 `PerformanceMeter` |
| 5 | `algo/common/lifo_event_buffer.h` | AEStack | `sparse_optical_flow.h` 自有 `lp_xs_/lp_ys_/lp_ts_` 管理事件历史；其他算法用时间戳过滤 |
| 6 | `algo/common/time_limiter.h` | TimeLimiter | 架构不兼容（需修改每个算法轮询 `should_stop()`）；当前实时控制用帧率限制 + 滞后事件丢弃（见 §3 分析） |
| 7 | `algo/common/data_loader.h/.cpp` | — | `camera_controller.cpp` 直接用 SDK `Metavision::Camera`；项目用 Qt 信号/槽，不用原始回调 |
| 8 | `algo/common/event_buffer.h/.cpp` | — | SDK `RollingEventBuffer` + Qt 信号/槽已覆盖线程间通信 |
| 9 | `algo/common/filter/median_lowpass.h` | MedianLowpassFilter | 处理标量信号去脉冲噪声；本项目处理事件流，事件噪声由 `noise_filter.h` 处理 |
| 10 | `algo/cv/overlay.h` | — | `FlowVector` 已在 `sparse_optical_flow.h` 定义；`OverlayBox/Line/Circle` 已在 `algo_backend.h` 定义；绘制由 `display_strategy.cpp` 负责 |

### 2.2 依赖修复

删除 `event_rate_estimator.h` 后，`performance_meter.h` 丢失了 `EventRateEstimator` 依赖。解决方案：将 `EventRateEstimator` 类完整内联到 `performance_meter.h` 中（保留相同的 API 和 jAER 语义）。

删除 `kmeans.h` 后，`particle_filter.h` 和 `periodic_spline.h` 丢失了 `Point2D` 依赖。解决方案：将 `Point2D` 结构体迁移到 `particle_filter.h`，`periodic_spline.h` 通过 `#include "particle_filter.h"` 获取。

### 2.3 测试清理

- `test_phase6_common.cpp`：删除了 7 个测试段（lifo_event_buffer 11 个测试、dvs_framer 9 个、freme 10 个、event_rate_estimator 6 个、time_limiter 7 个、kmeans 6 个、median_lowpass 7 个），保留 12 个段，重新编号为 1-12
- `test_phase8_10.cpp`：删除了 `#include "algo/cv/overlay.h"`、`using gui_algo::Overlay` 和 3 个 `OverlayTest` 测试
- `algo/CMakeLists.txt`：移除了 `common/event_buffer.cpp` 和 `common/data_loader.cpp`

测试总数从 341 降至 283（减少 58 个被删模块的测试），全部通过。

---

## 3. 在线模式滞后事件丢弃失败分析

### 3.1 项目约束

项目约束要求：
- "在线相机模式下播放速率锁定为 1"
- "滞后事件必须被丢弃以确保实时播放"

### 3.2 现状：完全没有实现

通过追踪完整的事件流路径，发现**项目从未实现滞后事件丢弃机制**。

#### 事件流路径（在线相机模式）

```
SDK CD Callback (camera_controller.cpp:267-292)
  │  SDK 流线程，事件实时到达
  ├─→ statistics_.add_events(b, e)          [统计]
  ├─→ filter_chain_.process(b, e, filtered) [噪声过滤]
  └─→ frame_pipeline_.add_events(...)       [显示帧生成]

Algo CD Callback (main_window.cpp:1204-1224)
  │  同一 SDK 线程
  └─→ for each inst in algo_bridge_.list_live():
        inst->push_events(b, e)              [算法处理]
```

#### AlgoInstance::push_events 的 flood guard（algo_bridge.cpp:125-158）

```cpp
void AlgoInstance::push_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    std::lock_guard<std::mutex> lk(mutex_);  // ← 阻塞点
    if (!enabled_ || overloaded_) return;
    // ...
    if (n > kMaxBatchEvents) {       // kMaxBatchEvents = 50000
        b = end - kMaxBatchEvents;    // 丢弃旧事件，保留最新
        if (++flood_strikes_ >= kFloodStrikes) {  // kFloodStrikes = 4
            overloaded_ = true;
            enabled_ = false;         // ← 自动禁用算法
            return;
        }
    }
    backend_->push_events(b, e);
}
```

### 3.3 失败根因

| 问题 | 详情 |
|------|------|
| **无时间戳比较** | 代码中没有任何位置比较事件时间戳与墙上时钟。无法判断事件是否"滞后" |
| **mutex 阻塞 SDK 线程** | `push_events` 持有 mutex 时，如果算法正在处理上一批事件，SDK 流线程被阻塞，导致相机内部缓冲区积压 |
| **flood guard 是被动防御** | 仅在批次超过 50000 事件时触发，且连续 4 次后直接禁用算法——这不是"丢弃滞后事件"，而是"算法过载后关闭算法" |
| **无队列深度控制** | 事件从 SDK 线程直接同步推送给算法，没有中间队列，无法实现 try_push/drop 语义 |
| **无在线模式感知** | `push_events` 不区分在线/文件模式，无法应用不同的丢弃策略 |

### 3.4 jAER TimeLimiter 的启示

已删除的 `time_limiter.h`（jAER TimeLimiter 移植）提供了不同的思路：

```cpp
// jAER TimeLimiter 的核心思路
TimeLimiter tl(33000);  // 33ms 预算
tl.start();
for (auto& ev : batch) {
    if (tl.should_stop()) break;  // 超预算则停止
    process(ev);
}
// 剩余事件延迟到下一帧
```

jAER TimeLimiter 的特点是：
1. **合作式**：算法主动轮询 `should_stop()`，不是外部强制中断
2. **时间预算**：每帧分配固定处理时间，超时则停止
3. **延迟而非丢弃**：未处理的事件保留到下一帧

### 3.5 本项目的正确修复方案

jAER 的 TimeLimiter 方案不适合直接移植到本项目，因为：
- 本项目的算法 `process()` 是批量处理（`push_events(b, e)`），不是逐事件循环
- 修改每个算法来轮询 `should_stop()` 需要大量改动，违反"最小化改动"原则

**建议方案：在 CD callback 层实现基于时间戳的主动丢弃**

```
在线模式事件流（修复后）：
  SDK CD Callback
    │  获取当前墙上时钟 now_us
    │  获取最新事件时间戳 ev_t = (e-1)->t
    │
    ├─ 若 (now_us - ev_t) > max_lag_us (如 100ms):
    │    └─ 丢弃整批，仅更新 statistics
    │
    └─ 否则正常处理：
       ├─→ frame_pipeline_.add_events(...)
       └─→ algo_bridge_.push_events(...)
```

关键实现点：
1. **位置**：在 `camera_controller.cpp` 的 CD callback 中（`main_window.cpp:1204` 的 algo callback 也需要同样处理）
2. **阈值**：可配置的 `max_lag_us`，默认 100ms（与帧率限制匹配）
3. **统计**：被丢弃的事件数应计入 `StatisticsPanel` 的丢弃率显示
4. **在线模式判断**：通过 `camera_.is_file_source()` 区分（在线模式才丢弃，文件模式不丢弃）

这个方案不需要修改任何算法代码，只在事件入口处做时间戳检查，是最小侵入性的修复。

---

## 4. 保留的算法库储备

以下 4 个文件虽然当前无生产消费者，但具有明确的未来集成路径，故保留。

### 4.1 performance_meter.h — 性能剖析器

**当前状态**：FPS 已在 `main_window.cpp:771` 手动计算，显示在 `StatisticsPanel`。`PerformanceMeter` 额外提供延迟、丢弃率、per-filter 开销。

**集成方案：侧栏性能面板**

1. **新增 ActivityBar 图标**：在侧栏 ActivityBar 添加 "Performance" 图标（仪表盘/lucide `gauge` 图标）
2. **新增 PerformancePanel**：继承 `AbstractPanel`，显示：
   - FPS（IIR 平滑后）
   - 端到端延迟（事件到达 → 帧显示）
   - 事件率（eps / Mev/s）
   - 丢弃率（dropped / total）
   - Per-filter 开销表（算法名 | ns/event | eps | 平均 ± 标准误差）
3. **集成点**：
   - `MainWindow` 持有 `PerformanceMeter` 实例
   - CD callback 中调用 `pm.tick_events(n, t)`
   - 帧渲染完成时调用 `pm.tick_frame()`
   - Flood guard 丢弃时调用 `pm.tick_drop(n)`
   - 每个 backend 的 `push_events` 前后调用 `pm.start(n)` / `pm.stop()`
4. **GUI 参数**：IIR 平滑系数（0.01-1.0，默认 0.1）、延迟告警阈值（ms）

**价值**：当用户报告"算法卡顿"时，性能面板可以精确定位是哪个算法的哪个环节慢。

### 4.2 particle_filter.h — 粒子滤波跟踪器

**当前状态**：`object_tracker.h` 使用 RCT + Kalman 滤波跟踪目标。

**集成方案：新增跟踪算法选项**

1. **注册算法**：在 `algo_bridge.cpp` 的 registry 中注册 `ParticleFilterTracker`
2. **新增 Backend**：`ParticleFilterBackend`（类似 `ObjectTrackerBackend`），封装 `ParticleFilter`
3. **GUI 参数**：
   - 粒子数（默认 200，范围 50-1000）
   - 扩散噪声 σ（默认 2.0，范围 0.1-10.0）
   - 似然 σ（默认 5.0，范围 1.0-20.0）
4. **显示**：复用 `display_strategy.cpp` 的 trajectory 渲染，额外用半透明点云显示粒子分布
5. **适用场景**：高速运动目标、多假设跟踪、非高斯噪声环境

**与 Kalman 的互补性**：
- Kalman 滤波：单假设、高斯噪声、计算量小——适合稳定跟踪
- 粒子滤波：多假设、任意噪声分布、计算量大——适合复杂运动

### 4.3 periodic_spline.h — 周期样条插值

**当前状态**：`object_tracker.h` 用 EMA 平滑轨迹。无闭合轨迹需求。

**集成方案：轨迹后处理模块**

1. **新增后处理选项**：在算法面板的"跟踪"类下添加 "Spline Smoothing" 开关
2. **集成点**：在 `ObjectTrackerBackend::pull_result()` 中，对轨迹点序列应用 `PeriodicSpline::resample()`
3. **GUI 参数**：
   - 重采样点数（默认 100，范围 10-500）
   - 平滑因子（默认 0.5，范围 0-2）
4. **适用场景**：
   - 旋转目标的闭合轨迹平滑
   - 高频抖动消除（比 EMA 更自然，不引入恒定延迟）
   - 轨迹美观渲染

### 4.4 filter/angular_lowpass.h — 角度低通滤波器

**当前状态**：`orientation_cluster.h` 和 `direction_selective_filter.h` 各自管理朝向平滑。

**集成方案：朝向后处理工具**

1. **作为可选后处理**：在 `OrientationClusterBackend` 和 `DirectionSelectiveFilterBackend` 中添加 "Angle Smoothing" 开关
2. **集成点**：在 `pull_result()` 中对输出的方向角度应用 `AngularLowpassFilter`
3. **GUI 参数**：
   - 截止频率（对应 α 系数，默认 0.3，范围 0.01-1.0）
4. **适用场景**：
   - 方向场可视化时消除抖动
   - 朝向聚类输出平滑
   - 光流方向场后处理

**与现有实现的区别**：
- `orientation_cluster.h` 的朝向平滑是内嵌在聚类逻辑中的，不可关闭
- `AngularLowpassFilter` 作为独立后处理，可以应用于任何角度输出，且可开关

---

## 5. 验证结果

```
编译：0 warnings
测试：283/283 passed (13.31 sec)
```

测试总数从 341 降至 283（减少 58 个被删模块的单元测试）。所有生产代码路径的测试（`RawAlgoTest`、`EventToVideoRawTest`、`SparseOpticalFlowRawTest`、`ConfigManager`、`LayoutManager`、`DisplayStrategy`、`AlgoBridge` 等）全部通过。
