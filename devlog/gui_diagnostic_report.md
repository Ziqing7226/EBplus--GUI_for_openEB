# GUI 诊断报告

> 版本：2.3（修复验证 + 误诊复核）
> 日期：2026-07-14
> 范围：`develop` 分支相对 `main` 分支（`7d5fde6`，v1.0.0）的回归审查 + GUI 全量静态代码分析
> 焦点：是否引入 BUG 或 GUI 到算法的参数传递缺失 + 通用 GUI 缺陷
> 方法：git diff 静态审查 + 关键文件逐行核对 + 341 个测试结果交叉验证 + GUI 源码全量扫描
>
> **修复状态**：BUG-R1~R9、BUG-G1~G4、BUG-G6~G12 共 20 项已修复（BUG-R10、BUG-G5 经分析确认为设计行为，非 BUG）。341/341 测试通过。
>
> **v2.2 更新**：对提交 `a67014a` 的 21 项修复进行逐项验证，发现 BUG-G2 修复无效（`AlgoInstance::get_param` 不委托 backend，num_bins 回读返回 `param_values_` 陈旧值），已在同提交 amend 中改为 `set_param("model_path")` 后从 backend 回读 num_bins 并同步到 `param_values_` 的精准方案。同时补齐 `apply_global_roi` 对 calibration 类别的过滤，与 `apply_global_preproc` 保持一致。
>
> **v2.3 更新**：对提交 `d0741d9` 的 28 项修复（21 项诊断 + 7 项系统性 N1-N8）进行逐项复核，发现：
> - **BUG-G5 误诊**：`set_enabled(true)` 不调用 `reset()` 是有意设计（支持 pause-resume 工作流），新会话 reset 由 `main_window.cpp:658-661` 相机重连时单独负责。已回退 `backend_->reset()` 调用。
> - **N2+R2 交互缺陷**：N2 的 rebuild-skip 优化仅比较 ROI 维度，但 R2 的 `downsample` 旧键转发通过此检查时会被错误跳过（`rebuild()` 使用 `aw/f, ah/f`，preproc factor 变化需要 rebuild）。已改为直接调用 `rebuild()` 并返回。
> - **BUG-G4 修复不完整**：原修复仅将信号发射移入 `if (inst)` 但未在失败时回退复选框。已补充 `else` 分支用 `QSignalBlocker` 回退状态并报错。
>
> **v2.4 更新**：对提交 `ea65262` 后的代码进行系统分析，发现并修复 2 个 N1-N8 修复的交互副作用：
> - **BUG-1 (MEDIUM): N1+N6 交互 — preproc 配置缓存覆盖全局修改**。`create()` 中 `algo_param_cache_`（含配置文件的 `preproc_*`）回放顺序在 `preproc_cache_` 之后，覆盖用户后续修改的全局 preproc 值。N6 移除了 toggled handler 中的 `apply_global_preproc` 调用，无补救路径（ROI 因 `apply_global_roi()` 补救而不受影响）。修复：交换回放顺序为 `algo_param_cache_` 先 → `preproc_cache_`/`roi_cache_` 后，使全局当前值覆盖配置旧值。
> - **BUG-2 (LOW-MEDIUM): N8 — set_all_enabled(true) 覆盖 populate_* 的 facility disable**。`populate()` 先调用 `populate_antiflicker/trail/erc`（缺失 facility 时 disable 对应 group），后调用 `set_all_enabled(true)` 启用全部 group，覆盖前者的 disable。修复：移除 `set_all_enabled(true)` 调用，各 `populate_*` 自行管理 enable 状态。

---

## 第一部分：develop vs main 回归诊断

### 1.1 审查范围

| 项 | 值 |
|----|----|
| main HEAD | `7d5fde6` (v1.0.0, 2026-07-09) |
| develop HEAD | `551fa7e` (2026-07-13) |
| 领先提交数 | 4 |
| 改动规模 | 112 文件，+8502/-4649 |
| develop 上的提交 | `e67a858` → `30cd784` → `c8d3f15` → `551fa7e` |

### 1.2 develop 上的 4 个提交主题

| 提交 | 主题 | 性质 |
|------|------|------|
| `e67a858` | release: v1.0.9 — stackable preprocessing + GUI optimization | 功能新增 + 重构 |
| `30cd784` | docs: add v1.0.9 self-check BUG report and VSCode sidebar optimization plan | 文档 |
| `c8d3f15` | fix: resolve 14 known BUGs + VSCode-style sidebar with §11 beautification | BUG 修复 + UI 重构 |
| `551fa7e` | refactor: remove legacy multi-window/toolbar, consolidate UI to sidebar | 纯 UI 精简 |

### 1.3 核心改动结构

```
main                                     develop
─────────────────────────────────────────────────────────────────
gui/algo_bridge/algo_backend.cpp (3314)  → backends/{11 个分类}.cpp
                                         + backend_common.h (共享 RoiFilter/Preprocessor)
                                         + backend_factory.cpp + backend_registry.h
gui/widgets/multi_window_manager.{h,cpp} → 删除（dock-based GUI 不需要）
gui/main_window.cpp 上帝类 (925 行)      → 拆分到 sidebar panels + ActivityBar
gui/panels/*.cpp                         → 全部继承 AbstractPanel
                                         + 新增 preproc_* 共享预处理参数系统
gui/resources/theme/                    → 新增 tokens.h + base.qss.in
gui/tests/                              → 新增 5 个测试文件
```

---

### 1.4 总体结论

**核心算法调用链路未被破坏**：`AlgoBridge → AlgoInstance → AlgoBackend` 的接口、`set_param/get_param`、`find_or_create`、`push_events/pull_result`、`apply_strategy` 全部向后兼容。最新提交 `551fa7e` 纯 UI 层重构，未触及任何算法代码。

---

## 第二部分：已验证未破坏的关键链路

### 2.1 AlgoBridge 接口完整性

| 接口 | main | develop | 状态 |
|------|------|---------|------|
| `AlgoBackend` 抽象基类 | 12 个虚函数 | 完全相同 | ✓ |
| `AlgoInstance::set_param/get_param` | map 存储 + backend 转发 | 完全相同 | ✓ |
| `AlgoBridge::find_or_create` | 注册表 + weak_ptr live | 完全相同 | ✓ |
| `AlgoBridge::apply_global_roi` | 遍历 live 实例 | 完全相同（新增 `source=="self"` 过滤） | ✓ |
| `AlgoBridge::apply_global_preproc` | 不存在 | **新增**（v1.0.9） | 加法 |
| `AlgoBridge::apply_strategy` | 内联 switch | **重构**为多态策略 | 等价 |
| `AlgoBridge::list_live` | 返回 `vector<shared_ptr>` | 完全相同 | ✓ |

### 2.2 算法注册完整性

main 注册 60 个算法 → develop 注册 59 个（移除 `noise_filter`，转为 `preproc_*` 共享预处理）。

48 个核心算法经 `backend_factory.cpp` 逐一核对全部迁移到位：
- CV 类：HotPixelFilter / OpticalGyro / PerspectiveUndistort / ObjectTracker / CornerDetector / BlobDetector / SparseOpticalFlow / HoughLine / HoughCircle / LineSegment / OrientationCluster / ClusterLif / TimeSurface / UltraSlowMotion / XYTVisualizer / Overlay
- Analytics 类：EventToVideo / FlowStatistics / ISIAnalyzer / FreqDetector / ActiveMarker / ParticleCounter / AutoBias / TriggerSynced / OrientationFilter / DirectionSelective / BackgroundMask / BandpassFilter / RoiMask / AdaptiveRateSplit
- Display 类：FrameIntegration / FrameDiff / FrameHisto / TimeDecay / ContrastMap / Periodic / OnDemand
- OpenEB 类：PreprocDiff / PreprocHisto / PreprocTimeSurface / PreprocEventCube / PreprocFactory / UtilFrameComposer / UtilRollingBuffer / UtilDataSynchronizer / UtilTimingProfiler / UtilRateEstimator / UtilVideoWriter
- Calibration 类：IntrinsicCalibration

### 2.3 关键算法参数保留情况

| 算法 | 关键参数 | 状态 |
|------|----------|------|
| E2VID | `num_bins`（由 ONNX 模型输入通道决定） | ⚠ 后端正确同步，但 GUI 未回读（见 BUG-G2） |
| E2VID | `model_path` / `decay_tau_ms` / `theta` / `auto_hdr` / `unsharp_*` / `bilateral_*` | ✓ 全部保留 |
| InteractingMaps | `im_iterations` / `relaxation_step` / V clamp [-1,1] / `I_map_` 从 Vc 热启动 | ✓ 与 main 一致 |
| BardowVariational | `lambda3` / `num_iterations` / `decay_tau_ms` | ✓ 保留 |
| InteractingMaps/BardowVariational | 共享 `window_ms` / `decay_tau_ms`（mode_filter="0,1"） | ✓ 保留 |
| InteractingMaps | `div_G` 四条边界置零 | ⚠ main/develop 均使用单边差分，未显式置零（**非回归**，属算法设计） |
| NoiseFilter 8 模式 | BAF/STCF/Refractory/DWF/AgePolarity/Harmonic/Repetitious/SpatialBP 全部 27 个参数 | ✓ 完整迁移到 `preproc_filter_*` |
| ROI 5 参数 | `roi_enabled/x/y/w/h` | ✓ 保留 |

### 2.4 三种调用路径

| 路径 | 入口 | 状态 |
|------|------|------|
| 在线相机 | `install_algo_callback` → SDK CD 回调 → `inst->push_events` | ✓ 完整 |
| 文件回放 | `on_events_window_ready` → 时间戳缩放 → `inst->push_events` | ✓ 完整 |
| 录制 | `on_record_start` → `recorder_.start` | ✓ 完整 |
| 每帧结果 | `process_algo_results` → `inst->pull_result` + `inst->apply_strategy` | ✓ 完整 |

### 2.5 测试覆盖

341/341 测试通过，包括：
- `test_algo_bridge.cpp` —— 注册表完整性、find_or_create、set/get_param
- `test_config_manager.cpp` —— save/load algo params
- `test_display_strategy.cpp` —— Passive/Overlay/Replace/Standalone 四种模式
- `test_layout_manager.cpp` / `test_theme_tokens.cpp` —— UI 层

**测试覆盖缺口**：`test_algo_bridge.cpp` 未覆盖 `apply_global_preproc` 对**新启用实例**的参数广播（BUG-R4 的盲区）。

---

## 第三部分：回归性风险点（develop vs main）

### 3.1 高严重度：旧配置迁移回归

#### BUG-R1 — `noise_filter` 算法从注册表移除【已验证 ✓】

- **位置**：[algo_bridge.cpp:565-567](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L565-L567)、[config_manager.cpp:449-453](file:///home/justin/GUI-for-openEB/gui/config/config_manager.cpp#L449-L453)
- **现象**：main 保存的算法配置 JSON 中若包含 `noise_filter` 条目，develop 加载时 `bridge->find("noise_filter")` 返回 nullptr
- **后果**：
  - `apply_algo_state` 在 [config_manager.cpp:449-453](file:///home/justin/GUI-for-openEB/gui/config/config_manager.cpp#L449-L453) 仅设 `ok=false` 但 **`err` 字符串不被填充**（已验证：L450 注释 "Unknown algorithm — skip but flag failure"，无 `err = ...` 赋值）
  - `load_algo_params_from_file` 返回 false 但用户看不到错误原因
  - 用户在 main 中精心配置的 28 个降噪参数**静默丢失**
  - 功能上由 `preproc_filter_*` 替代，但需用户手动重新配置
- **回归性**：是（main 支持，develop 不支持）
- **影响范围**：仅影响从 main 升级到 develop 并加载旧配置的用户
- **建议修复**：在 `apply_algo_state` 中对 `find(name)==nullptr` 的情况填充 `err` 提示"算法 X 已迁移/移除"，并在文档中说明 `noise_filter` → `preproc_filter_*` 的迁移映射

#### BUG-R2 — E2VID `downsample` 参数被移除【已验证 ✓】

- **位置**：[analytics_backends.cpp:88-92](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_backends.cpp#L88-L92)
- **现象**：main 的 E2VID backend 接受 `downsample` 参数；develop 中该参数被移除，改由共享的 `preproc_downsample` 处理（已验证：`rebuild()` 中 `algo_->set_e2vid_downsample(false)` / `algo_->set_downsample(false)` 硬编码关闭，交由 `preproc_` 处理）
- **后果**：
  - main 配置中 `event_to_video.downsample=false` 在 develop 中 `set_param("downsample", "false")` 被写入 `param_values_` map 但 backend 不消费（已验证：`set_param` 无 `k == "downsample"` 分支）
  - `get_param("downsample")` 仍返回旧值，但实际行为变为默认 `preproc_downsample=true`
  - **用户显式禁用下采样的意图丢失**，E2VID 会按默认开启下采样运行
- **缓解**：`preproc_downsample` 默认 "true" 与 main 的 `downsample` 默认 "true" 一致 → **默认行为保持**；仅用户显式设过 `downsample=false` 的配置会失效
- **回归性**：是
- **建议修复**：`EventToVideoBackend::set_param` 收到 `downsample` key 时自动转发到 `preproc_downsample` 并打印迁移警告

### 3.2 中严重度：preproc 链路实现不一致

#### BUG-R3 — 6 个 backend 暴露 preproc 参数但不实际应用【已验证 ✓】

- **位置**（6 个 backend 的 `push_events` 均已逐行核对）：
  - [cv_backends.cpp:51-65](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_backends.cpp#L51-L65) — HotPixelFilterBackend
  - [cv_backends.cpp:96-110](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_backends.cpp#L96-L110) — OpticalGyroBackend
  - [cv_backends.cpp:170-184](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_backends.cpp#L170-L184) — PerspectiveUndistortBackend
  - [analytics_extra_backends.cpp:140-155](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_extra_backends.cpp#L140-L155) — TriggerSyncedBackend
  - [display_backends.cpp:144-158](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/display_backends.cpp#L144-L158) — UltraSlowMotionBackend
  - [display_backends.cpp:247-258](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/display_backends.cpp#L247-L258) — OverlayBackend
- **根因**：这 6 个 backend 持有 `RoiFilter roi_`（[backend_common.h:280-336](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/backend_common.h#L280-L336) 内含 `Preprocessor preproc`），`set_param` 通过 `roi_.set_param(k, v)` 接收并存储 `preproc_*` 参数；但 `push_events` **手动 inline ROI 过滤**（直接访问 `roi_.region.enabled` 和 `roi_.region.contains`），**未调用 `roi_.apply()`**，导致 `preproc.apply()` 永不执行
- **已验证代码示例**：
  ```cpp
  // HotPixelFilterBackend::push_events（cv_backends.cpp:51-65）
  void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
      buf_.assign(b, e);
      auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
      std::size_t n = buf_.size();
      if (roi_.region.enabled && roi_.region.rw > 0 && roi_.region.rh > 0) {
          std::size_t kept = 0;
          for (std::size_t i = 0; i < n; ++i) {
              if (roi_.region.contains(ev[i].x, ev[i].y)) ev[kept++] = ev[i];
          }
          n = kept;
      }
      buf_.resize(n);
      // ❌ preproc.apply() 从未被调用，preproc_filter_enabled=true 对本算法完全无效
      algo_.learn(as_events(buf_.data()), n);
      last_kept_ = algo_.process(ev, n);
  }
  ```
- **对比**：ObjectTracker / CornerDetector / BlobDetector / SparseOpticalFlow / OrientationFilter / DirectionSelective / BackgroundMask / BandpassFilter / FreqDetector / ActiveMarker / ParticleCounter / AutoBias / LineSegment / OrientationCluster / ClusterLif / XYTVisualizer 共 16 个 backend 正确调用 `roi_.apply()` ✓（已验证 [display_backends.cpp:211](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/display_backends.cpp#L211) XYTVisualizer 使用 `roi_.apply()`）
- **后果**：用户在 GUI 启用 `preproc_filter_enabled=true` 并配置 STCF `correlation_time_s=0.02`，对这 6 个算法**完全无效**（参数被静默吞掉，事件未经降噪直接进入算法）
- **回归性**：否（main 中这些算法本就没有 preproc；这是 v1.0.9 新增功能的实现缺口）
- **影响**：用户期望"全局预处理对所有 self 算法生效"（按 [algo_bridge.cpp:558-561](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L558-L561) 注释），但实际有 6 个例外
- **建议修复**：6 个 backend 的 `push_events` 改用 `roi_.apply(src, n, buf_)` 替代 inline ROI，与其它 16 个 backend 对齐

#### BUG-R4 — 新启用实例不继承 mode-specific preproc 参数【已验证 ✓】

- **位置**：[algorithms_panel.cpp:233-238](file:///home/justin/GUI-for-openEB/gui/panels/algorithms_panel.cpp#L233-L238)
- **根因**：`onCheckboxToggle` 启用新算法时，仅向新实例补发 3 个基础 preproc 参数：
  ```cpp
  apply_global_preproc("preproc_filter_enabled",
      preproc_filter_cb_->isChecked() ? "true" : "false");
  apply_global_preproc("preproc_downsample",
      preproc_downsample_cb_->isChecked() ? "true" : "false");
  apply_global_preproc("preproc_filter_mode",
      std::to_string(preproc_filter_mode_combo_->currentIndex()));
  ```
  27 个 mode-specific 参数（`correlation_time_s` / `baf_dt_us` / `dwf_window_length` / `agep_tau_us` / `line_freq_hz` / `rep_period_us` / `sbp_center_radius_px` 等，见 [algorithms_panel.cpp:388-425](file:///home/justin/GUI-for-openEB/gui/panels/algorithms_panel.cpp#L388-L425)）**不会被补发**
- **后果**：
  - 场景：用户先全局调好 STCF `correlation_time_s=0.02`，已启用算法 A 收到 0.02；之后启用算法 B → B 用 `info_.params` 默认值 0.005
  - 已启用算法 A 和新启用算法 B **行为不一致**
- **`apply_global_preproc` 实现**（[algo_bridge.cpp:312-329](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L312-L329)）：仅广播单个 (key, value) 对，不遍历所有存储的 preproc 参数
- **回归性**：否（v1.0.9 新功能）
- **测试覆盖**：`test_algo_bridge.cpp` 无相关测试
- **建议修复**：`AlgoBridge` 维护一份全局 preproc 参数缓存（`unordered_map<string, string>`），`apply_global_preproc(key, val)` 写入缓存并广播；`find_or_create` 创建新实例时从缓存批量回放

### 3.3 低严重度：代码质量问题

#### BUG-R5 — `annotated_frame_ready` 信号变死信号【已验证 ✓】

- **位置**：[main_window.cpp:738](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L738)（发射）、[main_window.h:84](file:///home/justin/GUI-for-openEB/gui/main_window.h#L84)（声明）
- **引入提交**：`551fa7e`
- **现象**：删除 `MultiWindowManager` 后，`emit annotated_frame_ready(frame)` 已无任何接收者（已通过全项目 grep 验证：仅 2 处命中——声明 + 发射，无 `connect`），每帧仍发射一次
- **影响**：Qt 优雅处理（无崩溃），仅死代码 + 微量性能浪费

#### BUG-R6 — 键盘快捷键丢失【已验证 ✓】

- **位置**：[roi_panel.cpp:74](file:///home/justin/GUI-for-openEB/gui/panels/roi_panel.cpp#L74)、[file_tools_panel.cpp:41](file:///home/justin/GUI-for-openEB/gui/panels/file_tools_panel.cpp#L41)
- **引入提交**：`551fa7e`
- **现象**：原 `a_roi_drag_->setShortcut("Ctrl+R")` 和 `a_record_start_->setShortcut("R")` 在迁移到 sidebar 按钮后未重新绑定（已通过全项目 grep 验证：仅 `pb_toggle->setShortcut(QKeySequence("Ctrl+Shift+P"))` 1 处命中）
- **影响**：用户 `R` / `Ctrl+R` 快捷键失效

#### BUG-R7 — IconProvider 缓存永不清理【已验证 ✓】

- **位置**：[icon_provider.cpp:13-21](file:///home/justin/GUI-for-openEB/gui/app/icon_provider.cpp#L13-L21)
- **引入提交**：`551fa7e`
- **现象**：`icon_cache()` 是函数内静态局部 `QHash`，按 `(name, color)` 累积，主题切换时旧颜色的图标不被清除
- **影响**：理论无界增长，实际 < 100 条目，影响极小；另外该缓存为 static 局部变量，跨线程访问未加锁（当前仅在 GUI 线程调用，安全；未来若有后台线程调用 `IconProvider::get` 会触发 data race）

#### BUG-R8 — Calibration 算法 param_values_ map 污染【已验证 ✓】

- **位置**：[algo_bridge.cpp:818-824](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L818-L824)
- **引入提交**：早期 develop（`e67a858`）
- **现象**：`register_self_calibration` 的 `add` lambda 未调用 `roi_params()` 和 `preproc_params()`（已验证：[algo_bridge.cpp:826-832](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L826-L832) 仅注册 4 个 calibration 参数，无 roi/preproc 注入），但 `apply_global_roi` / `apply_global_preproc` 因 `source=="self"` 仍会向 `intrinsic_calibration` 实例调用 `set_param("roi_*")` / `set_param("preproc_*")`，这些 key 不在算法的 `AlgoParamSpec` 列表中，`param_values_` map 混入未定义 key
- **影响**：ConfigManager 序列化时不会写出（仅遍历 `info.params`），但反序列化时若 JSON 含这些 key 会被 `set_param` 接受；非回归，但配置可能污染

#### BUG-R9 — 4 个 panel 重复定义 `restyle()` 函数

- **位置**：[roi_panel.cpp:21](file:///home/justin/GUI-for-openEB/gui/panels/roi_panel.cpp#L21)、[esp_panel.cpp:39](file:///home/justin/GUI-for-openEB/gui/panels/esp_panel.cpp#L39)、[biases_panel.cpp:24](file:///home/justin/GUI-for-openEB/gui/panels/biases_panel.cpp#L24)、[trigger_panel.cpp:33](file:///home/justin/GUI-for-openEB/gui/panels/trigger_panel.cpp#L33)
- **引入提交**：早期 develop（`c8d3f15`）
- **现象**：4 处匿名命名空间各自定义相同的 `restyle(QWidget*)` 函数
- **影响**：代码重复，建议提取到 `AbstractPanel` 或工具函数

#### BUG-R10 — panel 刷新模式不一致

- **位置**：[main_window.cpp:1065-1068](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L1065-L1068)、[main_window.cpp:1137-1140](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L1137-L1140)
- **引入提交**：早期 develop（`c8d3f15`）
- **现象**：`on_apply_preset` 和 `on_load_config` 仍直接调用 `settings_->biases_panel()->on_camera_connected(&camera_)` 等 4 个 panel 的槽函数，而非依赖 `bind_camera` 的自动广播机制（[main_window.cpp:596-598](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L596-L598)）
- **影响**：功能正确，但与 `bind_camera` 模式不一致，维护时易混淆

---

## 第四部分：GUI 静态代码分析（全量扫描）

> 以下为对 `gui/` 目录下源码的全量静态审查发现，不限于 develop vs main 回归。

### 4.1 高严重度

#### BUG-G1 — `decay_tau_ms` 默认值违反规格约束【新发现】

- **位置**：[algo_bridge.cpp:761](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L761)
  ```cpp
  pfloat("decay_tau_ms", "Decay tau (ms)", "0", "0", "5000", "0,1"),
  ```
- **现象**：默认值为 `"0"`，但项目硬约束要求 `decay_tau_ms` 默认值为 **500ms**（范围 [0, 5000]）
- **后果**：InteractingMaps / BardowVariational 模式在用户未手动调整时，decay 完全关闭（等效于无时间衰减），重建图像会呈现瞬时响应而非平滑过渡
- **回归性**：否（develop 与 main 均为默认 0，属长期遗留问题），但违反现行规格
- **建议修复**：将默认值从 `"0"` 改为 `"500"`

#### BUG-G2 — E2VID `num_bins` 模型加载后 GUI 未回读【新发现】

- **位置**：[analytics_backends.cpp:147-155](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_backends.cpp#L147-L155)（后端同步）、[algorithms_panel.cpp:500-518](file:///home/justin/GUI-for-openEB/gui/panels/algorithms_panel.cpp#L500-L518)（GUI 单向写）
- **现象**：后端加载 ONNX 模型后，`e2vid_num_bins_` 被模型实际通道数覆盖：
  ```cpp
  } else if (k == "model_path") {
      model_path_ = v;
      if (algo_) {
          algo_->set_model_path(model_path_);
          e2vid_num_bins_ = algo_->e2vid_num_bins();  // 后端值已更新
      }
  }
  ```
  但 GUI 的 `apply_param` 只调 `set_param` 单向写，未回读 num_bins spinbox。GUI 仍显示用户输入值（或默认 5），而算法实际用模型通道数（如 10）。
- **后果**：GUI 显示与算法实际行为不一致，违反约束 *"E2VID num_bins is determined by the ONNX model's input channel count"*
- **建议修复**：`apply_param` 处理 `model_path` 后，通过信号或回调通知 GUI 刷新 num_bins spinbox
- **修复验证（v2.2）**：初次修复（a67014a）在 `algorithms_panel.cpp` 加了 `get_param("num_bins")` 回读，但 **`AlgoInstance::get_param` 只读 `param_values_` 不委托 backend**，导致回读返回陈旧值（注册表默认 "5"），修复无效。最终方案改为在 `AlgoInstance::set_param` 中，当 `key=="model_path"` 时，调用 `backend_->set_param` 后从 `backend_->get_param("num_bins")` 回读并写入 `param_values_["num_bins"]`，使后续 GUI 的 `get_param` 返回模型实际通道数。不采用"get_param 全面委托 backend"是因为 backend 用 `std::to_string(double)` 格式化（"5.500000"），与注册表默认字符串（"5.5"）不一致，会破坏 `ParamRoundTrip` 等测试。

#### BUG-G3 — 断开相机时未停止录制【新发现】

- **位置**：[main_window.cpp:1033-1035](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L1033-L1035)
  ```cpp
  void MainWindow::on_disconnect() {
      camera_.disconnect();
  }
  ```
- **现象**：`closeEvent`（L194-237）正确地先 `recorder_.stop()` 再 `camera_.disconnect()`，但 `on_disconnect()` 槽**未**停止录制就直接断开相机
- **后果**：若用户在录制中点 Disconnect，RecorderController 可能写入损坏文件（未 flush / 未写 trailer）或访问已释放的相机句柄
- **建议修复**：`on_disconnect()` 中在 `camera_.disconnect()` 前调用 `recorder_.stop()`

#### BUG-G4 — `find_or_create` 失败时仍发射窗口打开信号【新发现】

- **位置**：[algorithms_panel.cpp:223-239](file:///home/justin/GUI-for-openEB/gui/panels/algorithms_panel.cpp#L223-L239)
  ```cpp
  auto inst = bridge_->find_or_create(algo_name);
  if (inst) {
      inst->set_enabled(true);
      live_instances_[algo_name] = inst;
      apply_global_roi();
      ...
  }
  emit open_algo_window_requested(algo_name);  // 即使 inst 为 null 也发射
  ```
- **现象**：若 `find_or_create` 返回 null（注册表找不到该算法），`open_algo_window_requested` 信号仍发射，`MainWindow::on_open_algo_window` 会创建 AlgoWindow。同时 sidebar 复选框已被勾选却无实例，UX 状态不一致
- **建议修复**：`if (inst)` 内才发射 `open_algo_window_requested`；失败时回退复选框状态

### 4.2 中严重度

#### BUG-G5 — 重新启用算法未调用 `reset()` 清理陈旧状态【非BUG，设计行为】

- **位置**：[algo_bridge.cpp:90-106](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L90-L106)
- **原始诊断**：`set_enabled(true)` 仅清理过载状态，不调用 `backend_->reset()`，认为会导致跨会话残影和错误 decay 计算
- **复核结论**：**误诊**。原始行为是有意设计，用于支持 pause-resume 工作流：
  1. 用户临时禁用算法后重新启用，期望保留已累积的状态（如 E2VID `log_intensity_`、InteractingMaps `I_map_`）以继续重建
  2. 新会话的完整 reset 由调用方负责：[main_window.cpp:658-661](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L658-L661) 在相机重连时对每个 live 实例单独调用 `inst->reset()`
  3. 原始代码注释明确说明"only clears overload state"，与 `reset()` 的职责分离一致
- **修复回退**：提交 `d0741d9` 中新增的 `backend_->reset()` 调用已回退，恢复原始设计行为
- **状态**：非 BUG，无需修复

#### BUG-G6 — `EspPanel::populate_antiflicker` 在连接期间弹出模态对话框【新发现】

- **位置**：[esp_panel.cpp:318-319](file:///home/justin/GUI-for-openEB/gui/panels/esp_panel.cpp#L318-L319)、[esp_panel.cpp:367-368](file:///home/justin/GUI-for-openEB/gui/panels/esp_panel.cpp#L367-L368)、[esp_panel.cpp:409-410](file:///home/justin/GUI-for-openEB/gui/panels/esp_panel.cpp#L409-L410)
- **现象**：`populate()` 在 `on_camera_connected` 中调用，期间 `error_message` 信号通过 `forward_panel_message` 触发 `QMessageBox::warning`（模态）。相机连接时若多个 facility 都报错，会连续弹出多个模态对话框，阻塞 UI 流程
- **建议修复**：将错误收集到列表，连接完成后统一以非模态 `QStatusBar` 或日志面板展示

#### BUG-G7 — `EspPanel::populate_antiflicker` 未恢复 `af_preset_` 选中项【新发现】

- **位置**：[esp_panel.cpp:281-342](file:///home/justin/GUI-for-openEB/gui/panels/esp_panel.cpp#L281-L342)
- **现象**：从相机读取 `low_f`/`high_f` 后只更新 spinbox，不根据频率范围反查匹配的 preset（50Hz mains: 90-110，60Hz mains: 110-130）。若相机当前是 90-110Hz，preset 仍显示 "Custom"
- **影响**：UX 不一致，用户无法通过 preset 快速识别当前频段
- **建议修复**：`populate_antiflicker` 末尾根据 `low_f`/`high_f` 匹配 preset 并设置 `af_preset_` 当前项

#### BUG-G8 — `FileToolsPanel::on_convert_*` 未检查 `converter_` 是否为 null【新发现】

- **位置**：[file_tools_panel.cpp:109,124,159,168](file:///home/justin/GUI-for-openEB/gui/panels/file_tools_panel.cpp#L109)
  ```cpp
  converter_->convert(src, dst, FileConverter::Format::HDF5);
  ```
- **现象**：构造函数虽总传入非 null 的 `&file_converter_`，但 API 不健壮。`on_info` 用了 try/catch 但 null deref 是 UB 不是异常，catch 捕获不到
- **影响**：当前运行时无问题，但若未来重构改变 `converter_` 生命周期则崩溃
- **建议修复**：添加 `if (!converter_) return;` 守卫

### 4.3 低严重度

#### BUG-G9 — `EventToVideoBackend::rebuild()` 注释与实现不符【新发现】

- **位置**：[analytics_backends.cpp:82-85](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_backends.cpp#L82-L85)
- **现象**：注释说 "model reload is intentionally deferred to set_model_path"，但下一行 `if (!model_path_.empty()) algo_->set_model_path(model_path_);` 实际上在 `rebuild()` 中加载模型
- **后果**：每次 ROI 改变都触发 ONNX 模型重新加载，可能引入性能卡顿（ONNX 加载耗时）
- **建议修复**：修正注释，或缓存已加载的模型避免重复加载

#### BUG-G10 — `AlgoWindow::set_display_widget` 删除旧 widget 后才赋值新值【新发现】

- **位置**：[algo_window.cpp:71-83](file:///home/justin/GUI-for-openEB/gui/widgets/algo_window.cpp#L71-L83)
  ```cpp
  if (display_widget_) {
      display_layout_->removeWidget(display_widget_);
      delete display_widget_;
  }
  display_widget_ = w;
  ```
- **现象**：`delete` 后 `display_widget_` 短暂悬空（2 行之间），若 Qt 在 ~QObject() 期间通过事件循环派发访问 `display_widget_` 的信号，理论上触发 UAF。实际几乎不可达（`removeWidget` 已切断 layout 关联）
- **建议修复**：先 `display_widget_ = nullptr` 再 delete

#### BUG-G11 — `tabifyDockWidget` 隐式假设 other 与 w 同 dock 区域【新发现】

- **位置**：[main_window.cpp:1607-1613](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L1607-L1613)
  ```cpp
  for (auto tit = algo_windows_.constBegin(); ...) {
      AlgoWindow* other = tit.value().data();
      if (other && other != w && !other->isFloating()) {
          tabifyDockWidget(other, w);
          break;
      }
  }
  ```
- **现象**：若用户手动把某个 AlgoWindow 拖到左侧 dock 区域，后续新 AlgoWindow 仍加入右侧，`tabifyDockWidget` 会把新窗口移到 other 所在的左侧，与设计（算法窗口应在右侧）不符
- **建议修复**：遍历时检查 `dockWidgetArea(other) == RightDockWidgetArea`

#### BUG-G12 — `AlgoInstance::set_param` 将未知键插入 `param_values_`【新发现】

- **位置**：[algo_bridge.cpp:52-58](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp#L52-L58)
  ```cpp
  void AlgoInstance::set_param(const std::string& key, const std::string& value) {
      std::lock_guard<std::mutex> lk(mutex_);
      param_values_[key] = value;  // operator[] 会插入未知键
      if (backend_) backend_->set_param(key, value);
  }
  ```
- **现象**：`operator[]` 会插入未知键。GUI 只发已知键，但 ConfigManager 加载 JSON 时若文件含未知 key，会污染 param map 并被持久化
- **建议修复**：仅当 key 在 `info_.params` 列表中时才写入 `param_values_`

---

## 第五部分：诊断交叉验证与裁定

### 5.1 旧文档诊断裁定

本文档前身为 `develop_vs_main_diagnostic.md`（v1.0），其中所有诊断已通过代码逐行验证，裁定结果如下：

| 原编号 | 新编号 | 裁定 | 说明 |
|--------|--------|------|------|
| H1 | BUG-R1 | ✓ 确认 | `config_manager.cpp:449-453` 已验证无 `err` 赋值 |
| H2 | BUG-R2 | ✓ 确认 | `analytics_backends.cpp` `set_param` 无 `downsample` 分支 |
| M1 | BUG-R3 | ✓ 确认 | 6 个 backend 的 `push_events` 均已逐行核对，使用 inline ROI 而非 `roi_.apply()` |
| M2 | BUG-R4 | ✓ 确认 | `algorithms_panel.cpp:233-238` 仅补发 3 个基础参数 |
| L1 | BUG-R5 | ✓ 确认 | 全项目 grep 验证：`annotated_frame_ready` 仅声明+发射，无 connect |
| L2 | BUG-R6 | ✓ 确认 | 全项目 grep 验证：仅 `Ctrl+Shift+P` 1 处 setShortcut |
| L3 | BUG-R7 | ✓ 确认 | `icon_provider.cpp` static 局部 QHash 无清理逻辑 |
| L4 | BUG-R8 | ✓ 确认 | `register_self_calibration` 未注入 roi/preproc 参数 |
| L5 | BUG-R9 | ✓ 确认 | 4 处匿名命名空间重复 restyle |
| L6 | BUG-R10 | ✓ 确认 | `on_apply_preset` / `on_load_config` 直接调用 panel 槽 |

**旧文档所有 10 项诊断全部成立。**

### 5.2 旧文档遗漏的问题

旧文档在 §3.3 提到 "decay_tau_ms 默认 0，非 500ms"，但仅作为备注，未标记为 BUG。经核对项目硬约束（*"default 500ms"*），此处应升级为 **BUG-G1**（高严重度，违反规格约束）。

### 5.3 新增诊断与旧诊断的关系

| 新增诊断 | 与旧诊断关系 | 说明 |
|----------|-------------|------|
| BUG-G1 (decay_tau_ms 默认值) | **新发现** | 旧文档仅备注未标记为 BUG |
| BUG-G2 (num_bins GUI 未回读) | **新发现** | 旧文档 §3.3 标注 ✓ 但实际 GUI 侧存在同步缺口 |
| BUG-G3 (断开未停录) | **新发现** | 旧文档未覆盖 `on_disconnect` 路径 |
| BUG-G4 (find_or_create null 仍发信号) | **新发现** | 旧文档未覆盖 |
| BUG-G5 (重新启用未 reset) | **新发现（误诊）** | 旧文档未覆盖 `set_enabled` 路径；经复核确认为设计行为 |
| BUG-G6 (ESP 模态对话框) | **新发现** | 旧文档未覆盖 panel 层 |
| BUG-G7 (af_preset 未恢复) | **新发现** | 旧文档未覆盖 panel 层 |
| BUG-G8 (converter_ null) | **新发现** | 旧文档未覆盖 |
| BUG-G9 (rebuild 注释错误) | **新发现** | 旧文档未覆盖 |
| BUG-G10 (set_display_widget 顺序) | **新发现** | 旧文档未覆盖 |
| BUG-G11 (tabifyDockWidget 区域) | **新发现** | 旧文档未覆盖 |
| BUG-G12 (set_param 未知键) | 与 BUG-R8 相关 | R8 是 calibration 特例，G12 是通用机制问题 |

### 5.4 提交级别归因（修订版）

| 风险点 | 引入提交 | 是否回归 | 是否影响算法参数传递 |
|--------|----------|----------|----------------------|
| BUG-R1 noise_filter 注册移除 | `e67a858` | 是（配置迁移） | 否（仅影响旧配置加载） |
| BUG-R2 E2VID downsample 移除 | `e67a858` | 是（配置迁移） | 否（仅影响旧配置加载） |
| **BUG-R3 6 backend 不应用 preproc** | `e67a858` | 否（新功能缺口） | **是**（GUI 设置无效） |
| **BUG-R4 新实例不继承 preproc 参数** | `e67a858` | 否（新功能缺口） | **是**（参数丢失） |
| BUG-R5 死信号 | `551fa7e` | 否 | 否 |
| BUG-R6 快捷键丢失 | `551fa7e` | 否 | 否 |
| BUG-R7 icon 缓存 | `551fa7e` | 否 | 否 |
| BUG-R8 calibration param 污染 | `e67a858` | 否 | 否（map 污染但不影响行为） |
| BUG-R9 重复 restyle | `c8d3f15` | 否 | 否 |
| BUG-R10 panel 刷新不一致 | `c8d3f15` | 否 | 否 |
| **BUG-G1 decay_tau_ms 默认值违规** | 长期遗留 | 否 | **是**（影响重建质量） |
| **BUG-G2 num_bins GUI 未回读** | `e67a858` | 否 | **是**（GUI 显示不一致） |
| BUG-G3 断开未停录 | `551fa7e` | 否 | 否（影响录制完整性） |
| BUG-G4 null 仍发信号 | `c8d3f15` | 否 | 否 |
| BUG-G5 重新启用未 reset | 长期遗留 | 否 | 否（设计行为，非BUG） |
| BUG-G6 ESP 模态对话框 | `c8d3f15` | 否 | 否 |
| BUG-G7 af_preset 未恢复 | `c8d3f15` | 否 | 否 |
| BUG-G8 converter_ null | `c8d3f15` | 否 | 否 |
| BUG-G9 rebuild 注释错误 | `e67a858` | 否 | 否 |
| BUG-G10 set_display_widget 顺序 | 长期遗留 | 否 | 否 |
| BUG-G11 tabifyDockWidget 区域 | `c8d3f15` | 否 | 否 |
| BUG-G12 set_param 未知键 | 长期遗留 | 否 | 否 |

---

## 第六部分：建议修复优先级与修复状态

| 优先级 | 编号 | 修复方案 | 工作量 | 状态 |
|--------|------|----------|--------|------|
| **P0（必修）** | BUG-R3 | 6 个 backend 的 `push_events` 改用 `roi_.apply()` 替代 inline ROI | 小（每个 backend 改 5-8 行） | ✅ 已修复 |
| **P0（必修）** | BUG-R4 | `AlgoBridge` 维护全局 preproc 缓存，`find_or_create` 时批量回放 | 中（需新增成员 + 修改 `apply_global_preproc`） | ✅ 已修复 |
| **P0（必修）** | BUG-G1 | `decay_tau_ms` 默认值从 `"0"` 改为 `"500"` | 极小 | ✅ 已修复 |
| **P0（必修）** | BUG-G3 | `on_disconnect()` 中先 `recorder_.stop()` 再 `camera_.disconnect()` | 极小 | ✅ 已修复 |
| **P1（建议）** | BUG-R1 | `apply_algo_state` 对未知算法名填充 `err` 提示 | 小 | ✅ 已修复 |
| **P1（建议）** | BUG-R2 | `EventToVideoBackend::set_param` 收到 `downsample` 时转发到 `preproc_downsample` | 小 | ✅ 已修复 |
| **P1（建议）** | BUG-G2 | `apply_param` 处理 `model_path` 后通知 GUI 刷新 num_bins | 小 | ✅ 已修复 |
| **P1（建议）** | BUG-G4 | `find_or_create` 返回 null 时不发信号 | 极小 | ✅ 已修复 |
| **P2（可选）** | BUG-G5 | ~~`set_enabled(true)` 时调用 `backend_->reset()` 清除旧状态~~ | 小 | ❌ 非BUG，设计行为（已回退） |
| **P2（可选）** | BUG-R5 | 删除 `annotated_frame_ready` 信号发射及声明 | 极小 | ✅ 已修复 |
| **P2（可选）** | BUG-R6 | 给 `drag_check_` / `btn_record_` 补 `setShortcut` | 极小 | ✅ 已修复 |
| **P2（可选）** | BUG-G6 | ESP populate 错误改为 `info_message`（非模态状态栏） | 小 | ✅ 已修复 |
| **P2（可选）** | BUG-G7 | `populate_antiflicker` 末尾根据 low/high_f 匹配 preset | 小 | ✅ 已修复 |
| **P3（可选）** | BUG-R7 | 主题切换时调用 `IconProvider::clear_cache()` | 极小 | ✅ 已修复 |
| **P3（可选）** | BUG-R8 | `apply_global_preproc` 过滤 `category=="calibration"` 的算法 | 小 | ✅ 已修复 |
| **P3（可选）** | BUG-G8 | `FileToolsPanel` 4 个方法添加 `converter_` null 守卫 | 极小 | ✅ 已修复 |
| **P3（可选）** | BUG-G9 | 修正 `rebuild()` 注释（模型实际在此加载） | 极小 | ✅ 已修复 |
| **P3（可选）** | BUG-G10 | `set_display_widget` 先置 null 再 delete | 极小 | ✅ 已修复 |
| **P3（可选）** | BUG-G11 | `tabifyDockWidget` 前检查 `dockWidgetArea` 一致 | 极小 | ✅ 已修复 |
| **P3（可选）** | BUG-G12 | `set_param` 仅在 `info_.params` 列表中时写入 `param_values_` | 小 | ✅ 已修复 |
| **P3（可选）** | BUG-R9 | 提取 `restyle()` 到 `AbstractPanel`（4 文件去重） | 极小 | ✅ 已修复 |
| **P3（可选）** | BUG-R10 | `on_apply_preset` / `on_load_config` 直接调用 panel 槽 | 小 | ⏭️ 非BUG（设计行为） |

---

## 第七部分：审查方法说明

### 7.1 静态差异分析

```bash
# 总体规模
git diff --shortstat main..develop
git diff --stat main..develop | tail -3

# 算法相关函数调用变化
git diff main..develop -- gui/main_window.cpp | \
    grep -E "^[+-].*(set_param|get_param|apply_param|find_or_create|push_events|algo_bridge_|apply_global)"

# 算法注册函数对比
git show main:gui/algo_bridge/algo_backend.cpp | grep -E "^void.*register|^static.*register"
grep -rE "^void.*register" gui/algo_bridge/backends/

# 最新提交影响范围
git show --stat 551fa7e
```

### 7.2 关键文件逐行核对

- `gui/algo_bridge/algo_bridge.{h,cpp}` —— 接口签名、live 实例管理、参数注册
- `gui/algo_bridge/backends/backend_common.h` —— `RoiFilter` / `Preprocessor` 共享逻辑
- `gui/algo_bridge/backends/{cv,analytics,display}_backends.cpp` —— 各 backend 的 `push_events` 是否调用 `roi_.apply()`
- `gui/algo_bridge/backends/analytics_extra_backends.cpp` —— TriggerSyncedBackend
- `gui/panels/algorithms_panel.cpp` —— 参数控件与 `apply_global_preproc` 的连接
- `gui/panels/esp_panel.cpp` —— ESP 面板 populate 逻辑
- `gui/panels/file_tools_panel.cpp` —— 文件转换
- `gui/main_window.cpp` —— 算法调用三路径 + 生命周期
- `gui/config/config_manager.cpp` —— 配置加载/保存
- `gui/widgets/algo_window.cpp` —— AlgoWindow 生命周期
- `gui/widgets/activity_bar.cpp` —— ActivityBar 拖动
- `gui/app/icon_provider.cpp` —— 图标缓存

### 7.3 交叉验证

- 341 个 GTest 测试全部通过（`ctest --output-on-failure`）
- 测试覆盖盲区：`apply_global_preproc` 对新启用实例的参数广播（BUG-R4 的盲区）
- 项目 memory 中的硬约束全部核对（E2VID num_bins / InteractingMaps V clamp / decay_tau 范围 / 全英文界面 / 等）
- **发现 1 项硬约束违规**：`decay_tau_ms` 默认值应为 500ms，实际为 0（BUG-G1）

---

## 第八部分：附录——算法参数传递链路图

```
┌─────────────────────────────────────────────────────────────────┐
│ GUI Layer                                                       │
│                                                                 │
│  AlgorithmsPanel                                                │
│    ├─ roi_* Spinboxes        ──┐                                │
│    ├─ preproc_* Controls     ──┤                                │
│    └─ 算法参数 Controls       ──┤                                │
│                                │                                │
│  ┌─────────────────────────────▼──────────────────────────┐    │
│  │ apply_global_roi / apply_global_preproc / apply_param  │    │
│  └─────────────────────────────┬──────────────────────────┘    │
└────────────────────────────────┼────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│ AlgoBridge                                                      │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ find_or_create(name) → AlgoInstance                       │  │
│  │ apply_global_roi(key,val)  → 遍历 live (source=="self")   │  │
│  │ apply_global_preproc(key,val) → 遍历 live (source=="self")│  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────┬────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│ AlgoInstance                                                    │
│  set_param(k,v) → param_values_[k]=v + backend_->set_param(k,v)│
│  push_events(b,e) → backend_->push_events(b,e)                 │
│  pull_result() → backend_->pull_result()                       │
└────────────────────────────────┬────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│ AlgoBackend (具体实现)                                          │
│                                                                 │
│  ✅ 正确实现 (16 个 backend):                                   │
│    ObjectTracker / CornerDetector / BlobDetector / ...          │
│    push_events: roi_.apply(src, n, buf) → ROI + preproc 都执行 │
│                                                                 │
│  ❌ BUG-R3 缺陷 (6 个 backend):                                 │
│    HotPixelFilter / OpticalGyro / PerspectiveUndistort /        │
│    TriggerSynced / UltraSlowMotion / Overlay                   │
│    push_events: inline roi_.region.contains → 仅 ROI，无 preproc│
│                                                                 │
│  ⚠ BUG-G2: EventToVideoBackend 加载模型后 num_bins 未回读 GUI  │
│  ⚠ BUG-G1: decay_tau_ms 默认 0 (应 500)                        │
└─────────────────────────────────────────────────────────────────┘
```

---

## 第九部分：算法实现参数完整性审查

> 焦点：AlgoParamSpec（GUI 暴露的参数）与 AlgoBackend 的 `set_param`/`get_param`
> 及算法类的实际 API 之间的三方一致性。违反约束 *"All algorithm parameters
> must have both set_param and get_param implementations to ensure GUI can
> read/write values"*。

### 9.1 审查方法

对全部 self-developed 算法的 AlgoParamSpec → Backend set_param → Backend
get_param → 算法类 getter/setter 四层逐一交叉核对。OpenEB 算法因无自定义
参数无需审查。

### 9.2 发现的问题

#### ALG-1 — OrientationCluster 缺少 11 个真实参数【已修复 ✅】

- **现象**：`algo_bridge.cpp` 中 `orientation_cluster` 的 AlgoParamSpec 仅
  暴露 `min_events`（文档注释确认这是 no-op 兼容垫片， *"the value is stored
  but has no effect on filtering"*）。但后端 `OrientationClusterBackend::set_param`
  实际处理 12 个参数（dt, factor, rf_width, rf_height, tolerance, ori,
  neighbor_thr, thr_gradient, history_factor, use_opposite_polarity,
  ori_history_enabled, display_length），而 `get_param` 仅返回 `min_events`。
- **根因**：AlgoParamSpec 与后端 set_param/get_param 不同步——spec 只有 1 个
  参数，set_param 有 13 个，get_param 有 1 个。算法类 `OrientationCluster`
  已实现全部 12 个 getter（[orientation_cluster.h:249-283](file:///home/justin/GUI-for-openEB/algo/cv/orientation_cluster.h#L249-L283)）。
- **影响**：用户无法在 GUI 调节 OrientationCluster 的任何真实过滤参数；
  GUI 写入这些参数（若通过 JSON 配置加载）后无法回读。
- **修复**：在 spec 中添加 11 个参数；在 get_param 中添加 11 个对应 handler。

#### ALG-2 — ClusterLIF 缺少 3 个参数【已修复 ✅】

- **现象**：`cluster_lif` 的 AlgoParamSpec 仅暴露 `tau_ms` 和 `threshold`，
  但后端 `ClusterLifBackend::set_param` 实际处理 5 个参数（额外含
  receptive_field_size_pixels, initial_potential_percent,
  jump_after_firing_percent），`get_param` 仅返回前 2 个。
- **根因**：同 ALG-1，spec 与后端不同步。算法类 `ClusterLIF` 已实现全部
  getter（[cluster_lif.h:217-231](file:///home/justin/GUI-for-openEB/algo/cv/cluster_lif.h#L217-L231)）。
- **影响**：用户无法在 GUI 调节 LIF 神经元网格的关键参数（感受野大小、
  初始膜电位、发放后跳变量），影响 jAER BlurringTunnelFilter 的忠实复现。
- **修复**：在 spec 中添加 3 个参数；在 get_param 中添加 3 个 handler。

#### ALG-3 — TriggerSynced 缺少 t0_us 和 trigger_channel【已修复 ✅】

- **现象**：`trigger_synced` 的 AlgoParamSpec 仅暴露 `window_us`，但后端
  `TriggerSyncedBackend::set_param` 实际处理 4 个参数（额外含 t0_us, t1_us,
  trigger_channel），`get_param` 仅返回 `window_us`。
- **根因**：spec 与后端不同步。注意 `t1_us` 与 `window_us` 冗余（两者都映射
  到 `t1_`，因 `trigger_window_us()` 返回 `t1_`），故仅添加 `t0_us` 和
  `trigger_channel`。算法类已实现 `t0()` 和 `trigger_channel()` getter
  （[trigger_synced_filter.h:121-136](file:///home/justin/GUI-for-openEB/algo/cv/trigger_synced_filter.h#L121-L136)）。
- **影响**：用户无法在 GUI 调节激光延迟（t0）和触发通道选择。
- **修复**：在 spec 中添加 t0_us 和 trigger_channel；在 get_param 中添加
  对应 handler。

#### ALG-4 — BackgroundMask 缺少 erosion_size 参数【已修复 ✅】

- **现象**：`background_mask` 的 AlgoParamSpec 仅暴露 `learning_rate` 和
  `threshold`，但后端 `BackgroundMaskBackend::set_param` 实际处理 3 个参数
  （额外含 erosion_size），`get_param` 未返回 erosion_size。
- **根因**：spec 缺少 erosion_size；算法类已实现 `erosion_size()` getter
  （[background_mask_filter.h:121](file:///home/justin/GUI-for-openEB/algo/cv/background_mask_filter.h#L121)）。
- **影响**：用户无法在 GUI 调节形态学腐蚀核大小（jAER `erosionSize`）。
- **修复**：在 spec 中添加 erosion_size；在 get_param 中添加 handler。

### 9.3 排除的问题

| 编号 | 描述 | 裁定 |
|------|------|------|
| ALG-5 | TriggerSynced 缺少 t1_us get_param | ❌ 非BUG：`t1_us` 与 `window_us` 冗余（均映射到 `t1_`），spec 已有 `window_us` |
| ALG-6 | EventToVideo `im_Mat_[9]` 死代码 | ❌ 非BUG：属代码清理而非实现错误/GUI不对应；`build_calibration_map()` 预计算 `sum(I-C·C^T)` 但 `reconstruct_interacting()` 使用不同的 Jacobian 公式 `sum(A_i^T A_i)`，二者非同一矩阵 |
| ALG-7 | EventToVideo `decay_tau_ms_{0.0f}` 默认值与 spec `"500"` 不一致 | ❌ 非BUG：AlgoInstance 构造时自动应用 spec 默认值，硬编码默认值对用户不可见 |

### 9.4 修复状态总表

| 编号 | 严重度 | 算法 | 缺失参数数 | 状态 |
|------|--------|------|------------|------|
| ALG-1 | 高 | OrientationCluster | 11 | ✅ 已修复 |
| ALG-2 | 高 | ClusterLIF | 3 | ✅ 已修复 |
| ALG-3 | 高 | TriggerSynced | 2 | ✅ 已修复 |
| ALG-4 | 中 | BackgroundMask | 1 | ✅ 已修复 |

---

## 第十部分：develop 分支系统性代码审查修复（CR-1 ~ CR-8）

> 审查日期：2026-07-15
> 审查范围：develop 分支相对于 main 分支的全部变更（175 文件，+11669/−6944 行，15 个提交）
> 审查方式：3 个子代理并行审查 + 2 个验证子代理交叉验证，确认 8 个真实问题（无误报）
> 验证：283/283 测试通过，无新 BUG 引入

### 10.1 CR-1：PerformanceMeter 线程安全缺失【已修复 ✅】

- **位置**：[performance_meter.h](file:///home/justin/GUI-for-openEB/algo/common/performance_meter.h)
- **描述**：`PerformanceMeter` 被 SDK 数据线程（`tick_events`）和 GUI 线程（`tick_frame`/`reset`/所有 getters）并发访问，但原实现无任何同步机制，存在数据竞争
- **影响**：在高事件率下可能导致 FPS/延迟/事件率统计读数不稳定，或 `rate_estimator_` 内部状态损坏
- **修复**：添加 `mutable std::mutex mutex_` 成员，为所有公共方法（`tick_events`/`tick_frame`/`tick_drop`/`start`/`stop`/所有 getters/`reset`）添加 `std::lock_guard<std::mutex>`

### 10.2 CR-2：在线相机模式算法接收未过滤事件【已修复 ✅】

- **位置**：[main_window.cpp](file:///home/justin/GUI-for-openEB/gui/main_window.cpp)（`install_algo_callback`）
- **描述**：在线相机模式下，`FilterChain`（flip/rotate/transpose/roi/polarity）仅应用于显示渲染，算法实例和 XYT 显示接收的是未过滤的原始事件
- **影响**：`ReplaceStrategy` 算法（如 E2VID、BackgroundMask）会用未翻转的输出覆盖已翻转的显示帧，导致 flip/rotate 功能在算法运行时失效
- **修复**：在 `install_algo_callback` 的 `!file_source` 路径应用 `FilterChain`，使算法和 XYT 显示接收过滤后事件
- **线程安全**：`FilterChain::process()` 和 `has_enabled()` 均持有 `chain_mutex()`（[filter_chain.cpp:25-28](file:///home/justin/GUI-for-openEB/gui/algo_bridge/filter_chain.cpp#L25-L28)），与 GUI 线程的 `set_stage_enabled`/`set_stage_param`（同样持锁）互斥，无数据竞争。与文件模式 `FileFrameGenerator::render_frame` 使用相同的 `has_enabled()` + `process()` 模式

### 10.3 CR-3：相机连接失败 UI 未清理 + 重复对话框【已修复 ✅】

- **位置**：[camera_controller.cpp](file:///home/justin/GUI-for-openEB/gui/app/camera_controller.cpp)（`connect_first_available`/`connect_serial`/`connect_file`）+ [main_window.cpp](file:///home/justin/GUI-for-openEB/gui/main_window.cpp)（`on_connect_first`）
- **描述**：连接失败时 `disconnected()` 信号在 `error()` 信号之后发射，导致 UI 先显示错误对话框再清理连接状态（顺序错误）。`on_connect_first` 中有冗余的 `QMessageBox::information` 弹窗
- **影响**：用户看到错误对话框时 UI 仍显示"已连接"状态；冗余对话框造成双弹窗体验
- **修复**：catch 块中 `emit disconnected()` 移到 `emit error()` 之前；移除 `on_connect_first` 中的冗余 `QMessageBox::information`

### 10.4 CR-4：seek() 后向跳转不重置算法状态【已修复 ✅】

- **位置**：[file_frame_generator.cpp:92-113](file:///home/justin/GUI-for-openEB/gui/app/file_frame_generator.cpp#L92-L113)（`seek`）+ [frame_pipeline.cpp:17-20](file:///home/justin/GUI-for-openEB/gui/app/frame_pipeline.cpp#L17-L20) + [main_window.cpp:853-872](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L853-L872)
- **描述**：文件回放中用户拖动进度条 seek 到较早位置时，算法实例的内部时间戳（如 `time_surface::current_t_`、`E2VID::log_intensity_`）仍停留在 seek 前的时间点。新事件的时间戳小于算法内部时间戳，被算法忽略，导致输出冻结——与 `looped()` 相同的 stale-state freeze 问题，但 seek 未触发重置
- **影响**：seek 后算法输出冻结，XYT 点云混合新旧时间戳事件
- **修复**：`FileFrameGenerator::seek()` 在 `render_frame()` 之前 emit `seeked(t_us)` 信号；`FramePipeline` 转发为 `file_seeked` 信号；`MainWindow` 连接该信号，重置所有算法实例并清空 XYT 显示（与 `file_looped` 处理器相同的逻辑）

### 10.5 CR-5：ActivityBar 拖拽不能交换停靠侧【已撤销 ❌】

- **位置**：[activity_bar.cpp](file:///home/justin/GUI-for-openEB/gui/widgets/activity_bar.cpp)（`mouseMoveEvent`）
- **描述**：`mouseMoveEvent` 中使用 `drag_start_area_`（拖拽开始时的停靠区域）作为判断条件，导致用户在一次拖拽中无法来回交换停靠侧
- **影响**：拖拽体验受限——用户必须松开鼠标后重新拖拽才能换回原来的停靠侧
- **撤销原因**：使用 `current_area` 替代 `drag_start_area_` 的修复以及配套的 `dockLocationChanged` 宽度恢复方案引入了新问题——`dockLocationChanged` 信号在构造函数 `addDockWidget`、`restoreState()`、`reset_layout()` 等非拖拽场景下也会触发，导致 `resizeDocks`/`setMinimumWidth`/`setMaximumWidth` 在不恰当的时机执行，干扰了正常的布局管理流程。经评估无法在不影响其他布局逻辑的前提下干净地修复，故完全撤销此问题的修改，保持原始 `drag_start_area_` 行为
- **补充说明**：撤销过程中发现 `current_area` 检查（原始代码中用于防止拖拽时重复 `removeDockWidget`/`addDockWidget` 的保护条件）被意外移除，已恢复

### 10.6 CR-6：config_manager 仅显示首个未知算法【已修复 ✅】

- **位置**：[config_manager.cpp](file:///home/justin/GUI-for-openEB/gui/config/config_manager.cpp)（`apply_algo_state`）
- **描述**：配置文件中存在多个未知算法名时，`apply_algo_state` 在遇到第一个未知算法后立即 `return ok`（此时 `ok=false`），错误信息只包含第一个未知算法名
- **影响**：用户不知道配置文件中还有哪些算法名是错误的，需要逐次修复
- **修复**：使用 `QStringList` 累积所有未知算法名，循环结束后合并显示

### 10.7 CR-7：事件丢弃率未统计显示【已修复 ✅】

- **位置**：[algo_bridge.h](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.h) + [algo_bridge.cpp](file:///home/justin/GUI-for-openEB/gui/algo_bridge/algo_bridge.cpp)（`AlgoInstance`）+ [information_panel.h](file:///home/justin/GUI-for-openEB/gui/panels/information_panel.h) + [information_panel.cpp](file:///home/justin/GUI-for-openEB/gui/panels/information_panel.cpp) + [main_window.cpp:510-540](file:///home/justin/GUI-for-openEB/gui/main_window.cpp#L510-L540)（`perf_timer`）
- **描述**：`AlgoInstance` 的 flood guard 会丢弃事件（批次截断、自动禁用），但丢弃的事件数从未统计或显示。用户无法知道算法是否在静默丢弃事件
- **影响**：用户可能不知道算法因事件率过高而丢弃大量事件，导致输出不完整却无明显提示
- **修复**：
  - `AlgoInstance` 添加 `total_pushed_`/`total_dropped_` 计数器和 `total_pushed()`/`total_dropped()` getters
  - `push_events` 中累加：禁用/过载时全部计为丢弃；截断时丢弃前段；自动禁用时截断批次也计为丢弃
  - `set_enabled(true)` 重置计数器（新会话 fresh start）
  - `MainWindow::perf_timer` 计算所有 live 实例的最大丢弃率百分比
  - `InformationPanel` 新增 "Drop rate" 标签显示最大丢弃率

### 10.8 CR-8：line_freq_hz 参数类型错误【已修复 ✅】

- **位置**：[algorithms_panel.cpp](file:///home/justin/GUI-for-openEB/gui/panels/algorithms_panel.cpp)（`build_preproc_selector`）
- **描述**：`preproc_filter_line_freq_hz` 在 `AlgoBridge` 注册表中定义为 `enum` 类型（值 "50"/"60"），但 `AlgorithmsPanel` 的 `PDef` 表将其声明为 `'i'`（int）类型，导致构建循环用 `QSpinBox` 创建控件（范围 5-5，只能输入 5），而非 `QComboBox` 让用户选择 50/60Hz
- **影响**：用户无法选择 60Hz 工频陷波，Harmonic 滤波器始终使用 50Hz
- **修复**：将 `PDef` type 从 `'i'` 改为 `'e'`，在构建循环添加 `'e'` 类型处理创建 `QComboBox`（选项 "50"/"60"）

### 10.9 修复状态总表

| 编号 | 严重性 | 修复方案 | 状态 |
|------|--------|----------|------|
| CR-1 | 高 | PerformanceMeter 全方法加 mutex | ✅ 已修复 |
| CR-2 | 高 | install_algo_callback 应用 FilterChain 到算法事件（chain_mutex 保护，线程安全） | ✅ 已修复 |
| CR-3 | 中 | disconnected() 在 error() 之前 emit；移除冗余对话框 | ✅ 已修复 |
| CR-4 | 高 | seek() emit seeked 信号，FramePipeline 转发，MainWindow 重置算法 | ✅ 已修复 |
| CR-5 | 低 | 已撤销——修复引入新问题，保持原始 drag_start_area_ 行为 | ❌ 已撤销 |
| CR-6 | 低 | 累积所有未知算法名合并显示 | ✅ 已修复 |
| CR-7 | 高 | AlgoInstance 计数器 + InformationPanel 丢弃率显示 | ✅ 已修复 |
| CR-8 | 中 | line_freq_hz PDef type 'i' → 'e'，使用 QComboBox | ✅ 已修复 |

---

## 第十一部分：版本历史

| 版本 | 日期 | 作者 | 变更 |
|------|------|------|------|
| 1.0 | 2026-07-13 | Justin | 初始版本，基于 develop `551fa7e` vs main `7d5fde6` 审查（原名 `develop_vs_main_diagnostic.md`） |
| 2.0 | 2026-07-14 | Justin | 重命名为 `gui_diagnostic_report.md`；整合 GUI 全量静态分析（+12 项新发现）；对旧文档 10 项诊断全部验证确认；新增 decay_tau_ms 默认值违规（BUG-G1）；更新修复优先级总表 |
| 3.0 | 2026-07-14 | Justin | 新增第九部分：算法实现参数完整性审查（ALG-1~ALG-7）；修复 4 个参数缺失问题（ALG-1~ALG-4）；排除 3 个非BUG（ALG-5~ALG-7） |
| 4.0 | 2026-07-15 | Justin | 新增第十部分：develop 分支系统性代码审查修复（CR-1~CR-8）；修复 7 个问题（CR-1/2/3/4/6/7/8），撤销 1 个（CR-5 ActivityBar 拖拽引入布局问题） |
