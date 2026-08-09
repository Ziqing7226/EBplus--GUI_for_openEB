# 2.0.0 系统审计与开发计划（基于 main @ caee2c0）

> 本文档替代被撤销的 a94b28a（`doc/systematic_audit.md`），是基于 main 最新状态重新完成的
> 系统审计 + develop / develop-beta 分支取舍分析 + 2.0.0 开发计划。
> 文档目录自此采用 `devlog/`（doc → devlog 改名已采纳，引用更新随 2.0.0 基建提交完成）。

> **发布状态（2.0.0）**：Phase 1–6 全部完成并提交，323 项 ctest 全绿。Phase 4（标定重新设计）
> 三个子提交落地后，2.0.0 路线图全部完成，可作为 2.0.0 版本发布。各 Phase 落地提交见各小节
> "实施状态"；提交已就绪，待用户确认后推送。

---

## 0. 背景与开发原则

- main 已回退至 `caee2c0`（feat: rebuild calibration tool …）；`a94b28a`（旧审计报告提交）已撤销。
  develop 与 develop-beta 分支**保留现状不动**。
- 2.0.0 以**解决 main（caee2c0）已知 BUG 为主**，改动保守——原版实际体验 BUG 不多，
  而 develop / develop-beta 的改动或多或少引入了新 BUG（两分支历史中共记录了 8 处
  "改动自引入、靠现场测试才发现"的回归，见 §3.4 / §4.4）。
- **铁律：一次提交只解决一个问题；每解决一个问题即停下来由用户实测，通过后再继续。**
- **每个修复的验证协议（必须按序执行）：**
  1. 先编译**旧版**（未修复的 main），向用户说明该问题应有的现象；
  2. 用户实测确认问题存在（确认不了的问题不修，见 §6 Phase 3 的先例）；
  3. 再编译**新版**（含修复），用户核验问题已解决；
  4. 核验成功 → 提交保留，由用户决定是否推送；核验失败 → 继续修改并 **amend 本地提交**，
     直到核验通过。
- 新增功能必须零 BUG；GUI 参数必须真实传递到算法（main 上存在一批"调了没反应"的死参数，见 §2）。
- 所有算法变更须同时通过合成单测（test_phase6/7/8_10）与真实 raw 集成测试（test_raw_algos）。

---

## 1. 用户已决事项（本报告的既定输入）

| 事项 | 决策 |
|---|---|
| AVI 导出 | develop-beta 的导出方案整体可取（AVI 为默认格式、导出时长正确）；**遗留问题：进度条只跳变 99%→100%，不显示实际进度**，2.0.0 必须给出正确方案 |
| 调焦功能 | 参考 [inivation DV 文档](https://docs.inivation.com/software/dv/gui/focus-event-camera.html)：屏幕上绘制**缓慢旋转的 Siemens Star**（**预绘制相位帧**，运行时只做贴图，见 §6 Phase 5）；用户目视调焦；**完全移除现有锐度计算** |
| dv-processing 移植 | 保守移植三项：**KNoise 滤波器模式**（仅此一种滤波器）、**eArc/Arc\* 角点检测**、**TimeSurface 指数 decay 模式** |
| 标定功能 | 重新设计：**非对称圆点阵**（黑底白点、不闪烁、**忽略事件极性**）；点阵界面旁边显示相机输出；屏幕上有小字提示"按空格键捕捉"；用户按**空格主动抓拍 500µs 窗口内的事件累加成帧**，算法判断抓拍质量并取舍；**抓拍不降采样**；GUI 重新设计，符合当前体系且用户友好 |
| 文档目录 | doc → devlog 改名采纳 |
| filtered_events 回显 | **已否决，改为显示路径预处理**（滤波/去畸变/降采样抽稀进主显示与录制；ROI 概念合并为硬件 ROI 另立 Phase 2.6，见 §6 历史记录与两节设计） |

---

## 2. main @ caee2c0 经核实的已知 BUG 清单

旧审计报告（a94b28a 版本，937 行）审计对象与当前 main 是同一提交，抽查 25 条**全部属实、
行号偏差 ±3 以内**。以下为复核后的清单（按严重程度）。

### 2.1 严重（3 项）

1. **corner_detector 模式枚举 GUI 标签全错位** — `gui/algo_bridge/algo_bridge.cpp:750` 注册
   `{"0=Harris","1=FAST","2=AGAST"}`，`backends/cv_backends.cpp:304-306` 裸 `static_cast<Mode>(m)`，
   而 `algo/cv/corner_detector.h:55` 枚举是 `{EndStopped, TypeCoincidence, Harris}`。
   界面选 Harris 实际跑 EndStopped，三个标签全假。
2. **ObjectTracker RCT/Median 模式跟踪失效（默认配置）** — `algo/cv/object_tracker.h:543-565`
   `update_velocity` 每事件直接覆盖 `vx_`、无任何低通（jAER 有 velocityTauMs=100ms）；
   `age(dt)`（:278-282）按整包间隔外推位置 → 静止目标漂移飞出传感器、簇反复重建。
   `algo/tests/raw_algos` 的错误注释一直在掩盖此问题。
3. **AVI 导出帧大量丢失 + 尾部丢弃** — `gui/exporter/exporter_controller.cpp:200`
   `CDFrameGenerator(w, h)` 缺省 `process_all_frames=false`：每缓冲批只产最后一帧
   （导出视频时长被严重压缩，60s 录像可能只出几百帧），且 `stop()` 直接 abort
   丢弃尾部未处理事件。SDK 源码佐证：`openeb/sdk/.../cd_frame_generator.cpp:96-99,142-149`。

### 2.2 高（6 项）

4. **orientation_filter / direction_selective 着色事件错位（默认必现）** —
   `filter_backends.cpp:61-77` 分类针对 ROI 过滤后的 `ev[i]`，:80-91/:157-183 pull 时却与
   未过滤的 `passthrough_[i]` 按下标配对，颜色和位置全错。
5. **trigger_synced 恒零输出** — `add_trigger`（`algo/cv/trigger_synced_filter.h:71`）无任何调用方，
   GUI 无 Trigger In 接线。
6. **HoughCircle 节流丢整批事件** — `cv_vector_backends.cpp:251-262` 注释谎称"累加器仍接收事件"，
   实际早退在 `process()` 之前，事件根本没进累加器；且衰减 dt 被拉长到 50ms+，每次检测前
   累加器内容被乘 ~0.2 几乎清空 → **检测能力崩塌**（不是简单的丢数据）。
   注意：防卡顿的 20Hz 节流本身是对的、必须保留；正确修法是便宜的 accumulate 每批都喂、
   贵的 find_peaks 仍 20Hz 节流（见 §6 Phase 1）。
7. **`preproc_downsample` 默认 ON 但只有 5 个后端做坐标减半** — `algo_bridge.cpp:264` 默认 true；
   其余 ~19 个后端被静默抽稀 75% 事件且坐标不变。
8. **播放中切换文件后新文件永不自动播放** — `playback_controller.cpp:84-122` `open_file()`
   不重置 `playing_`，`play()` 被旧标志短路。
9. **24 个自研后端仅 8 处实现 `set_sensor_dimensions`，换源后 ROI 假死** — 未连相机先建实例、
   再接 640×480 源时 ROI 按 1280×720 算，算法静默丢光事件。

### 2.3 中（8 项，摘要）

10. 过期 EOF/错误 lambda 停掉新相机 — `camera_controller.cpp:247-264` 只查 `if (!camera_)`，
    源 A 的回调可 stop 已换上的源 B。
11. ObjectTracker `prev_batch_t_{0}` 无首包哨兵 — `object_tracker.h:678`，大时间戳首包全簇误删。
12. TimeSurface Split 模式亮度恒减半 — `time_surface.h:116-118` `(c_off+c_on)*0.5` 饱和加后乘半。
13. E2VID 热像素掩码坐标错位（潜伏，GUI 未接线）— `e2vid_inference.h:116` 半分辨率网格
    索引全尺寸 mask。
14. intrinsic.cpp AsymmetricCircles 物点网格公式错误（潜伏）— `intrinsic.cpp:43-51`，
    正确应为 `x=(2c+r%2)*square, y=r*square`。**2.0.0 标定重启用圆点阵，此项变为必修。**
15. background_mask "learning_rate" 误接 `set_learning_window_s` — `filter_backends.cpp:250`。
16. DWF 窗长注册默认 2、上限 100（jAER 工作点 512 不可达）— `algo_bridge.cpp:279`、
    `algorithms_panel.cpp:435`。
17. flood guard 按批大小校准，文件模式误杀 — `algo_bridge.cpp:153-167`，高速文件 4 帧后
    算法被自动禁用且 UI 仍勾选。
18. （对应 7）文件 OOM 无防护 — `file_frame_generator.cpp:22-37` 全量驻留无上限，
    大文件+快进会中途误判 EOF。

### 2.4 低 / 用户可见死参数（"调了没反应"）

- `n_sigma`（hot_pixel_filter，自标 deprecated）、`min_radius`/`accumulator_decay_us`
  （hough_circle/line，自标 legacy unused）、xyt `max_points`、多处 `min_events` ——
  全部注册在 GUI 但算法不读。
- intrinsic.cpp `ok=true` 先于 LUT 预计算，异常时 ok 与 error 并存（`intrinsic.cpp:139-153`）。
- 导出路径不查 source==output（可覆盖源文件）；导出 worker 只 catch `std::exception`；
  取消导出后半成品 AVI 残留。
- H264 编码器缺失时被误报为"路径不可写"（`cv_video_recorder.cpp:24-34`），且对话框默认
  quality=90 正好走 H264。
- **filtered_events 无消费者**：hot_pixel_filter、ultra_slow_motion 等"输出=事件流"的算法
  启用后主显示完全无变化（已核实 main 代码无消费路径）。

---

## 3. develop 分支改动总结（main..develop，11 提交，+5196/−4433）

分支形状：审计文档（a94b28a）→ **1 个 119 文件的 mega-commit（de0e607）** →
回归审计（6edcea2）→ 8 个回归修复小提交。`doc/systematic_audit.md`（937 行）是全部改动
的设计文档，§十一/§十二 记录了 mega-commit 自引入回归的根因。

### 3.1 按主题

- **A. 审计文档**（a94b28a + 6edcea2）：jAER 移植比对、死代码 32 处、算法 BUG 22 项、
  桥接 33 项、GUI 30+ 项、标定/锐度专项。零风险，是最有价值的工程资产。
- **B. jAER 移植修复**（de0e607）：Repetitious 短 ISI 分支误移植（>1kHz 像素持续丢事件）、
  direction_selective ori 路径方向错位 90°、optical_gyro 虚构外推、trigger_synced O(n²) 重扫。
  方向正确但改变多个算法输出语义，需逐项验收。
- **C. 死代码清理**（de0e607，−2000 行量级）：4 个无引用 widget、4 个不可达 openeb_*_backends、
  noise_tester、FilteredEventPacket、intrinsic LUT 死链、perspective_undistort（本身已坏）、
  20 个零调用方法、11+ 个死参数。风险：旧配置键被静默跳过；**decay_tau_ms 删除系误判**
  （§12.2-A 自承），与 beta 决策直接冲突。
- **D. 算法库 BUG 修复**（de0e607）：ObjectTracker 低通（全 develop 最有价值的单修复）、
  TimeSurface 亮度减半、prev_batch_t_ 哨兵、ParticleCounter/Hough 无界 map、14 项低危修复
  （哨兵/clamp/UB/溢出）。都在算法类内部，低-中风险。
- **E. 桥接修复**（de0e607 + 3 个补丁）：corner 标签、G2 着色、HoughCircle 拆分、
  set_sensor_dimensions 补齐、flood guard 改速率制、E2VID 线程化、downsample 默认改 OFF、
  配置迁移表。**风险最高主题**：动了锁语义、线程模型、配置加载路径；自身引入冷启动丢包、
  pause-resume 破坏等回归，且 tip 上仍残留 30 Mev/s 误杀、drop-OLDEST 队列两个问题。
- **F. GUI 功能修复**（de0e607 + 2 个补丁）：播放切文件、loop 失效（修复过程中先引入
  loop 回归再修）、AVI process_all_frames + 背压、标定事件重复注入、~25 项小修复。
- **G. 标定/锐度重写**（de0e607 + 2 个补丁）：角点位移查重替 MSE、内角点约定统一、
  worker 线程化、棋盘 HUD、锐度计改事件计数图。初版引入"棋盘闪烁不佳""锐度卡顿"两个
  用户可见回归，补丁修复；LCD 闪烁环境下检测仍不稳（beta 后续才解决）。

### 3.2 develop tip 仍残留的问题（由 beta 后续修复证明）

1. flood guard 30 Mev/s 误杀 E2VID 闪烁板场景；2. E2VID drop-OLDEST 队列重影；
3. decay_tau_ms 误删（E2VID 帧变亮 ~6%）；4. 锐度/标定 cd_broadcast 共享冲突未修。

### 3.3 评价

mega-commit 形态（119 文件单提交）不可 review、不可 bisect、不可选择性回滚，是"先制造回归
再修补"的过程产物。**develop 的每一项有价值内容在 develop-beta 中都有等价或更优版本。**

## 4. develop-beta 分支改动总结（main..develop-beta，45 提交，+7191/−4694）

beta = 同一份审计的**拆分重做版**（每个主题独立提交）+ 25 个 develop 没有的实测修复，
内容上近似 develop 的严格超集。分支内自引入并已修复的回归 4 处（见 4.4），全部落在 tip。

### 4.1 纯文档（7 提交，零风险）

审计报告 + **`899e8c6` doc/→devlog/ 改名**（同步 README/wiki/CMake/.gitignore）+ 设计文档/wiki 同步。

### 4.2 算法正确性修复（`7bcf316` + `954fef5`）— 全部针对 main 真实 BUG

棋盘内角点约定（load-bearing：UI 说 9×6 内角点，intrinsic 按 8×5 搜，检测必败）、
object_tracker 哨兵、AsymmetricCircles 物点公式、direction_selective 90° 错位、
Repetitious 语义、optical_gyro 外推、time_surface 双极性合成、无界 map 等。
改变算法输出数值（对齐 jAER/OpenCV），不动数据结构/线程。

### 4.3 按组清单（✅=低风险可直接取，⚠️=需整体取+实测）

| 组 | 提交 | 内容 | 分级 |
|---|---|---|---|
| 算法正确性 | `7bcf316`, `954fef5` | 见 4.2 | ✅ |
| 死代码 | `195d36a`, `50745ed`, `5fb8f2f` | widget/noise_tester/openeb_* 后端删除；**注意 5fb8f2f 有意删除面板可见的 perspective_undistort** | ✅（附决策） |
| 参数正确性 | `232dfcc` | corner 标签、float clamp、learning_rate 误接、9 处默认值漂移、配置迁移 | ✅ |
| bridge 结构 | `70e812a` | D1 换源假死、G2 着色、G4 hough（accumulate/find_peaks 拆分，**20Hz 节流保留**）、D4 ROI 平移、C2 持锁 | ✅ |
| 相机/面板 UI | `8f31713` | C1 过期回调、C4 假 Connected、U1 bias 滚轮不落硬件、U5 stoi 崩溃 | ✅ |
| 显示防御 | `625046e` | paintGL 除零、ROI clamp、文件 OOM 防护（300M 硬顶+信号） | ✅ |
| 播放 | `efd35ad`, `f38312a` | P1 playing_ 复位、EOF/loop 状态机、seek clamp、OSC 重试 | ✅ |
| 收尾 | `ab0cc22`, `e5ca677`, `04dd0d3` | config 互斥恢复、AlgoInfo description、文档修正 | ✅ |
| flood guard | `6e342b4`+`77d4b97` | 速率制+面板同步；**两个必须一起**（后者修前者 30M 误杀回归，终值 100 Mev/s） | ⚠️ |
| AVI 导出 | `78e8704`+`d197581`+`d88d5bb`+`be035d7`+`bda1417`+`729e9ab` | 见 §5 | ⚠️ 六个整体取 |
| E2VID 线程化 | `cb451ee`+`52a7ee9`+`ab0cc22`(e2v 部分)+`45c2542` | 4 锁 worker+有界队列+双缓冲；**必须整体取**；**2.0.0 改为复现驱动、可整体跳过，见 §6 Phase 3** | ⚠️ 待复现决策 |
| 标定重做 | `c375ecc`…`0d0f344`（8 提交） | worker 化、全周期闪烁像素检测、LCD 翻峰对齐、E2E 测试 | 不取（2.0.0 走新设计，见 §6 Phase 4） |
| 对焦工具 | `1fef78f`…`cda4b59`（5 提交） | DFT 对焦工具+闪烁图案；**2.0.0 改走 Siemens Star 无计算方案**，见 §6 Phase 5 | 不取 |
| filtered_events 回显 | `c23e0e0` | 滤波算法输出回注主显示 | ❌ 已否决（实测卡顿，见 §6 Phase 2.5 历史记录；改走显示路径预处理） |

### 4.4 分支内自引入回归实证（"rework 引入新 BUG"的全部记录）

1. `77d4b97`：flood guard 30 Mev/s 误杀（`6e342b4` 引入）；2. `52a7ee9`：E2VID drop-oldest
   队列重影（`cb451ee` 引入）；3. `cda4b59`：对焦 worker 迭代器越界 segfault（`7ce5ebc` 引入）；
4. `ab0cc22`：config 互斥 hunk 重做中丢失。全部已在 tip 修复。
残留小瑕疵：对焦 stride 循环严格意义 UB（`sharpness_dialog.cpp:457/472/484`，
2.0.0 不取该工具则无关）。

### 4.5 两分支决策冲突点（beta 的选择均经实测验证更优）

- decay_tau_ms：develop 删 / beta 保留并统一默认 500 → **采 beta（保留）**。说明：该参数是
  event-to-video 的帧间时间衰减（`exp(-dt/tau)`），防止 log_intensity 累积产生残影；
  AGENTS.md 硬性要求 GUI 暴露 [0,5000] 默认 500；删除后有可观察的逐帧调光差异（帧变亮 ~6%）。
  注意保留并统一默认 500 会改变开箱行为（默认开启 per-frame dimming）。
- flood guard：30 Mev/s / 100 Mev/s → **采 beta**；
- E2VID 队列：drop-OLDEST / 无损背压 → **采 beta**（若 Phase 3 经复现决定做）。

### 4.6 开箱行为变更清单（合并即生效，需逐条确认）

`preproc_downsample` 默认 开→关（改为按算法自动）；`decay_tau_ms` 默认 0→500；
导出默认格式→AVI；flood 阈值 100 Mev/s；删除 perspective_undistort 面板项。

### 4.7 名词澄清（供取舍判断）

- **release_resources**：develop 添加的后端虚函数，禁用时显式释放重资源（ONNX 会话等），
  动机是修 dock 拖拽 segfault；但禁用时卸载 ONNX 导致每次暂停/恢复/A-B 切换付 300–500ms
  模型重载，develop 自己回退了调用、只剩死机制。2.0.0 **不引入**：禁用算法保持资源加载
  （瞬时恢复），与 main 现状一致。
- **EdgeMap（dv）**：dv 的边缘图累加器——整型帧 + 256/512 项 LUT，单事件只贡献 ~25% 亮度，
  同像素需多个事件才饱和，边缘纹理比二值事件帧清晰；decay 按帧查表步进。属 dv 调研的
  中价值候选（新显示窗口），按"避免冗余"原则**不移植**。

---

## 5. AVI 导出：根因与 2.0.0 正确方案

### 5.1 main 上的三重 BUG（已逐行核实）

1. `exporter_controller.cpp:200` `CDFrameGenerator(w, h)` 缺省 `process_all_frames=false`
   → 每缓冲批只产最后一帧，**导出视频时长被严重压缩**；
2. `stop()` abort 丢弃 `events_back_` 尾部事件 → **结尾缺失**；
3. `Metavision::CvVideoRecorder` 线程化帧池在生产快于编码时**静默丢帧**；
   附带：fps 同时被当帧周期、H264 缺失误报权限、无 source==output 检查、cancel 残留半成品。

### 5.2 beta 修复链（六提交整体取，已含端到端帧数验证 2999/3000）

process_all_frames+背压（78e8704）→ 帧周期绑定 accumulation 慢动作语义（d197581）→
静默段补黑帧（d88d5bb）→ 换同步 cv::VideoWriter 去线程池（be035d7）→
进度上报+默认 AVI（bda1417）→ 三个线程 bug 修复（729e9ab）。

### 5.3 beta 遗留问题：进度条只跳 99%→100%

根因方向：进度依赖帧写出回调计数 / 总量估算，而总量要到 EOF 才确知，中间无有效上报点。
**2.0.0 正确方案**：文件时长在导出前即可获知（`get_duration`，OSC 文件需 start 后查询），
进度 = 已处理事件时间戳 / 文件总时长，在 cd 事件回调（或帧回调）中以节流频率
（如每 200ms）上报——与编码速度、帧数估算完全解耦，天然平滑单调。此修复作为
AVI 组的第 7 个独立提交（一次提交一个问题）。

---

## 6. 2.0.0 开发路线图

排序原则：先低风险纯修复（快速消除 main 已知 BUG），再动线程模型，最后做新功能。
**每一步 = 一个提交 + §0 验证协议（旧版复现 → 用户确认 → 新版核验 → 通过才保留提交）。**

### Phase 1：低风险 BUG 修复（均从 beta 摘取，逐组一提交）

1. `899e8c6` doc→devlog 改名（基建，含 README/wiki/CMake 引用）；
2. `232dfcc` 参数正确性（corner 标签、float clamp、learning_rate、默认值对齐、配置迁移）；
3. `954fef5`+`7bcf316` 算法正确性（含标定内角点约定、object_tracker 低通与哨兵——
   注意会改变算法输出，需逐项说明）；
4. `70e812a` bridge 结构（D1/G2/G4/D4/C2）。**G4 说明**：HoughCircle 修复保留 20Hz 节流
   （防卡顿机制不变），只是把便宜的 accumulate 移回每批执行、find_peaks 仍节流——
   修的是"累加器被饿死 + 衰减 dt 被拉长导致检测力崩塌"，不是取消丢事件防卡顿；
5. `efd35ad`+`f38312a` 播放状态机；
6. `8f31713` 相机/面板 UI；
7. `625046e` 显示防御（含文件 OOM 防护）；
8. `ab0cc22`+`e5ca677` 收尾（config 互斥、AlgoInfo description）；
9. 死代码三提交（`195d36a`/`50745ed`/`5fb8f2f`），perspective_undistort 删除单独确认；
   同组决策两条旧审计遗留死代码项（§8.2）：`PerformanceMeter` 无调用 API 删或留、
   7 个 standalone 诊断程序注册 CTest 或删除；
10. `6e342b4`+`77d4b97` flood guard（两提交可合并为一次取，取后测闪烁板+E2VID 场景）；
11. **旧审计遗漏补网**（§8.2，beta 两分支都漏掉的 5 个行级修复，各一提交）：
    a. undistort 预处理链三问题（§五-F3，`backend_common.h`）："仅 undistort 启用且 LUT
       无效"时 `out.assign(out.data(),...)` 自赋值 UB（一行修）；`cv::undistortPoints`
       异常逃逸被 `catch(...)` 吞（加 catch+qWarning）；YAML 加载失败静默清 K（一次性
       状态栏提示）。**须先于 Phase 4 修**——标定产出物正是这条链的输入；
    b. 算法异常静默死亡（§五-H3）：`main_window.cpp:1375,1520` 两处 `catch(...){}`
       加一次性 qWarning/statusBar 提示+计数；
    c. background_mask Standalone 死分支（§五-G5，`main_window.cpp:1806-1808`）删除；
       `mat_to_qimage` 对非 1/3 通道 Mat 静默返回空图 → 加 qWarning；
    d. Config 加载后 AlgorithmsPanel 控件不刷新（§5.9-疑点4）：config 加载后按实例
       实际值刷新控件（用户可见："显示的数和跑的不一样"）；
    e. Refractory 非单调事件放行（§一-1.3，`noise_filter.h:368`）：`e.t < lt` 直接放行
       与 jAER 不符（jAER 滤掉时间回退事件）——对齐 jAER 一行改动；
    f. 顺手注释定档（不改行为）：DWF 单窗模式窗口减半（`noise_filter.h:408-409`）、
       LocalPlanes 缺 jAER 50ms per-pixel refractory（`sparse_optical_flow.h`，
       输出密度差异非错误）、`filter_chain.h:49-53` 注释引用无声明的 `chain_mutex()`；
12. **E2VID 模型加载失败提示（H1，从 Phase 3 解耦）**：main 上 ONNX 加载失败静默降级
    为 heuristic 重建，用户会误认为"E2VID 质量"。修复在 beta `cb451ee` 内但与线程化
    无耦合（status 行显示 model=loaded/heuristic + 面板一次性 error_message）——
    拆为独立小提交落入本 Phase，**保证 Phase 3 即使整体跳过也不丢失此项**。

### Phase 2：AVI 导出（§5.2 六提交整体取 + 新增进度修复提交）

- 六提交作为一个功能组落地（内部仍是逐提交 cherry-pick，保持一次一问题粒度）；
- 第 7 提交：进度条改为"已处理事件时间戳/文件总时长"节流上报（§5.3）；
- 配套小修：H264 失败回退 MJPG+正确报错文案、source==output 拒绝、cancel 删半成品。
- 实测场景：慢动作、静默段、中途取消、长文件内存。

### Phase 2.5：显示路径预处理（滤波 + 去畸变 + 降采样进主显示与录制）

> **历史记录**：旧方案（`c23e0e0` filtered_events 回显）已实施后被用户实测否决并撤销
> （未推送）。否决原因：① `filtered_events` 字段语义在各后端不一致（透传/滤波后/
> ROI 相对坐标），回显把正确性押在一个不可靠的字段上；② 大多数后端返回透传流，
> 启用任何算法都会触发每帧全尺寸重渲染（720p×60fps 的 GUI 线程内存 churn）→
> 其他算法明显卡顿；③ ultra_slow_motion 的时间拉伸在静态帧上不可见，回显无意义。
> 教训：**显示效果应在事件路径上实现，而不是事后从算法结果重渲染。**

**新设计（用户已确认方向）**：把 Preprocessing 面板的预处理级应用到**显示事件路径**——
预处理从"算法输入级"升级为"显示级"，jAER 语义（去噪器作用于观察流）。

- **注入点**：`FramePipeline::add_events()`（live 与文件回放统一经过）；文件路径的
  `FileFrameGenerator::render_frame()` 在 FilterChain 之后追加同一处理。
  单点注入，关闭时与现状逐字节一致。
- **进显示路径的级**（三级全进，用户已确认）：
  ① 噪声滤波（8 模式）；
  ② 去畸变（用户明确要求：去畸变应影响主显示）；
  ③ 降采样——**按"抽稀滤波器"语义进入**（用户明确要求）：仅做事件抽稀
  （偶数坐标保留），坐标不变，主显示按原始分辨率显示抽稀后的事件。
  **不做坐标减半**——坐标减半仍属算法输入语义（halve_coords 后端照旧）。
- **配置来源**：复用 Preprocessing 面板的现有控件与参数（filter_enabled/mode/参数 +
  undistort_enabled/path + preproc_downsample），不新增 GUI 概念；
  显示与算法输入看到同一配置（算法侧的降采样仍走 halve_coords 后端自有逻辑）。
- **录制联动（用户明确要求）**：滤波/去畸变/降采样任一级启用时，录制写**处理后**
  的事件流（`RAWEvt2EventFileWriter`，经显示路径的同一处理后写出），不再是 SDK
  原始 `log_raw_data`；全部级关闭时保持 `log_raw_data` 原始记录。bias 保存行为不变。
- **性能**：滤波为每事件 O(1)~O(9)（DWF 略高）；去畸变 LUT 查表 O(1)；抽稀 O(1)。
  高事件率场景实测帧率影响。
- **配套清理**：`ultra_slow_motion` 从 GUI 注册表删除（用户决定；算法文件一并删除）。
  理由：其设计承诺（Replace 主显示）从未实现，时间拉伸作为 GUI 显示无意义，
  下游消费者不存在。`hot_pixel_filter` 保留现状（用户决定）。
- **拆分提交**：①注册表删除 ultra_slow_motion（+测试清理）②显示路径滤波
  ③显示路径去畸变 ④显示路径抽稀 ⑤录制联动，各一提交一测。

### Phase 2.6：ROI 概念合并（统一为硬件 ROI）+ 主显示 ROI 显示模式

> 用户方向：算法 ROI 与硬件 ROI 两个概念合并为**一个 ROI**，直接使用硬件 ROI 功能；
> 同时主显示下方新增 ROI 显示模式按钮。**此 Phase 危险度高**（触及显示/录制/
> 全部算法输入路径），实施前必须完整核对数据链路（开发前重读：
> `camera_controller.cpp` cd 回调链、`frame_pipeline.cpp` 两条路径、
> `file_frame_generator.cpp` 渲染链、24 个后端的 ROI 消费点、
> `draw_roi_overlays`、RoiPanel↔硬件 ROI 双向同步、ConfigManager 的 roi_* 配置项）。

**设计（用户已逐项确认）**：

- **唯一 ROI 概念**：live 相机用硬件 ROI（`I_ROI` 设施，支持 ROI/RONI 模式，
  RoiPanel 已在用）；**文件回放在 `FramePipeline` 注入点做软件裁剪模拟**
  （用户确认）——ROI 语义全场景统一。
- **算法 ROI 机制删除**：24 个后端的 `roi_*` 参数、`RoiFilter`/`crop_to_roi` 输入
  裁剪路径、`draw_roi_overlays` 的算法 ROI 黄框、`algo_bridge` 的
  `apply_global_roi`/`roi_cache_` 全部移除——算法输入在事件源处已被统一裁剪，
  后端不再需要各自裁剪。**但保留机制中蕴含的用户指示**：
  ① "哪些算法默认启用 ROI"的知识——复杂算法（e2v/isi_analyzer 等）启用时
  **自动把统一 ROI 应用到默认中心区域**（默认启用清单逐算法核对保留）；
  ② AlgoWindow 的"算法处理区域放大显示"视图保留（纯显示功能，用户明确可取）。
- **旧配置兼容**：ConfigManager 加载含 `roi_*` 键的旧配置时，映射到统一 ROI
  （首个启用算法携带的 ROI 作为全局 ROI），不再按实例应用；未知键按现有
  qWarning 路径提示。
- **主显示 ROI 显示模式按钮**（主显示窗口下方新增）：两种模式——
  (a) **自适应放大**：仅显示 ROI 区域内容并放大至窗口尺寸；
  (b) **原始分辨率**：全幅画布，仅 ROI 区域有内容（用户选定**默认 (b) 全幅**）。
  硬件 ROI 启用后事件天然只在 ROI 内，(b) 即现状；(a) 为帧裁剪放大。
- **RoiPanel↔AlgorithmsPanel 去重**：ROI 控件收敛到一处（保留 RoiPanel 的
  硬件 ROI/RONI + 拖拽 + 预设；AlgorithmsPanel 顶部的 Algorithm ROI 选择器
  改为驱动同一 ROI，或并入 RoiPanel——实施时先出方案再写码）。
- **录制**：硬件 ROI 天然作用于录制（相机只输出 ROI 事件；文件源录制本来就被禁止）。
- **拆分提交**：①统一 ROI 基础设施（live 硬件 + 文件软裁剪 + 面板驱动切换）
  ②删除算法 ROI 机制（后端/bridge/黄框/旧配置映射）③默认启用清单迁移
  ④主显示模式按钮，各一提交一测，② 前后各做一次全算法回归。
- **实施状态**：①~④ 已提交（`caf37d1`/`90ce1ad`/`c98f338`/`e314637`）。实测暴露 7 个回归/
  问题后，Phase 2.6 Debug D-1~D-7 全部修复并提交（`455d505` D-1~D-4 + `a6c14f2` D-5~D-7），
  323 项 ctest 全绿。**Phase 2.6 完成。**

### Phase 2.6 Debug：实测回归修复 + ROI 全面统一（GUI 收敛）

> 触发：Phase 2.6 实测暴露 7 个问题。已对基线 caee2c0 与 d32f5e8..HEAD 做逐文件
> 考古，以下根因全部代码核实。**用户约束**：不丢弃任何捕捉到的事件（哪怕积压）、
> 只优化性能、不魔改、输出必须是传感器+预处理的完整结果；GUI 一律全英文。

#### D1 算法小窗显示回归（考古结论：Phase 2.6 未严格保持基线小窗逻辑）

**基线（caee2c0 代码核实，自 92cb363 起所有版本一致）的小窗显示语义**：

- 主显示 = 全幅累积帧 + 在算法处理区域绘制的 overlay（圆/线/框/着色事件/轨迹，
  `OverlayStrategy` → `FrameAnnotator`，`process_algo_results` 在 `set_frame` 之前）；
- Overlay/self 算法小窗 = **zoom 放大视图**：已绘制 overlay 的主帧按算法 ROI
  （per-backend `roi_enabled` 默认 true、中心 128×128——**基线历史默认，当前新
  默认为 256×144，见 D3**）裁剪放大——即用户描述的"事件累积帧+检测到的圆/线"
  的放大图。hough 的 aux 累加器图每帧先于 zoom 推入同一 widget、被 zoom 覆盖，
  **基线实践中从未可见**（用户确认，本 Debug 删除）；仅用户手动关 ROI 时小窗才
  显示 aux（旧代码全幅时 aux 同样巨大——"大 aux"是全幅的自然结果，回归在于
  **默认态**从 128×128 变成全幅）；
- Standalone（e2v/ISI/TimeSurface）小窗 = 自有帧；Passive/freq_detector = 仅 status
  文本；background_mask（Replace）小窗无 display widget。

**Phase 2.6 的错误**：把"per-backend ROI 默认开"改为"统一 ROI 默认关"时未保持
小窗显示逻辑——zoom 被统一 ROI 门控（R1）、坐标平移未配套回移（R2）、默认态
变全幅（R3）。另注：本人曾提议"小窗=主显示镜像"系错误——基线小窗是**放大视图**，
非同尺度镜像（用户纠正，目标设计已确认=恢复基线行为）。四处回归：

- **R1（14 个 Overlay/self 算法小窗永久空白）**：orientation/direction_selective/
  sparse_optical_flow/blob/object_tracker/corner/line_segment/orientation_cluster/
  cluster_lif/bandpass/overlay/active_marker/particle_counter/auto_bias/optical_gyro
  等小窗唯一帧源是 zoom view，现由统一 ROI 门控（默认关）→ 永远收不到帧，
  "No camera connected" 占位常驻（`display_strategy.cpp:203-225`）。
- **R2（统一 ROI 开启时主显示 overlay 坐标错位，新引入的真 BUG）**：
  `AlgoInstance::push_events` 把事件平移为 ROI 相对坐标（`algo_bridge.cpp:211-217`），
  但 overlay 结果画回主显示时**没有任何路径平移回传感器坐标**；hough 的 shift-back
  依赖内部 `roi_.enabled`（现恒 false → 偏移 0，`cv_vector_backends.cpp:123-131`）。
  后果：overlay 画在左上角偏移区而非 ROI 上；主显示 "Zoom to ROI" 放大视图里
  也只有事件、没有 overlay（小窗 zoom 路由已随 R1 决策删除）。
  Replace 模式的 background_mask 输出帧同理尺寸/位置不匹配。
- **R3（hough 小窗默认显示巨型 aux）**：zoom 停推后 aux 成为小窗唯一内容——
  hough_line aux=1468×90（基线默认 181×90）、hough_circle aux=640×360（基线
  64×64，大 45 倍）；且旧注释自认的"默认 128×128 限制 ~750K cells 防 freeze"
  保护已被默认关闭（`cv_vector_backends.cpp:171-176` 注释过时）。
- **R4（XYT 3D 云被自动 ROI 裁剪）**：XYT 从不用 ROI（用户确认），移出名单即恢复。

**修复方案（目标=恢复基线行为，用户已确认；叠加用户新决策——主显示 zoom 已
逐像素等价于小窗放大视图，Overlay 小窗冗余）**：

- **R1=Overlay 算法不再弹 AlgoWindow**（orientation/hough/object_tracker 等 14 个）：
  主显示画 overlay（基线行为）+ 主显示 "Zoom to ROI" 放大（2.6④）已覆盖其小窗
  全部内容；原小窗唯一增量信息 **status 文本（检测数/有效参数回显）移到侧栏**——
  该算法组内新增一行只读标签，每次 pull 更新。display_strategy 的 zoom 路由随之
  删除。Standalone（e2v/ISI/TimeSurface，自有帧不上主显示）与 Passive（status
  文本窗）的 AlgoWindow **保留不变**。
- **R2=OverlayStrategy 绘制时统一 +(x0,y0) 回移**（覆盖 RoiFilter 系与 hough 系；
  Replace 模式同查）。
- **R3=删除 hough aux 显示**（用户确认：基线实践中 aux 从未可见——每帧被 zoom
  覆盖，仅手动关 ROI 时成为小窗唯一内容；7ad7160 仿 jAER 的调试视图）：
  hough_line/circle 后端不再生成 `aux_frame`（省每帧 colormap 开销）、
  display_strategy 的 aux 路由删除；hough 进默认名单恢复小工作域（256×144）+
  修过时注释（`cv_vector_backends.cpp:171-176`）。原"限宽防御"方案取消。
- **R4=XYT 移出名单。**

#### D2 P1 高事件率或然黑屏（结论：非性能回归，最可能是 R5）

- 用户现场确认：出事时滤波/降采样/去畸变全关。008d121..HEAD 逐文件 diff：
  **预处理全关时热路径零显著增量**（每 CD 批仅 +1 次无竞争锁 + 布尔判断，~几十 ns；
  文件路径全关时零拷贝）。滤波器语义解释（SpatialBP/Harmonic）已排除。
- **最可能根因 R5**：Phase 2.6③ 的自动硬件 ROI——启用 e2v/ISI/XYT/TimeSurface 时
  传感器物理上只输出中心 128×128 事件（当前 HEAD 的自动矩形；D6 将改为 256×144）
  → 主显示大面积无事件。场景活动在中心外时
  即"或然不显示任何事件"；此行为 2.6 之前不存在，与"以往没出现过"的时间线吻合。
  Debug 的自动化语义重做（256×144 默认 + 恢复前态 + 黄框/zoom 可见）使其可预期。
- 用户否决"丢批/mailbox"式防护（不丢事件）。**本 Debug 不做 P1 代码改动**；
  若完成后仍复现，再按候选做 instrumentation（时间戳离群检测、帧延迟埋点）。
- 缓议（条件性性能项，仅在 preproc 开或 processed 录制时存在）：SDK 线程上的
  O(n) 拷贝+滤波（`frame_pipeline.cpp:146-152`）、录制写盘上 SDK 线程
  （`recorder_controller.cpp:133-154`）——与 Phase 3 复现决策合并评估。

#### D3 ROI 全面统一（GUI 收敛，用户已逐项决策）

- **单一状态源**：`CameraController`（live 写 I_ROI；file 转发 FileFrameGenerator）。
  所有写入者（勾选框/弹窗/拖拽/配置加载/自动化）一律经 `set_unified_roi`；
  新增 `roi_state_changed(enabled,x0,y0,x1,y1)` 信号单驱动：黄框、zoom 按钮、
  `AlgoBridge::set_unified_roi_state`、两页面勾选框回显。状态扩展 `mode`（ROI/RONI），
  **文件软裁剪配套支持 RONI**。修复"RoiPanel 直连 facility 不更新缓存 → zoom 变
  128×128 + 黄框残留"的实测 BUG（缓存只在 set_unified_roi 内写即永远新鲜）。
- **GUI 合并**：硬件页、算法页各保留 **"Enable ROI" 勾选框 + "ROI Settings..." 按钮**；
  勾选→开启并弹窗，取消→关闭；按钮随时再开弹窗；两勾选框经信号双向同步。
  **删除** PreprocessingPanel 的 "ROI Filter" 组及 FilterChain 的 RoiFilterStage
  （旧配置走 unknown-key 警告）。RoiPanel 其余部分仅保留偏置预设（与 ROI 矩形无关）。
- **统一 ROI 设置弹窗**（新 `UnifiedRoiDialog`，**模态**，全英文）：内容=现 RoiPanel
  设置区（Enable/Mode/X/Y/W/H/拖拽模式），**数值合法性校验**（W/H>0 且不超 sensor、
  X/Y∈[−1,sensor−W]，非法禁 OK）；**默认 256×144 居中**；文件回放同样可用。
- **自动化语义重做**：名单 `event_to_video`/`isi_analyzer`/`time_surface`/`hough_line`/
  `hough_circle`（移出 XYT）；**删除 roi_user_touched_ 门控，无条件自动开**；
  启用时保存当前 ROI 状态→强制中心 256×144，**禁用时恢复启用前状态**。
  （e2v 注意：旧 AGENTS.md 约束 128×128 → 新默认 256×144+1/4 降采样=工作 128×72，
  需实测 ONNX 动态维度重建正常，同步更新 AGENTS.md。）
- **ConfigManager/拖拽改道**：`apply_roi` 改调 `set_unified_roi`、`capture_roi` 改读
  缓存；`roi_dragged` 经 `set_unified_roi`（不再直连 facility）。
- **ISI 输出**：恒定输出 512×256 图表（删除 resize-to-工作分辨率分支），小窗
  letterbox 缩放。

#### D4 提交拆分（一次一问题，每步编译+ctest 全绿）

1. R1 Overlay 算法不弹 AlgoWindow + status 文本进侧栏只读标签 + 删 zoom 路由
   （Standalone/Passive 窗口保留）
2. R3 删 hough aux 显示（后端生成 + aux 路由 + 过时注释）
3. R2 overlay 坐标回移（OverlayStrategy + Replace 核查）
4. P6 ISI 恒定 512×256
5. 统一 ROI 状态源扩展（mode/信号/文件 RONI/ConfigManager/拖拽改道/黄框单驱动）
6. GUI 合并（两页面勾选框+设置按钮、模态弹窗含校验、删 ROI Filter）
7. 自动化语义重做（名单±、删门控、256×144、保存/恢复前态）

### Phase 3：E2VID 线程化 —— **复现驱动，可整体跳过**

- main 代码证据（已核实）：`algo_bridge.cpp:178-181` `pull_result()` 持 `AlgoInstance::mutex_`，
  `analytics_backends.cpp:297` 在锁内跑 `algo_->get_frame()`（ONNX 推理数十~数百 ms），
  SDK 数据线程的 `push_events` 抢同一把锁。beta `cb451ee` commit message 记录的现象：
  "live 开 E2VID 即 UI 冻结 + 采集背压"。
- **但用户实测体验是原版 E2VID 无明显问题**（可能因为 128×128 ROI 轻量模型推理够快、
  或主要用文件回放）。按 §0 协议：先编译旧版，live 相机开 E2VID 观察 UI 是否卡顿/冻结——
  **复现不了就整体跳过本 Phase，E2VID 代码一行不动**（decay_tau_ms 保留现状，不删不改）；
- 仅当复现确认后才取 `cb451ee`+`52a7ee9`+`ab0cc22`(e2v 部分)+`45c2542` 整体，
  并真机验证：live 重影、背压阻塞、reset 黑帧、pause-resume；
  decay_tau_ms 默认 500 的开箱行为变更届时需用户确认。

### Phase 4：标定重新设计（新代码，不采用 beta 的闪烁棋盘方案）

按用户决策实现，复用 beta 已验证的工程资产（tap 的 DirectConnection+mutex 线程模型、
worker 线程化、导出 mkpath、内角点约定修复、E2E 测试思路）：

- **图案**：非对称圆点阵，黑底白点，**不闪烁**（静态显示；OpenCV `CALIB_CB_ASYMMETRIC_GRID`）；
- **极性**：**忽略事件极性**，ON/OFF 事件同等累加；
- **界面**：重新设计的用户友好 GUI（符合当前体系）：点阵显示区与相机实时输出**并排同窗**，
  屏幕上有小字提示"**按空格键捕捉**"；
- **抓拍**：用户按**空格**主动抓拍——取触发时刻前 **500µs 窗口**内的事件累加成帧
  （忽略极性），跑 `findCirclesGrid`；检测失败/覆盖率不足/与已有帧视角重复则拒绝并提示；
  **抓拍帧不降采样**（beta 的 1/4 分辨率检测不采用）。
  注意：静态图案下事件主要来自用户手持微动与屏幕刷新，500µs 窗口事件量可能偏少——
  若实测检测率不足，按 §0 协议调整窗口长度后再 amend；
- **修复 intrinsic.cpp AsymmetricCircles 物点公式**（§2.3-14，新设计使其从潜伏变为必经之路）；
- 检测与 calibrateCamera 在 worker 线程；导出自动建目录；
- **新代码必须吸收的旧 BUG 遗留动作**（§8.2，否则旧 bug 在新代码里复活）：
  ① tap `attach()` 必须带 UniqueConnection/先 disconnect（main `calibration_event_tap.cpp:28-40`
  重复 connect，beta 的修复在我们不取的 `c375ecc` 里，新 tap 代码要自己带上）；
  ② 屏幕跟踪用 `QPointer<QScreen>`（main 的 `attached_screen_` 裸指针热拔悬垂）；
  ③ 点阵物理间距 mm 由用户直接输入，**不走** `physicalDotsPerInch()` DPI 推算（X11 上不可靠）；
  ④ 热像素在抓拍帧上打孔的兜底：若实测抓拍拒绝率过高，加一行 `cv::medianBlur`
  （按 §0 协议实测后再定）；
- 建议拆分为：①图案显示窗口+并排 GUI ②事件 tap+空格抓拍判定 ③标定计算+导出，各一提交一测。
- **实施状态**：三个子提交全部完成，323 项 ctest 全绿（含 `test_intrinsic` 5 项 + `test_raw_algos`）：
  - ①图案显示窗口+并排 GUI（`53a3d72`）——CircleGridDisplay 静态非对称圆点阵（黑底白点、不闪烁）
    + 点阵/相机实时输出并排向导 + 空格捕捉提示；
  - ②事件 tap+空格抓拍判定（`175360d`）——CalibrationEventTap（`Qt::UniqueConnection`）+
    500µs 窗口空格抓拍（忽略极性、不降采样）+ CalibrationWorker 异步 `findCirclesGrid` +
    覆盖率/重复视角剔除（`detect_only`/`accept`/`is_duplicate_pose`）；
  - ③标定计算+导出（`d19f6f4`）——worker 线程 `cv::calibrateCamera`（bundle adjustment 不阻塞 GUI）
    + auto-mkdir YAML 导出（键名 `image_width/height`/`camera_matrix`/`distortion_coefficients`/`rms`
    对齐 `load_intrinsics_yml`，与 Preprocessor 去畸变默认路径 `~/Documents/EBplus/calibration/intrinsic.yml`
    一致）+ `teardown_worker` 显式释放 worker 防泄漏（`deleteLater` 在 `quit()+wait()` 后不再触发）。
  - §8.2 旧 BUG 遗留动作全部吸收：tap UniqueConnection ✓、`QPointer<QScreen>` ✓、
    mm 手输不走 DPI ✓；AsymmetricCircles 物点公式已修（`test_intrinsic.AsymmetricObjectGridFormula` 验证
    `x=(2c+(r&1))·s, y=r·s`）。④ `cv::medianBlur` 兜底按 §0 协议留待现场实测决定。
  - **Phase 4 完成。** 注意：500µs 抓拍窗口事件量是否足够，仍需现场实测（§0 协议，必要时
    调整窗口长度后 amend）。

### Phase 4 Debug：实测 UI/UX 回归修复（标定向导现场实测）

> 触发：Phase 4 三个子提交（`53a3d72`/`175360d`/`d19f6f4`）落地、323 项 ctest 全绿后，现场实测
> 标定向导暴露 9 个 UI/UX 问题。以下根因全部代码核实（`gui/calibration/` + `gui/main_window.cpp`）。
> **用户约束**：① capture 必须是"抓拍→判定"串行，不实时判定每一帧（计算代价太大）；② 判定完
> 上一帧的取舍后才允许下一次 capture，并自动计数有效帧，否则连续 capture 会卡死；③ 点阵尽量大、
> 横向排列，参数设置/相机画面/已抓拍画面三者同行，点阵在它们下方满宽放大展示。

#### 逐项根因

**D1 布局混乱 + 弹窗无法有效放大（对应实测 1、4）**
- 现状：`build_ui()`（`calibration_wizard.cpp:335-438`）外层 QVBoxLayout = [QFormLayout 参数表] +
  [QHBoxLayout `pattern_` | `camera_view_` 各 stretch 1，整体 stretch 2] + hint + 按钮行 + progress +
  [`preview_area_` stretch 1] + status。点阵与相机各占一半宽度、约 1/4 高度 → 点阵偏小；参数表独占
  顶部整行破坏视觉分组；各模块间距/边距未调，整体观感"混乱"。
- 弹窗仅 `setMinimumSize(900,560)`、无 `setFixedSize`，理论上可拖拽放大，但布局未把点阵作为主元素
  （点阵与相机平分、且只占中段）→ 放大后点阵增长有限，用户感知"无法放大"。

**D2 点阵应横向排列且更大（对应实测 2、6）**
- 现状：默认 `kDefaultCols=4`/`kDefaultRows=11`（`calibration_wizard.cpp:47-48`），非对称网格
  footprint = (2·4−1)×(11−1) = 7×10 cell → **纵向高瘦**。`CircleGridDisplay::recompute_layout()`
  （`circle_grid_display.cpp:40-59`）按 widget 取最大 spacing，但 11 行决定它纵向占优。
- 用户要求：点阵**横向**（landscape）、尽量大，置于参数/相机/已抓拍三者下方满宽放大展示。

**D3 点阵右侧始终 "No camera connected"（对应实测 3）**
- 根因：`on_camera_tick()`（`calibration_wizard.cpp:146-157`）在 `display_->current_frame()` 返回空
  QImage 时把标签设为 "No camera connected"。但 `EventDisplayWidget::current_frame()`
  （`event_display_widget.cpp:208-210`）直接返回 `frame_`，为空的条件包括：① 无相机连接；
  ② 相机已连接但未 `start()`（`camera_.start()` 在 `main_window.cpp:406` 由 Start 按钮触发，向导自身
  不调用）；③ 相机已 start 但首帧尚未渲染（启动竞态）。**消息把"无帧"与"无相机"混为一谈，误导用户。**
- 向导不验证/不确保相机处于 running 态：`show_intrinsic()`（`:117-129`）仅 `set_cd_broadcast(true)`
  （`if is_connected()`），不 `start()`。aim-view 复用主显示 `current_frame()`——主显示流水线未跑则恒空。

**D4 "square size" 参数含义不清（对应实测 5）**
- 现状：标签 "Square size"（`calibration_wizard.cpp:365`）是棋盘术语；圆点阵无"方格"。tooltip 解释
  为物理间距 mm，但标签本身误导。该值实为**相邻圆心物理间距（mm）**，用于 `AsymmetricCircles` 物点
  坐标的真实尺度（`intrinsic.cpp` make_object_grid），是标定的必要输入。

**D5 capture 不起作用（对应实测 7）**
- 根因（与 D3 同源）：`on_capture_pressed()`（`:159-185`）需要 ① 按钮启用（`enable_capture` 在
  `set_camera` 时按 `is_connected()` 决定）；② 相机 running（CD 回调 `camera_controller.cpp:373` 仅在
  camera running 时触发，`:396-398` 在 `cd_broadcast_` 为真时 emit `cd_events_ready`）；③ `cd_broadcast`
  开（`show_intrinsic` 设置）。相机未 start → 无 CD 事件 → `tap_.drain_last_window` 返回 0 → 状态栏
  "No events in the last 500 µs"；相机未连接 → 按钮禁用 → 点击无反应。向导对"相机须先 start"无任何提示。

**D6 流程须为 capture→判定，非实时每帧（对应实测 8）**
- 现状**已满足**：检测只在 `CalibrationWorker::process_frame()`（`calibration_worker.cpp:46-99`）由
  `submit_frame` 触发（即用户按 Space 后），`camera_timer_`（30 Hz）仅轮询 aim-view 帧用于显示、
  **不做检测**。本项为需求复述，修复须保持"仅抓拍时判定"，不引入逐帧检测。

**D7 串行 capture：判定完成后才允许下一次 + 自动计数（对应实测 9）**
- 现状：`capture_in_flight_`（`calibration_wizard.h:128`）在 `on_capture_pressed` 置真、在
  `on_frame_accepted`/`on_frame_rejected`/`on_capture_complete` 复位，**逻辑上**已阻止重入（`:160` 早退）。
  但**按钮在 in-flight 期间保持启用**，连续点击被静默忽略 → 用户感知"卡死/无效"。
- 自动计数已由 `progress_`（`on_frame_accepted` setValue(accepted)）实现，但按钮启用态未与 in-flight
  绑定，串行性对用户不可见。

#### 修复方案（用户已逐项确认）

**布局重做（D1+D2，对应实测 1/2/4/6）**——外层改为：
- **顶行 QHBoxLayout**（三列**等宽**）：[参数表 QFormLayout] | [相机 aim-view] | [已抓拍预览]，三者同行；
  参数表含 Grid(circles)/Circle spacing/Target frames。
- **下方点阵区**（`CircleGridDisplay`，stretch 主导、满宽）：尽可能大、横向；默认网格改为 **8×5**
  （cols=8, rows=5，40 点，footprint 15×4 cell，landscape，conditioning 良好）。
- **底部薄行**：hint + [Capture][Reset][Export] + progress + status。
- 弹窗保持可自由缩放（仅 minimumSize，无 maximumSize），点阵区 sizePolicy=Expanding + stretch 主导，
  随窗口放大而放大。

**D3+D5 相机态提示与前置校验**：
- aim-view 标签按真实相机态分三态显示：`!is_connected()` → "No camera connected"；
  `is_connected() && !is_running()` → "Camera connected — press Start to stream"；
  `is_running() && frame.isNull()` → "Waiting for first frame…"（启动竞态）。
- `on_camera_tick()`/`show_intrinsic()` 以 `camera_->is_connected()`/`is_running()` 为判据，不再用
  `current_frame().isNull()` 兜底为"无相机"。
- 向导**不自动** `start()`（避免改变用户相机运行态）。用户按 Space/Capture 时若相机未 running，**弹模态
  对话框**提示（全英文）："Camera is not running. Please start the camera in the main window first."
  （取代当前静默的状态栏文案），让用户明确知道须先在主窗口 Start。

**D4 参数改名 + 路径核验（已核验：参数确实到达算法，无 BUG）**：
- **核验结论**：`square_mm_` 数据流已逐级核实——`square_mm_` spinbox `valueChanged` → `on_config_changed`
  → `configure_worker()` emit `configure_requested(cols,rows,square_mm,target)` →（跨线程 queued）
  `CalibrationWorker::configure` → `intrinsic_->set_pattern(AsymmetricCircles, cols, rows, square_size_mm)`
  （`intrinsic.cpp:40` 存 `square_size_mm_`）→ `make_object_grid()`（`intrinsic.cpp:53-54` 用
  `(2c+(r&1))·square_size_mm_, r·square_size_mm_`）→ `run()` 喂 `cv::calibrateCamera`。**参数正确到达算法。**
- 用户感知"不起作用"的真因：① 该值**不影响屏幕点阵**（`CircleGridDisplay::set_square_size_mm` 仅存储、
  不改像素布局——按设计不走 DPI，像素间距由 widget 自适应）；② **不影响检测**（`findCirclesGrid` 不读它）；
  ③ 仅影响标定结果的 K 焦距尺度（mm 单位），而当前 capture 坏（D5）→ 跑不出标定结果 → 无从观察。
  D5 修好后该参数的效果即在校准结果中可见。
- 改名："Square size" → "Circle spacing"（suffix " mm"），tooltip 改述"相邻圆心物理间距（mm）。用直尺
  量屏幕上相邻圆心的间距后填入；此值仅用于标定的真实尺度，不影响屏幕点阵大小（不走屏幕 DPI）"。

**D6 aim-view 轮询——不改（澄清）**：
- 我的理解：`camera_timer_`（30 Hz）**仅**轮询 `display_->current_frame()` 缩放显示到 aim-view，**不做任何
  检测**（检测只在 `process_frame` 由 `submit_frame` 触发，即用户按 Space 后）。它是纯显示、轻量
  （`QImage` 隐式共享，轮询取的是 refcount 拷贝），与 FocusAssistant 一致。**无理由改动**——之前把它列为
  "待确认项"系过度谨慎，现澄清：保持 30 Hz 不变。

**D7 按钮串行化**：`on_capture_pressed` 置 `capture_in_flight_=true` 时**同时**
`capture_btn_->setEnabled(false)` 并 `set_status("Detecting…")`；`on_frame_accepted`/`on_frame_rejected`
复位 in-flight 后，若 `!capture_done_` 则按相机态重新 `enable_capture(...)`。Space 键路径同理（`keyPressEvent`
已调 `on_capture_pressed`，靠 `capture_in_flight_` 拦截重入）。自动计数保持 `progress_` 现状。

**内存优化（用户要求：避免多 buffer，复用主显示）**：
- **aim-view（缩小相机预览）**：已复用主显示帧——`on_camera_tick` 取 `display_->current_frame()`
  （`EventDisplayWidget::frame_` 的 `QImage` 隐式共享拷贝，不深拷像素），缩放后 `setPixmap`（缩放结果为
  瞬态 `QPixmap`，下一 tick 覆盖）。**无独立持久 buffer**，即用户建议的"用主显示画面缩小显示"。不改。
- **capture（取事件）**：tap（`CalibrationEventTap`）**必需**——已核实 `FramePipeline` live 模式不保留原始
  事件（`add_events` 转发给 SDK `CDFrameGenerator` 累积后丢弃，无可访问的滚动事件 buffer），主显示帧是
  ~33 ms 累积渲染（带颜色/decay），非干净 500µs 二值窗，`findCirclesGrid` 不能用。tap 是拿到最近 500µs
  原始事件、渲染干净二值帧的唯一途径。
  - **优化**：`kMaxBufferEvents` 从 2M（32 MB）降至 256K（4 MB）——256K 事件可覆盖 500µs @ 512 Mev/s
    （远超任何真实传感器），capture 间隔内 tail-trim 仅留最近 ~6 ms，对 500µs 切片绰绰有余。**内存降 8×**。
- **已抓拍预览**（`preview_label_`）：仅持一张 annotated `QImage`（最近一次 accept），小且瞬态，非关注点。
- 结论：持久 buffer 实为两个（主显示 `frame_` + tap `buffer_`），tap 缩至 4 MB；aim-view/预览均复用或瞬态。
  无第三个独立 buffer。

#### 实施状态（D1–D7 全部落地，323 项 ctest 全绿）
- **D1+D2**：`build_ui` 重写为顶行三等宽列（参数 | aim-view | 预览）+ 下方满宽大点阵
  （`QSizePolicy::Expanding` + stretch 1，`min 400×200`）+ 底部控件行（按钮 + progress）+ status；
  默认网格 8×5（`kDefaultCols=8`/`kDefaultRows=5`，40 点 landscape）。
- **D3+D5**：`on_camera_tick` 三态标签（以 `is_connected()`/`is_running()` 为判据，不再用
  `current_frame().isNull()` 兜底为"无相机"）；`on_capture_pressed` 在未连接/未 running 时弹模态
  `QMessageBox::information`（全英文）；去掉 `!isEnabled()` 守卫，使 Space 在未连接时也能弹窗提示。
- **D4**：参数改名 "Circle spacing (mm)" + 英文 tooltip（阐明"仅真实尺度，不影响屏幕点阵，不走 DPI"）；
  已核验参数确实到达算法（`square_mm_` → `configure_requested` → `set_pattern` → `make_object_grid`
  → `calibrateCamera`），无 BUG——用户感知"不起作用"实因不影响屏幕点阵/检测，且 capture 曾坏（D5 已修）。
- **D6**：aim-view 30 Hz 轮询保持不变（纯显示、不做检测），澄清后无需改动。
- **D7**：`on_capture_pressed` 置 in-flight 时 `capture_btn_->setEnabled(false)`；`on_frame_accepted`/
  `on_frame_rejected` 判定后按相机态 `enable_capture(...)` 复位（`on_capture_complete` 达目标后永久禁用）。
- **内存**：`kMaxBufferEvents` 2M→256K（32 MB→4 MB，已确认 USB3 极端突发 300 Mev/s 下 500µs=150K<256K，tail 874µs>500µs）；
  aim-view 复用主显示帧（QImage 隐式共享）；持久 buffer 仅两个（主显示 `frame_` + tap `buffer_`）。
- **提交**：按用户指示，4 个旧 Phase 4 提交（`53a3d72`/`175360d`/`d19f6f4`/`a52f74d`）+ 本轮 D1–D7 +
  本节文档合为**单一 Phase 4 提交**（`git reset --soft origin/main` + 单次 commit，不用 `rebase -i`），
  未推送，待用户确认后作为 2.0.0 推送。

#### D8 最大化按钮卡顿（仅按钮卡、拖拽放大不卡）—— 性能问题第三轮根因修复

> 触发：D1–D7 落地后用户报告标定向导运行卡顿；经两轮性能优化（事件 tap batch-ring、点阵预渲染
> QPixmap）仍卡；第三轮把 `showMaximized()` 换成 `setGeometry(availableGeometry())` 仍卡。用户关键
> 新发现：**拖拽窗口边缘到满屏 = 流畅；只有点右上角最大化按钮才卡。**

**现象与根因**
- 两种放大方式窗口内容、尺寸相同，唯一差异是**窗口状态**：拖拽放大保持 `Qt::WindowNoState`（普通态）；
  最大化按钮则由窗口管理器（Mutter/KWin）把窗口置为 `Qt::WindowMaximized`。
- X11 合成器对 maximized 窗口走**与普通窗口不同的帧同步/scanout 路径**（Mutter 按自身帧时钟同步上屏）。
  该路径与本应用渲染管线（主显示 `EventDisplayWidget` 为 `QOpenGLWidget` 且相机持续推流 + 弹窗内
  `CalibrationCameraView` 30 Hz `update()` 重绘）配合不良 → `update()` 被串行节流 → 卡顿。普通态走
  常规合成路径 → 流畅（即"拖拽放大不卡"的成因）。

**前三轮为何失败**
- 第 1、2 轮改渲染侧（`CalibrationCameraView` 改 `QOpenGLWidget`、关主显示 `setUpdatesEnabled`）——
  方向错：卡顿源于**窗口状态**而非渲染开销，故全无效；第 3 轮清理时已全部回退。
- 第 3 轮 `show_intrinsic()` 用 `setGeometry()` 替代 `showMaximized()` —— 只影响**初始弹出**，对用户
  **之后点击最大化按钮**无作用（按钮仍进入 `Qt::WindowMaximized`），故"只放大不卡"的初始场景看似
  好转，按钮一按仍卡。

**修复（根因级）**
- `calibration_wizard.h/cpp`：override `changeEvent(QEvent*)`。拦截 `QEvent::WindowStateChange` 中任何
  进入 `Qt::WindowMaximized` 的转变（最大化按钮 / 标题栏双击 / WM 快捷键均覆盖），立即撤销 maximized
  态并 `setGeometry(availableGeometry())` 满屏。窗口**看起来最大化**，但始终停留在与"拖拽放大"相同的
  普通合成路径 → 流畅。
- 用 `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` 延迟执行，避免在 `changeEvent` 内重入
  `setWindowState`；自身 `setWindowState` 触发的第二次 `changeEvent` 中窗口已非 maximized，守卫落空，
  无递归循环。
- `show_intrinsic()` 的 `setGeometry()` 初始满屏**保留**（独立有益：弹出即满屏且不进入 maximized 态），
  注释更新为指向 `changeEvent()` 统一说明。

**撤销评估（用户要求：撤前三轮画蛇添足、留确定有益）**
- 经第 3 轮清理，`calibration_wizard.*` 已无 OpenGL/`setUpdatesEnabled`/`cacheKey` 残留
  （`grep` 核实）；`kCameraPollMs=33`（30 Hz）已回退为原值，未私自改 10 Hz。`event_display_widget.cpp`、
  `main_window.cpp` 同样无残留。**当前态已干净，无需再撤。**
- 保留的有益改动：事件 tap batch-ring（SDK 线程 O(N)→O(1)/批）、点阵预渲染 QPixmap、`Qt::Window`
  + `WindowMinMaxButtonsHint`（使最大化按钮可见可用）、`setGeometry()` 初始满屏、6×6 默认网格、
  Circle spacing 英文 tooltip、三态相机提示、capture 串行化、worker 线程 `calibrateCamera`。
- 本轮新增仅 `changeEvent` 一处（约 15 行 + 注释），无画蛇添足。

**实施状态**：编译通过；`ctest` 322/323 通过，唯一失败 `loop_flip` 为时序敏感 flaky 测试，单独重跑通过
（与本改动无关，diff 仅 `calibration_wizard.{h,cpp}`）。amend 入 `4b4d9ba`。

#### D9 预览卡死在 "No camera connected"（主显示相机正常）—— 缺 showEvent

> 触发：D8 落地后用户报告：标定向导预览只显示 "No camera connected"，但主显示相机一切正常。

**根因**
- `on_camera_tick` 的 "No camera connected" 仅来自 `!camera_->is_connected()` 分支；
  `is_connected()` = `bool(camera_)`，只要连了实时相机或文件就为真。主显示正常推流 ⇒
  `camera_` 非空 ⇒ `is_connected()` 真 ⇒ 若 `on_camera_tick` 在跑，必显示帧而非该文案。
- 故 `on_camera_tick` **没在跑**（定时器已停）。定时器只在 `hideEvent` 里 `stop()`，而向导
  **没有 `showEvent`** 重新启动它。任一 hide→show 循环（最小化→还原 / 工作区切换 / WM 状态切换）
  后定时器永久停止，预览冻结在停止前最后一条消息上——若停止时相机尚未启动，就卡在
  "No camera connected"，随后在主窗口启动相机也不更新（向导不再轮询）。
- 这是 D1 给弹窗加 `Qt::Window` + `WindowMinMaxButtonsHint`（启用最小化/最大化交互）后引入的
  潜在缺陷，D8 测试最大化时频繁窗口交互使其暴露。

**修复**：`calibration_wizard.h/cpp` 新增 `showEvent(QShowEvent*)` override，镜像 `hideEvent`——
窗口再次可见时重启 `camera_timer_` 并（相机已连接时）`tap_.clear()` + `set_cd_broadcast(true)`。
`QTimer::start()` 对已运行的定时器无副作用（仅重置间隔）；无相机连接时下一 tick 自然报状态。
`show_intrinsic()` 仍保留显式 `start()`（覆盖"已可见时再次点菜单"的场景，此时 `show()` 不触发 showEvent）。

**实施状态**：编译通过；`ctest` 320/320 通过（排除 `loop_flip`/`raw_e2v`/`playback_e2v` 三个慢/flaky，
前两者此前已验证）。amend 入 Phase 4 提交。

### Phase 5：调焦工具（新代码，替换锐度计）

- 屏幕绘制**缓慢旋转的 Siemens Star**（高分辨率、居中），用户旋转镜头对焦环目视调焦，
  参考 inivation 官方做法；**无需任何锐度/DFT 计算**；
- **预绘制优化（采纳用户建议）**：星图按旋转对称周期预渲染一组 QPixmap 相位帧，
  运行时只做 `drawPixmap` + 递增相位索引，比每帧 QPainter 画扇区更省且天然无撕裂/闪烁；
- **移除现有 sharpness 计算**（sharpness_dialog 的数据源——渲染后显示帧——已被证实方向不可用）；
- 界面可与标定共用并排思路：星图 + 相机输出同窗。
- ①星图窗口 ②移除旧锐度工具，各一提交一测。

### Phase 6：dv-processing 保守移植（三项，各一提交一测）

1. **KNoise 滤波模式**（dv `noise/k_noise_filter.hpp` → `algo/cv/noise_filter.h` 新增
   `Mode::KNoise`）：W+H 行列单元（640×480 约 18KB vs BAF 的 2.4MB），硬极性匹配；
   不共享 `last_any_` 面，自带单元数组；接入现有 GUI 滤波参数体系；
   dv 默认值按 DAVIS 调的，须用 raw 集成测试重新标定后才能定默认。
2. **eArc/Arc\* 角点检测**（dv `features/arc_corner_detector.hpp` → `corner_detector.h` 第四模式）：
   复用现有双极性时间面；输出连续 response 写入 `Corner::strength`；**必须加 is_recent
   前置门控**（dv 逐事件无门控，成本 ~10× EndStopped）；建议半径 3/4 小模板降本。
3. **TimeSurface 指数 decay**（dv `core/frame/accumulator.hpp` EXPONENTIAL 分支 →
   `time_surface.h` 增加 `Decay{Linear, Exponential}` + `tau_us`，display backend 透传）：
   线性 decay 窗口尾部硬切到 0，指数过渡自然（dv 默认模式）。
- 每项：合成单测 + raw 集成测试 + GUI 参数透传验证；
- 顺带补注册三个"backend 已支持仅缺注册"的既有参数（§8.2，随对应主题提交）：
  time_surface `refresh_rate_hz`（backend 硬编码 30）、trigger `t1_us`、
  sensor_self_test `rep_averaging_samples`。其余算法公开参数 GUI 不可达项（§五-A4）
  属增强，有理由 defer。

### 明确不做

- develop 的 mega-commit（de0e607）形态合并；develop 整支合并；
- **decay_tau_ms 删除**（develop 的误判；该参数是 e2v 帧间衰减，防残影，AGENTS.md 硬性要求
  GUI 暴露 [0,5000] 默认 500——保留，见 §4.5）；
- **release_resources 机制**（develop 的禁用即卸载 ONNX 生命周期，导致 pause/resume 付
  300–500ms 重载，develop 自己已回退调用——不引入，保持 main 的"禁用保持加载"行为，见 §4.7）；
- beta 的 DFT 对焦工具、闪烁棋盘标定方案（被新设计取代）；
- dv 的其他功能（频率滤波、滤波链、**EdgeMap**（边缘图显示窗口，见 §4.7）、mean-shift
  等——避免过度冗余）。

---

## 7. 验证与提交规范

- 每个提交：单一问题、清晰分离关注点（禁止 mega-commit）；
- **验证协议（§0）**：旧版编译复现 → 用户确认问题 → 新版编译 → 用户核验 →
  通过则保留提交（推送由用户决定），不通过则继续修改并 **amend 本地提交**；
- 算法变更：合成单测 + raw 集成测试双通过；
- 涉及线程模型的组（Phase 2/3/4）需真机 live 场景验证。

---

## 8. 旧审计（a94b28a，585 行版）覆盖率核对

对旧审计文档全部约 90 条可操作发现逐条映射到本计划（映射过程抽查了关键 beta 提交
message/diff 验证修复确实落入对应组）。**总账：严重/高级别条目 100% 覆盖**；
直接覆盖 ~60 条 + 新设计取代 ~15 条 + 有意决策/注释定档 ~10 条 + 实质遗漏 8 条
（已按 §8.2 补入计划）+ 3 条死代码决策项 + 2 条条件性覆盖（已解耦/加核验项）。

### 8.1 取代关系确认（Phase 4/5 使旧审计对应章节整体失效）

- Phase 4（静态圆点阵+空格抓拍）取代 §六-6.4 与 §九-9.1/9.2/9.4 全部：闪烁机制消失
  （9.2-B/§六-B5）、棋盘极性论证前提消失（9.2-A）、MSE 查重与抓拍相位问题消失
  （9.2-D/F）、检测/worker 架构重写（9.2-C、§六-B3/B4）——取代成立；
  但 4 个遗留动作必须在新代码中吸收（已写入 Phase 4 清单：tap UniqueConnection、
  QPointer、点间距手输、medianBlur 兜底）。
- Phase 5（Siemens Star）取代 §九-9.3/9.4 锐度部分全部（R1-R4 失败分析的对象被
  "完全移除锐度计算"删除，S1-S6 路线作废）——取代成立。

### 8.2 实质遗漏及处置（已全部补入计划）

| 遗漏条目 | 级别 | 处置 |
|---|---|---|
| §五-F3 undistort 预处理链三问题（自赋值 UB/异常被吞/YAML 静默失败） | 中 | Phase 1-11a，先于 Phase 4 |
| §五-H3 算法异常静默死亡（两处 `catch(...){}`） | 低 | Phase 1-11b |
| §五-G5 background_mask Standalone 死分支 + mat_to_qimage 静默空图 | 低 | Phase 1-11c |
| §5.9-疑点4 Config 加载后面板控件不刷新 | 中低 | Phase 1-11d |
| §一-1.3 Refractory 非单调事件放行（与 jAER 不符） | 中低 | Phase 1-11e |
| §一-1.3 DWF 单窗模式窗口减半 | 中 | Phase 1-11f 注释定档，行为对齐 defer |
| §一-2.2 LocalPlanes 缺 jAER per-pixel refractory（输出密度差异） | 中 | Phase 1-11f 注释定档，移植 defer |
| §五-A4 算法公开参数 GUI 调不到 | 中 | 增强类 defer；其中 3 个"仅缺注册"参数随 Phase 6 顺带补 |
| §三-S4/S7/S8 死代码决策项（PerformanceMeter/诊断程序未注册 CTest/注释引用无声明） | 低 | Phase 1-9 同组决策、1-11f 顺手改注释 |
| §五-H1 E2VID 模型加载失败静默降级 | 中 | 从 Phase 3 解耦 → Phase 1-12 |
| §5.9-疑点3 ultra_slow_motion 输出未来时间戳 | 低 | 随 Phase 2.5 配套清理整体删除该算法，疑点消失 |

### 8.3 有理由 defer 的验证/环境项

§六-M2 无单实例机制（低）；§六-6.8-2 `Camera::stop()` join 语义（SDK 层，仓库内不可修）；
§六-6.8-5 FileFrameGenerator 假设严格时间排序（疑点未证实）。三条均保持观察、不进计划。

### 8.4 937 行版（develop tip 扩充版）附注

937 版新增的 §十一~§十四 是 develop/beta 自身 rework 的回归审计与过程日志，其结论
（8 处自引入回归、flood 30M 误杀、drop-OLDEST 重影、decay_tau_ms 误删等）已全部反映在
本计划 §3.2/§4.4/§4.5 的取舍中，**无独立于 585 版的新可操作发现**；唯一实质技术点
§11.2-H（hough accumulate_only 显式 cur_t）已随 `7bcf316`+`70e812a` 带入。
