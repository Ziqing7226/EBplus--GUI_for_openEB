# 2.0.0 系统审计与开发计划（基于 main @ caee2c0）

> 本文档替代被撤销的 a94b28a（`doc/systematic_audit.md`），是基于 main 最新状态重新完成的
> 系统审计 + develop / develop-beta 分支取舍分析 + 2.0.0 开发计划。
> 文档目录自此采用 `devlog/`（doc → devlog 改名已采纳，引用更新随 2.0.0 基建提交完成）。

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
| filtered_events 回显 | **采纳**（main 上滤波类算法启用后主显示无变化，经确认为缺陷，见 §6 Phase 2.5） |

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
| filtered_events 回显 | `c23e0e0` | 滤波算法输出回注主显示（main 上启用滤波无视觉效果，已核实） | ✅ 已采纳 |

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

### Phase 2.5：filtered_events 回显主显示（`c23e0e0`）

- main 现状（已核实代码）：hot_pixel_filter、ultra_slow_motion 等滤波类算法启用后主显示
  无变化——输出没有任何消费者，属缺陷而非设计；
- 按 §0 协议先在旧版验证该现象（启用滤波算法观察主显示），确认后取 `c23e0e0`；
- 该改动触及逐帧主渲染路径（加了三相重渲染分支，带 50k 事件上限 + try/catch），
  核验时重点测：空流行为与旧版逐帧等价、高事件率下不掉帧；
- 核验清单附加项（§8.2）：启用 ultra_slow_motion 观察回显——该算法输出时间戳指向未来，
  回显后首次有了消费者，需确认其影响可接受。

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
| §5.9-疑点3 ultra_slow_motion 输出未来时间戳 | 低 | Phase 2.5 核验清单附加项 |

### 8.3 有理由 defer 的验证/环境项

§六-M2 无单实例机制（低）；§六-6.8-2 `Camera::stop()` join 语义（SDK 层，仓库内不可修）；
§六-6.8-5 FileFrameGenerator 假设严格时间排序（疑点未证实）。三条均保持观察、不进计划。

### 8.4 937 行版（develop tip 扩充版）附注

937 版新增的 §十一~§十四 是 develop/beta 自身 rework 的回归审计与过程日志，其结论
（8 处自引入回归、flood 30M 误杀、drop-OLDEST 重影、decay_tau_ms 误删等）已全部反映在
本计划 §3.2/§4.4/§4.5 的取舍中，**无独立于 585 版的新可操作发现**；唯一实质技术点
§11.2-H（hough accumulate_only 显式 cur_t）已随 `7bcf316`+`70e812a` 带入。
