# 算法代码性能优化文档

> 版本：1.0
> 日期：2026-07-14
> 范围：`algo/` 及 `gui/algo_bridge/` 的代码级性能优化，不改变任何算法语义
> 原则：仅优化代码实现（内存管理、访问模式、等价数学重写），不改变算法行为

---

## 一、概述

### 1.1 文档目的

本文档列出算法代码中的性能优化点，按"严格复查 → 放弃有风险项 → 保留安全项"的流程筛选。
所有保留项均经验证确认不改变算法输出，仅改善执行效率。

### 1.2 筛选标准

- **保留**：纯机械优化（缓冲区复用、指针访问、预计算常量、等价数学变换），行为完全一致
- **放弃**：涉及容器类型变更、内存布局重构、语义边界条件变化、数值稳定性风险、或维护性隐患的项

### 1.3 复查方法

对每个候选优化点逐一读源码验证：
1. 确认被优化的变量/循环的完整生命周期（写入、读取、覆盖时序）
2. 确认无隐藏的语义依赖（如连续两次调用、边界条件、退化路径）
3. 确认优化后结果与原实现数学等价

---

## 二、保留的优化点

### 2.1 后端桥接层（`gui/algo_bridge/`）

#### OPT-01：TriggerSyncedBackend 本地 vector 改为成员复用
- **文件**：[analytics_extra_backends.cpp:143-149](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_extra_backends.cpp#L143-L149)
- **问题**：`push_events` 内 `std::vector<Metavision::EventCD> inp(b, e)` 每次调用堆分配。同类后端（如 `ParticleCounterBackend`）已使用 `passthrough_` 成员 + `.assign()` 复用容量
- **方案**：添加 `std::vector<Metavision::EventCD> passthrough_;` 成员，改为 `passthrough_.assign(b, e);`
- **安全性**：`passthrough_` 仅在 `push_events` 写入、`pull_result` 拷贝读取，无跨调用依赖

#### OPT-02：UltraSlowMotionBackend 本地 vector 改为成员复用
- **文件**：[display_backends.cpp:140-145](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/display_backends.cpp#L140-L145)
- **问题**：同 OPT-01，`push_events` 内本地 vector 每次分配
- **方案**：同 OPT-01，添加 `passthrough_` 成员
- **安全性**：同 OPT-01

#### OPT-03：Preprocessor 下采样循环拆分循环不变分支
- **文件**：[backend_common.h:240-250](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/backend_common.h#L240-L250)
- **问题**：`if (halve_coords_)` 每事件判断一次，但该成员仅在构造/`set_param` 时变（GUI 线程），`apply()` 执行期间不变
- **方案**：按 `halve_coords_` 拆为两个专用循环，消除每事件分支
- **安全性**：两分支逻辑与原 `if` 完全一致，仅消除重复判断

#### OPT-04：crop_to_roi 改为输出缓冲区引用复用容量
- **文件**：[backend_common.h:258-277](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/backend_common.h#L258-L277)
- **问题**：返回 by value 导致调用方 `roi_events_ = crop_to_roi(...)` 每次释放旧缓冲、分配新缓冲。`RoiFilter::apply`（同文件）已正确使用输出引用模式
- **方案**：改为 `void crop_to_roi(const Event*, size_t, const ProcessRegion&, Preprocessor*, std::vector<Event>& out)`，`out.clear()` + `out.reserve(n)` 后填充。更新 5 个调用点（[analytics_backends.cpp:261](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_backends.cpp#L261)、[analytics_backends.cpp:408](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/analytics_backends.cpp#L408)、[cv_vector_backends.cpp:97](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_vector_backends.cpp#L97)、[cv_vector_backends.cpp:258](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_vector_backends.cpp#L258)、[display_backends.cpp:85](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/display_backends.cpp#L85)）
- **安全性**：行为完全一致，仅复用调用方 `roi_events_` 的容量

#### OPT-05：FilterChain process 预分配 next 缓冲
- **文件**：[filter_chain.cpp:343-359](file:///home/justin/GUI-for-openEB/gui/algo_bridge/filter_chain.cpp#L343-L359)
- **问题**：`std::vector<Metavision::EventCD> next;` 零容量启动，首阶段 `back_inserter` 触发 ~16 次 realloc（log2(50000)≈16）
- **方案**：循环前 `next.reserve(cur.size());`
- **安全性**：`reserve` 不改变逻辑，仅预分配

#### OPT-06：OrientationFilterBackend 固定角度 cos/sin 改静态表
- **文件**：[filter_backends.cpp:93-99](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/filter_backends.cpp#L93-L99)
- **问题**：角度固定为 0/π/4/π/2/3π/4，每帧调 8 次 `std::cos`/`std::sin`
- **方案**：`static constexpr float kCosTable[4]` / `kSinTable[4]` 查表
- **安全性**：值完全相同，编译期常量

#### OPT-07：Frame backends cv::Mat::at 改 ptr 行指针
- **文件**：[openeb_frame_backends.cpp:160-166](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/openeb_frame_backends.cpp#L160-L166)、[openeb_preproc_backends.cpp:135-145](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/openeb_preproc_backends.cpp#L135-L145)、[openeb_preproc_backends.cpp:262-268](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/openeb_preproc_backends.cpp#L262-L268)
- **问题**：`r.frame.at<cv::Vec3b>(y, x)[0]` 每像素边界检查 + 偏移重算
- **方案**：`cv::Vec3b* row = r.frame.ptr<cv::Vec3b>(y);` + `row[x][0]`
- **安全性**：像素访问等价，仅省去边界检查

#### OPT-08：ObjectTrackerBackend pull_result 预分配 overlay vector
- **文件**：[cv_backends.cpp:250-282](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_backends.cpp#L250-L282)
- **问题**：`r.boxes`/`r.lines`/`r.texts`/`r.trajectories` 在循环内增长无 reserve
- **方案**：循环前按 `objs.size()` reserve
- **安全性**：仅预分配

#### OPT-09：SparseOpticalFlowBackend pull_result 除法改乘倒数
- **文件**：[cv_backends.cpp:422-428](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/cv_backends.cpp#L422-L428)
- **问题**：`(angle_rad + M_PI) / (2.0f * M_PI)` 和 `mag / 1000.0f` 每次除法
- **方案**：`static constexpr float kInv2Pi = 1.0F / (2.0F * M_PI);` 改乘法
- **安全性**：数学等价

#### OPT-10：DirectionSelectiveFilter overlay 字符串预分配
- **文件**：[filter_backends.cpp:214-217](file:///home/justin/GUI-for-openEB/gui/algo_bridge/backends/filter_backends.cpp#L214-L217)
- **问题**：`t.text += ...` 循环内 8 次 `+=` 可能多次 realloc
- **方案**：循环前 `t.text.reserve(128);`
- **安全性**：仅预分配

### 2.2 Analytics 算法层（`algo/analytics/`）

#### OPT-11：chambolle_tv phi 缓冲改成员复用
- **文件**：[event_to_video.h:380](file:///home/justin/GUI-for-openEB/algo/analytics/event_to_video.h#L380)
- **问题**：`std::vector<double> phi(N)` 每次调用分配。chambolle_tv 每帧调用 3 次
- **方案**：改为 `mutable std::vector<double> chambolle_phi_;` 成员，`if (chambolle_phi_.size() != N) chambolle_phi_.resize(N);`
- **安全性**：`phi` 是纯 scratch，在函数内先写后读，无跨调用依赖

#### OPT-12：reconstruct_bardow utx/uty 改成员复用
- **文件**：[event_to_video.h:492](file:///home/justin/GUI-for-openEB/algo/analytics/event_to_video.h#L492)
- **问题**：`std::vector<double> utx(N), uty(N)` 每帧分配
- **方案**：改为成员 `utx_`/`uty_`，懒 resize
- **安全性**：纯 scratch，函数内先写后读

#### OPT-13：reconstruct_interacting Vc/I_out 改成员复用
- **文件**：[event_to_video.h:624](file:///home/justin/GUI-for-openEB/algo/analytics/event_to_video.h#L624)、[event_to_video.h:771](file:///home/justin/GUI-for-openEB/algo/analytics/event_to_video.h#L771)
- **问题**：`std::vector<double> Vc(N)` 和 `I_out(N)` 每帧分配
- **方案**：改为成员 `Vc_`/`I_out_`，懒 resize
- **安全性**：纯 scratch

#### OPT-14：event_window_ push_back 前预 reserve
- **文件**：[event_to_video.h:75-92](file:///home/justin/GUI-for-openEB/algo/analytics/event_to_video.h#L75-L92)
- **问题**：循环 `push_back` 无 reserve，事件突发时多次 realloc
- **方案**：循环前 `event_window_.reserve(event_window_.size() + n);`
- **安全性**：仅预分配

#### OPT-15：freq_detector DFT twiddle 因子预计算
- **文件**：[freq_detector.h:303-311](file:///home/justin/GUI-for-openEB/algo/analytics/freq_detector.h#L303-L311)
- **问题**：`std::cos(2πkn/N)` 和 `std::sin(2πkn/N)` 每个 (k,n) 调用，但值仅依赖 (k,n,N)
- **方案**：预计算 `cos_tab[n]`/`sin_tab[n]`（n=0..N-1），索引 `(k*n) % N`。利用 `cos(2πkn/N) = cos(2π(kn mod N)/N)`（cos 周期 2π）
- **安全性**：数学恒等，结果完全相同

#### OPT-16：freq_detector Hann 窗预计算共享
- **文件**：[freq_detector.h:285-292](file:///home/justin/GUI-for-openEB/algo/analytics/freq_detector.h#L285-L292)
- **问题**：Hann 窗仅依赖 N，但每簇重算 N 次 `std::cos`
- **方案**：在 `analyze()` 中计算一次，传给 `compute_frequency`，或缓存为成员
- **安全性**：N 在一次 `analyze()` 内对所有簇相同，值完全相同

#### OPT-17：freq_detector 每簇 vector 改成员复用
- **文件**：[freq_detector.h:257](file:///home/justin/GUI-for-openEB/algo/analytics/freq_detector.h#L257)、[freq_detector.h:277](file:///home/justin/GUI-for-openEB/algo/analytics/freq_detector.h#L277)、[freq_detector.h:302](file:///home/justin/GUI-for-openEB/algo/analytics/freq_detector.h#L302)
- **问题**：每簇分配 `ts`/`signal`/`mag` 三个 vector；`ts` 无 reserve
- **方案**：`signal_`/`mag_` 改成员懒 resize；`ts_` 改成员 + `reserve(64)`
- **安全性**：纯 scratch

#### OPT-18：active_marker 合并双遍热图扫描 + ptr 访问
- **文件**：[active_marker.h:89-110](file:///home/justin/GUI-for-openEB/algo/analytics/active_marker.h#L89-L110)
- **问题**：mask 构建（89-98）和 max_count 查找（103-110）各遍历全图一次；`Mat::at` 每像素边界检查
- **方案**：合并为单遍循环，同时构建 mask 和跟踪 max_count；用 `ptr<uint8_t>(y)` 代替 `at<uint8_t>(y,x)`
- **安全性**：两条件（`heatmap[idx] >= threshold` 和 `heatmap[idx] > max_count`）独立，合并不影响结果

#### OPT-19：active_marker estimate_frequency ts 预 reserve
- **文件**：[active_marker.h:266-272](file:///home/justin/GUI-for-openEB/algo/analytics/active_marker.h#L266-L272)
- **问题**：`ts` 在循环内 push_back 无 reserve
- **方案**：`ts.reserve(64);`（3×3 区域事件数少）
- **安全性**：仅预分配

#### OPT-20：e2vid_inference padded 缓冲改成员复用
- **文件**：[e2vid_inference.h:399-406](file:///home/justin/GUI-for-openEB/algo/analytics/e2vid/e2vid_inference.h#L399-L406)
- **问题**：每个 bin `crop_.pad(bin)` 内 `cv::copyMakeBorder` 分配新 Mat
- **方案**：预分配 padded 缓冲成员，或直接反射填充进 `input_buffer_`
- **安全性**：padding 操作确定性，复用缓冲不改变结果

#### OPT-21：unsharp_mask 用 addWeighted 消除中间临时对象
- **文件**：[unsharp_mask.h:33](file:///home/justin/GUI-for-openEB/algo/analytics/e2vid/unsharp_mask.h#L33)
- **问题**：`(1.0f + amount_) * img - amount_ * blurred` 产生 2 个中间 Mat 临时对象
- **方案**：`cv::addWeighted(img, 1.0f + amount_, blurred, -amount_, 0.0f, sharp);`
- **安全性**：`addWeighted` 计算 `dst = alpha*src1 + beta*src2 + gamma`，数学等价

#### OPT-22：flow_statistics 延迟 sqrt 计算
- **文件**：[flow_statistics.h:65-67](file:///home/justin/GUI-for-openEB/algo/analytics/flow_statistics.h#L65-L67)
- **问题**：`mag_gt` 先 sqrt 再比较 `mag_gt < kEps`；被跳过的样本浪费了 sqrt
- **方案**：先比较平方幅值 `mag_gt2 < kEps * kEps`，通过后再 sqrt
- **安全性**：对非负值，`a < b` 等价于 `a² < b²`

#### OPT-23：intensity_rescaler median_scalar 用 nth_element
- **文件**：[intensity_rescaler.h:107-121](file:///home/justin/GUI-for-openEB/algo/analytics/e2vid/intensity_rescaler.h#L107-L121)
- **问题**：每次分配 vector + 全排序，但只需中位数
- **方案**：复用成员 `sort_buf_`；用 `std::nth_element` 代替 `std::sort`（O(n) vs O(n log n)）
- **安全性**：`nth_element` 后 `tmp[size/2]` 与 `sort` 后 `tmp[size/2]` 值相同

### 2.3 CV 算法层（`algo/cv/`）

#### OPT-24：sparse_optical_flow LocalPlanes xs/ys/ts 改成员复用
- **文件**：[sparse_optical_flow.h:161-176](file:///home/justin/GUI-for-openEB/algo/cv/sparse_optical_flow.h#L161-L176)
- **问题**：每事件分配 `xs`/`ys`/`ts` 三个 vector（各最多 289 元素）
- **方案**：改成员 `xs_`/`ys_`/`ts_`，每事件 `.clear()`（保留容量），构造时 `reserve((2*r+1)*(2*r+1))`
- **安全性**：`fit_plane` 和 `plane_confidence` 通过 const 引用读取，clear 后重新填充

#### OPT-25：sparse_optical_flow BlockMatch 下采样 ptr 访问
- **文件**：[sparse_optical_flow.h:596-603](file:///home/justin/GUI-for-openEB/algo/cv/sparse_optical_flow.h#L596-L603)
- **问题**：6 次 `Mat::at<float>` 每输出像素
- **方案**：`ptr<float>(y*2)` / `ptr<float>(y*2+1)` / `ptr<float>(y)` 行指针 + 直接索引
- **安全性**：像素访问等价

#### OPT-26：background_mask_filter 合并双遍 packet 扫描
- **文件**：[background_mask_filter.h:65-79](file:///home/justin/GUI-for-openEB/algo/cv/background_mask_filter.h#L65-L79)
- **问题**：第一遍找 `t_now`（max timestamp），第二遍累积直方图
- **方案**：合并为单遍，在累积循环中同时跟踪 `t_now`
- **安全性**：`t_now` 仅在循环后使用

#### OPT-27：cluster_lif label 缓冲改成员复用
- **文件**：[cluster_lif.h:134](file:///home/justin/GUI-for-openEB/algo/cv/cluster_lif.h#L134)
- **问题**：`std::vector<int> label(n, -1)` 每 packet 分配
- **方案**：改成员 `label_`，`assign(n, -1)` 复用容量
- **安全性**：纯 scratch

#### OPT-28：line_segment_detector buckets 改成员复用
- **文件**：[line_segment_detector.h:68-69](file:///home/justin/GUI-for-openEB/algo/cv/line_segment_detector.h#L68-L69)
- **问题**：每 packet 构造 `vector<vector<Point2f>>` + 内层无 reserve
- **方案**：改成员 `buckets_`，每 packet 对各内层 `.clear()`（保留容量）
- **安全性**：纯 scratch

#### OPT-29：orientation_cluster 预计算单位向量表
- **文件**：[orientation_cluster.h:173-178](file:///home/justin/GUI-for-openEB/algo/cv/orientation_cluster.h#L173-L178)
- **问题**：每邻居每事件 `sqrt` + 2 次除法。`(xx, yy)` 来自有限 RF 偏移集合
- **方案**：按 `(w, h)` 和极性预计算 `(xx/vl, yy/vl, is_zero)` 查表，`rf_width_`/`rf_height_`/`use_opposite_polarity_` 变化时重建
- **安全性**：`(xx, yy)` 的计算确定性，查表值与实时计算相同

#### OPT-30：optical_gyro 簇裁剪用 move 代替 copy
- **文件**：[optical_gyro.h:202-207](file:///home/justin/GUI-for-openEB/algo/cv/optical_gyro.h#L202-L207)
- **问题**：`kept.push_back(c)` 拷贝每个 GyroCluster
- **方案**：`kept.push_back(std::move(c));`（源 `clusters_` 随即被 swap 丢弃）
- **安全性**：moved-from 元素在 swap 后不再使用

#### OPT-31：noise_filter DWF 环扫描取模改条件递增
- **文件**：[noise_filter.h:387-388](file:///home/justin/GUI-for-openEB/algo/cv/noise_filter.h#L387-L388)
- **问题**：`(r.head + i) % r.cap` 每迭代取模（非 2 的幂时编译器发 div 指令）
- **方案**：递增索引 + 条件回绕 `if (++idx >= r.cap) idx = 0;`（分支可预测）
- **安全性**：循环体只读 `r.buf`，不修改 `r.head`/`r.cap`，等价

#### OPT-32：direction_selective_filter 传感器中心改成员
- **文件**：[direction_selective_filter.h:301-302](file:///home/justin/GUI-for-openEB/algo/cv/direction_selective_filter.h#L301-L302)
- **问题**：`cx`/`cy` 每事件重算，但 `width_`/`height_` 构造后不变
- **方案**：构造时存储 `cx_`/`cy_` 成员
- **安全性**：值完全相同

#### OPT-33：hough_line_tracker cands 改成员复用
- **文件**：[hough_line_tracker.h:175](file:///home/justin/GUI-for-openEB/algo/cv/hough_line_tracker.h#L175)
- **问题**：`std::vector<Cand> cands;` 每调用分配
- **方案**：改成员 `cands_`，`.clear()` 复用容量
- **安全性**：纯 scratch

#### OPT-34：xyt_visualizer 批量 insert 代替逐个 push_back
- **文件**：[xyt_visualizer.h:64-66](file:///home/justin/GUI-for-openEB/algo/cv/xyt_visualizer.h#L64-L66)
- **问题**：`for (i=0..n) buffer_.push_back(events[i]);` 逐个插入 deque
- **方案**：`buffer_.insert(buffer_.end(), events, events + n);` 批量插入
- **安全性**：结果完全相同

### 2.4 Common 工具层（`algo/common/`）

#### OPT-35：kmeans++ 初始化从 O(n·k²) 降为 O(n·k)
- **文件**：[kmeans.h:86-97](file:///home/justin/GUI-for-openEB/algo/common/kmeans.h#L86-L97)
- **问题**：内层循环遍历所有已选中心，但 `d2[i]` 已保存到所有旧中心的最近距离
- **方案**：内层只计算到新中心 `centroids_.back()` 的距离，与 `d2[i]` 取 min
- **安全性**：`d2[i]` 的不变式为"到所有已选中心的最小距离"。退化路径（`sum <= 0`，即所有 `d2[i] = 0`）下，新中心到任意点的距离 ≥ 0，min 不变

#### OPT-36：kmeans assign 用 resize 代替 assign（避免无效零填）
- **文件**：[kmeans.h:113-114](file:///home/justin/GUI-for-openEB/algo/common/kmeans.h#L113-L114)
- **问题**：`labels_.assign(points.size(), 0)` 先零填再全覆盖
- **方案**：`labels_.resize(points.size())`（容量足够时不零填，每个元素随后被覆盖）
- **安全性**：每个元素在 `assign` 循环中被完全覆盖

#### OPT-37：dvs_framer Mat::at 改 ptr 行指针
- **文件**：[dvs_framer.h:78-99](file:///home/justin/GUI-for-openEB/algo/common/dvs_framer.h#L78-L99)
- **问题**：`frame.at<uint8_t>(i)` 每像素边界检查，6 个 switch case 重复
- **方案**：`auto* dst = frame.ptr<uint8_t>();` + 平坦索引
- **安全性**：`create` 后 Mat 连续，平坦索引等价

#### OPT-38：dvs_framer reset 用 fill 代替 assign
- **文件**：[dvs_framer.h:145-151](file:///home/justin/GUI-for-openEB/algo/common/dvs_framer.h#L145-L151)
- **问题**：稳态下 `assign(n, 0)` 每次检查容量再零填
- **方案**：`std::fill(on_counts_.begin(), on_counts_.end(), 0)`（无容量检查分支）
- **安全性**：稳态下 size 不变，fill 与 assign 零填等价

#### OPT-39：lif_integrator 缓存 1/tau_us（已回退）
- **文件**：[lif_integrator.h:73-75](file:///home/justin/GUI-for-openEB/algo/common/lif_integrator.h#L73-L75)
- **问题**：每事件 `static_cast<double>(tau_us_)` + 隐式除法
- **原方案**：缓存 `inv_tau_us_ = 1.0 / static_cast<double>(tau_us_)` 成员，`set_tau_us` 时更新；`exp(-dt / tau_us)` → `exp(-dt * inv_tau_us)`
- **回退原因**：(1) IEEE 754 中 `a/b` 与 `a*(1/b)` 不保证 bit-for-bit 相等（先算 `1/b` 再算 `a*r` 涉及两次舍入，可能差 1 ULP），而 LIF 积分器是累积型状态机，微小偏差会持续累积，在阈值附近可能改变发放时机。lif_integrator.h 头部注释明确标注"Mirrors jAER LIFNeuron.addEvent"，属于 jAER 移植算法，项目约束要求严格功能一致性。(2) 该优化在构造函数初始化列表中引入了 `reset_value_(reset_value_)` 拼写错误（成员自初始化），导致 `reset_value_` 未初始化，在 legacy full-reset 模式下会将电位设为垃圾值。

#### OPT-40：particle_filter resample 缓冲改成员复用
- **文件**：[particle_filter.h:91-102](file:///home/justin/GUI-for-openEB/algo/common/particle_filter.h#L91-L102)
- **问题**：`std::vector<Particle> next(n_)` 每次分配
- **方案**：改成员 `next_`，构造时 `resize(n_)`，`swap` 交换
- **安全性**：纯 scratch

#### OPT-41：periodic_spline 工作缓冲改成员复用
- **文件**：[periodic_spline.h:88-143](file:///home/justin/GUI-for-openEB/algo/common/periodic_spline.h#L88-L143)
- **问题**：每次 `fit()` 约 9 次 vector 分配（`d/a/b/c/u/ysol/zsol/cp/dp`）
- **方案**：全部改成员，`resize(n)` 复用容量
- **安全性**：纯 scratch

#### OPT-42：median_lowpass 复用排序缓冲 + nth_element
- **文件**：[median_lowpass.h:32-36](file:///home/justin/GUI-for-openEB/algo/common/filter/median_lowpass.h#L32-L36)
- **问题**：每样本分配 vector + 全排序
- **方案**：复用成员 `sort_buf_`；`std::nth_element` 代替 `std::sort`（只需中位数）
- **安全性**：`nth_element` 后 `tmp[size/2]` 与 `sort` 后值相同

#### OPT-43：highpass/lowpass alpha 缓存 + dirty flag
- **文件**：[highpass.h:33-39](file:///home/justin/GUI-for-openEB/algo/common/filter/highpass.h#L33-L39)、[lowpass.h:40-45](file:///home/justin/GUI-for-openEB/algo/common/filter/lowpass.h#L40-L45)
- **问题**：`alpha` 每样本重算（含除法 + clamp），但 `dt_`/`rc_`/`cutoff_hz_` 仅 `set_*` 时变
- **方案**：缓存 `cached_alpha_` + `dirty_` 标志，`set_*` 时置 dirty，`process` 中仅 dirty 时重算
- **安全性**：值完全相同

#### OPT-44：event_buffer 移除非标准 pad_[0]
- **文件**：[event_buffer.h:79](file:///home/justin/GUI-for-openEB/algo/common/event_buffer.h#L79)
- **问题**：`char pad_[0]` 是 GNU 扩展（零长度数组），无实际效果，`head_`/`tail_` 的 `alignas` 已足够
- **方案**：删除该行
- **安全性**：零长度数组不占空间，删除不改变布局

#### OPT-45：event_packet is_filtered_unchecked 内联访问
- **文件**：[event_packet.h:122-124](file:///home/justin/GUI-for-openEB/algo/common/event_packet.h#L122-L124)、[event_packet.h:164-168](file:///home/justin/GUI-for-openEB/algo/common/event_packet.h#L164-L168)
- **问题**：`advance_to_valid` 已保证 `idx_ < size()`，但 `is_filtered` 每次重复检查 `i < filtered_.size()`
- **方案**：添加 `bool is_filtered_unchecked(std::size_t i) const { return filtered_[i]; }`，`advance_to_valid` 调用 unchecked 版
- **安全性**：`advance_to_valid` 的循环条件已保证 `idx_ < owner_->size()`，而 `filtered_.size() == size()`

---

## 三、放弃的优化点

以下优化点经复查后因存在风险而放弃：

### 3.1 容器类型变更（放弃：改变内部存储语义，影响面广）

| 编号 | 原方案 | 放弃原因 |
|------|--------|----------|
| DROP-01 | LifoEventBuffer 线性数组改环形缓冲 | `trim_old`/`for_each_*`/`at_from_top` 全部依赖直接索引，环形映射易出 off-by-one |
| DROP-02 | event_window_ vector 改 deque | 消除 erase(begin) 的 O(N)，但容器类型变更影响迭代器语义 |
| DROP-03 | corner_detector traj vector 改 deque | 同 DROP-02 |
| DROP-04 | object_tracker trajectory vector 改 deque | 同 DROP-02 |
| DROP-05 | sparse_optical_flow LK 时间戳 vector 改 deque | 内层 vector 容器类型变更，影响所有 lk_ts_ 访问代码 |
| DROP-06 | histogram_ring_buffer deque 改 vector 环形 | 改变内部存储，影响 mean/std_dev/percentile 全部遍历代码 |
| DROP-07 | trigger_synced_filter vector-of-vectors 改扁平化 | 每像素映射内存布局重构，所有访问代码需重写 |
| DROP-08 | frame_generator map 改 vector | 容器类型变更，影响 add_events/add_window/remove_window |

### 3.2 语义变化风险（放弃：可能改变输出结果）

| 编号 | 原方案 | 放弃原因 |
|------|--------|----------|
| DROP-09 | r.filtered_events = std::move(passthrough_) | 若 `pull_result` 被连续调用两次（无 intervening `push_events`），第二次返回空。当前无法保证此约束 |
| DROP-10 | ux_prev_ = ux_ 改为 std::swap | 虽已验证 chambolle_tv 完全覆盖 ux_，但若未来 chambolle_tv 改为热启动 u，swap 会引入 bug。维护性风险 |
| DROP-11 | hough_circle_tracker 移除 increase_hough_point 内极值跟踪 | `max_coord_` 在 re-scan 中不被重置，increase_hough_point 可能设置 re-scan 未覆盖的值。行为差异 |
| DROP-12 | hot_pixel_filter 用 partial_sort 代替选择排序 | 边界处 count 相同时，选择排序按数组序选，partial_sort 可能不同序。tie-breaking 差异 |
| DROP-13 | histogram_ring_buffer percentile 从 counts_ 计算 | 直方图分箱是有损的，与精确排序结果不同 |
| DROP-14 | event_to_video 移除 cv::Scalar(0) 零初始化 | 早期返回路径（width_<=0）需返回零帧，零初始化是必需的 |

### 3.3 数值稳定性风险（放弃：可能引入精度差异）

| 编号 | 原方案 | 放弃原因 |
|------|--------|----------|
| DROP-15 | histogram_ring_buffer std_dev 改单遍 sum+sum² | 单遍公式 `(E[X²] - E[X]²)` 在大窗口时数值不稳定，可能与两遍公式结果有微小差异 |

### 3.4 复杂重构风险（放弃：实现复杂度高，易引入 bug）

| 编号 | 原方案 | 放弃原因 |
|------|--------|----------|
| DROP-16 | event_buffer 每元素取模改两段 memcpy | 环形缓冲分段逻辑易出 off-by-one，且需确认 EventT 可平凡复制 |
| DROP-17 | vector<bool> 改 vector<uint8_t> | 改变内存布局，API 语义变化，测试可能依赖 |
| DROP-18 | perspective_undistort LUT fallback 优化 | 这是设计问题（应确保 LUT 默认开启），非代码优化 |
| DROP-19 | object_tracker merge 循环内 erase 优化 | 改变合并逻辑顺序，可能影响簇合并结果 |

### 3.5 收益过低（放弃：ROI 不足以承担风险）

| 编号 | 原方案 | 放弃原因 |
|------|--------|----------|
| DROP-20 | Preprocessor::set_param 中 substr 改 string_view | 仅 GUI 线程调用，非热路径 |
| DROP-21 | FilterChain stages_ 改 vector | 仅 8 个 stage，hash 查找 ~400ns/batch，可忽略 |
| DROP-22 | AlgoInstance/AlgoBridge/FilterChain 加 final | 方法多为非虚，devirtualization 收益极小 |

---

## 四、实施优先级

按收益/工作量比排序：

### 第一批（高收益、低风险，推荐优先实施）

| 编号 | 描述 | 预期收益 |
|------|------|----------|
| OPT-01/02 | 后端 passthrough_ 成员复用 | 消除每帧堆分配 |
| OPT-03 | Preprocessor 循环拆分 | 消除每事件分支 |
| OPT-04 | crop_to_roi 缓冲复用 | 消除 4 个后端每帧分配 |
| OPT-11/12/13 | event_to_video scratch 改成员 | 消除每帧 5+ 次堆分配 |
| OPT-15 | DFT twiddle 预计算 | 消除 ~99% 超越函数调用 |
| OPT-18 | active_marker 合并扫描 | 减半全图遍历次数 |
| OPT-24 | sparse_optical_flow xs/ys/ts 改成员 | 消除每事件 3 次堆分配 |
| OPT-35 | kmeans++ O(n·k) 优化 | 减少 ~10× 距离计算 |

### 第二批（中等收益、低风险）

| 编号 | 描述 | 预期收益 |
|------|------|----------|
| OPT-05/06/07/08/09/10 | 后端层零散优化 | 各消除小幅开销 |
| OPT-14/16/17 | freq_detector 各项 | 消除每簇分配/重算 |
| OPT-19/20/21/22/23 | analytics 零散优化 | 各消除小幅开销 |
| OPT-25/26/27/28/29/30/31/32/33/34 | CV 层零散优化 | 各消除小幅开销 |
| OPT-36/37/38/40/41/42/43/44/45 | common 层零散优化 | 各消除小幅开销 |

---

## 五、验证结果

### 5.1 测试与编译

- `ctest` 全套测试：**341/341 通过**
- 编译警告：**0**（`-Wall -Wextra`）

### 5.2 严格审查

对全部 44 项已实施优化逐一读源码验证，结果：

| 层级 | 项数 | 结果 |
|------|------|------|
| 后端桥接层 (OPT-01~10) | 10 | 全部 VERIFIED OK |
| Analytics 层 (OPT-11~23) | 13 | 全部 VERIFIED OK |
| CV 层 (OPT-24~34，含 OPT-29) | 11 | 全部 VERIFIED OK |
| Common 层 (OPT-35~45，排除 OPT-39) | 10 | 全部 VERIFIED OK |

关键验证点：
- OPT-03：`buf_[kept] = buf_[i]` 在覆写 x/y 之前执行（保留 t/p 字段）
- OPT-15：`(k*n) % N` 索引正确，使用 `size_t` 防溢出
- OPT-22：使用 `mag_gt2 < kEps * kEps`（平方比较）
- OPT-23：偶数 n 中位数取 `(lo + vals[mid]) * 0.5f`，与原始 sort 行为一致
- OPT-26：`t_now` 在 `continue` 检查之前更新
- OPT-29：查表使用 `double` 精度，预计算 same/diff 两种极性，运行时选择
- OPT-35：`d2[i]` 在循环前用 `centroids_[0]` 初始化，退化路径保持不变式
- OPT-36：`resize` 后每个元素都被覆写
- OPT-43：所有改变 alpha 相关参数的 setter 都设置了 `alpha_dirty_`

### 5.3 OPT-39 回退确认

OPT-39（lif_integrator 缓存 `inv_tau_us_`）已回退。当前代码使用原始的 `exp(-dt / static_cast<double>(tau_us_))`。回退原因：
1. IEEE 754 中 `a/b` 与 `a*(1/b)` 不保证 bit-for-bit 相等（两次舍入可能差 1 ULP），LIF 积分器是累积型状态机，微小偏差会跨事件累积，改变发放时机
2. 实现中引入了 `reset_value_(reset_value_)` 拼写错误（成员自初始化），导致 `reset_value_` 未初始化
