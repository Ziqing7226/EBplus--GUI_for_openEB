# 系统性审计报告

> 审计对象：HEAD（caee2c0，v1.9.0 之后）。审计范围：`algo/`、`gui/`、`models/` 桥接层，对照 `ref/jaer` Java 原版与 `openeb/` SDK 源码。
> 审计方式：6 路并行深审（jAER 移植比对 ×2、死代码、算法逻辑、GUI–算法桥接、GUI 功能），全部结论以两侧代码行号为证据。
> 严重程度：**严重** = 崩溃/数据损坏/默认配置即产生错误结果；**高** = 特定条件下出错或功能实质失效；**中** = 边界情形/语义偏差；**低** = 健壮性/一致性。

## 统计总览

| 类别 | 严重 | 高 | 中 | 低 |
|---|---|---|---|---|
| 一、jAER 移植差异（滤波类） | 0 | 3 | 9 | ~20 |
| 二、jAER 移植差异（跟踪/运动类） | 0 | 5 | 9 | ~20 |
| 三、死代码 | — | 确定 32 处，疑似 8 处 | — | — |
| 四、算法库 BUG | 1 | 0 | 7 | 14 |
| 五、GUI–算法衔接 | 1 | 8 | 14 | 10 |
| 六、GUI 功能性问题 | 1 | 2 | 9 | 21 |

**三大主题性问题**（贯穿全部类别，详见 §七）：
1. **默认值三方漂移**：jAER 原版 / `algo/` 成员默认值 / GUI 注册表默认值 / `doc/design.md` 文档值，四方对同一参数经常各说各话。
2. **注释失实**：大量头注释声称"✅ 移植自 jAER X"或"对应 jAER 参数 X（默认 Y）"，与 Java 源码或实际实现不符。
3. **"调了没反应"参数**：注册表暴露给用户的参数中有多个在算法端是死参数（`n_sigma`、`min_radius`、`accumulator_decay_us`、`max_points`、`min_events`…），属于用户可见的正确性问题。

---

## 一、jAER 移植差异（滤波类）

分类约定：**A** = 移植 bug（实现与原版不符且非有意）；**B** = 有意差异但注释/实现/文档不一致；**C** = 有意差异且合理（仅列关键项）。

### 1.1 cluster_lif ↔ BlurringTunnelFilter（`algo/cv/cluster_lif.h`、`algo/common/lif_integrator.h` ↔ `ref/jaer/.../einsteintunnel/sensoryprocessing/BlurringTunnelFilter.java`）

已核对一致：网格维度公式、神经元中心、4 神经元路由与边界守卫、衰减→注入顺序、时间回退 MP=0、发放后下降 `max(th*pct/100, 1.0)`、默认值 tau 22000us/threshold 15/RF 8/jump 10%。

- **[A · 高] 初始膜电位与 jAER 实际行为不符，且注释失实。** jAER `LIFNeuron.reset()`：`membranePotential = MPInitialPercnetTh*MPThreshold`（Java:450, 1849）——**没有 /100**，默认 50×15=750（疑似 jAER 上游 bug，但即原版行为，神经元首个事件即发放并连续发放数百次）。C++ 按 `percent*threshold/100` 计算（lif_integrator.h:158-161 → 7.5），而 cluster_lif.h:21-22 注释却声称"jAER MPInitialPercnetTh，默认 50% → 7.5"。另叠加：jAER `lastEventTimestamp` 初值 0（首事件把初始 MP 从 t=0 起衰减，大时间戳下直接清零）；C++ `last_ts_` 初值 -1（首事件不衰减）。**后果**：流初期的簇输出与 jAER 完全不同。建议：明确取舍——若意在修正上游 bug，注释改为"有意修正 jAER 漏除 100"；若求忠实，去掉 /100 并按 t=0 基准衰减。
- **[B · 高] 分组语义不同，注释声称一致失实。** cluster_lif.h:28-29 称"与 jAER 的 4-邻接 inside/border 分组语义一致"。但 jAER 只有当发放神经元的**全部邻居也在发放**时才成组（INSIDE 需 4 邻居全发放 :1665、EDGE 3 个、CORNER 2 个）；C++ 是普通 4-连通分量（:152-194），稀疏发放也会产出簇——比 jAER 产出更多、更碎的簇。建议：注释明确近似关系，或实现"全包围核+边界"两阶段分组。
- **[B · 中] 质心权重缺少时间衰减因子。** jAER `NeuronGroup.add` 按成员 `lastEventTimestamp` 差做 `exp(dt/tau)` 折扣（Java:940-952）；C++ 是纯发放次数加权平均（:173-176）。同包内偏差有限。

### 1.2 direction_selective_filter ↔ AbstractDirectionSelectiveFilter（`algo/cv/direction_selective_filter.h`）

- **[A · 高] ori 感知路径方向表错位 90°（潜伏）。** jAER 合约：`unitDirs` 从 down 起（`unitDirs[0]=(0,-1)`），朝向通道 o 搜索 `unitDirs[o]` 与 `unitDirs[(o+4)%8]`，o=0 为水平边缘 → 搜 down/up（垂直于边缘）。C++ 方向表 0=E（:213-214），`compute_direction_ori` 直接套 `{ori, (ori+4)%8}`（:251）→ o=0 搜 E/W，即**沿边缘**搜索。本仓库 `orientation_filter.h` 的 ori 约定与 jAER 相同（0=水平边缘），错位成立。正确映射应为 `{(ori+2)%8, (ori+6)%8}`。当前 GUI 只走 raw 路径（filter_backends.cpp:163），属潜伏 bug，接 ori 路径前必须修。
- **[A · 中] 内部 LowPass 对相同/回退时间戳直接采用新样本**（:339-342 `else value = input`）；jAER LowpassFilter 在 dt≤0 时 fac=0 保持原值。包内大量同时间戳事件时，全局平移/旋转/膨胀估计会被最后样本反复覆盖，远比 jAER 噪声大。
- **[B · 中]** 时间窗默认 10000us（:355）vs jAER `maxDtThreshold` 100000us（10 倍差，未声明）；jAER `useAvgDtEnabled` 默认 true（用平均 dt 而非最小 recency）未移植。
- **[B · 低]** jAER `speedControlEnabled`（超速剔除）未移植；中心坐标 jAER 用整数除法 `sizex/2`，C++ 用 `width/2.0`（奇数宽差 0.5px）。

### 1.3 noise_filter ↔ BAF/STCF/Refractory/DWF/AgePolarity/Harmonic/Repetitious/SpatialBP（`algo/cv/noise_filter.h`）

公共基线已核对：jAER AbstractNoiseFilter `correlationTimeS` 默认 25ms、`letFirstEventThrough` 默认 false、首事件丢弃、判后写图、被滤事件也写图——C++ 均一致。

- **[A · 中] Repetitious：`thisdt < minDtToStore` 分支语义移植错误。** C++ `return false` **丢弃**该事件（:473-475），注释称 jAER `continue` 即 drop。但 jAER 的 `continue`（RepetitiousFilter.java:171-173）不调用 filterOut、输出迭代器从未使用 → 短 ISI 事件**保留通过**（仅不更新 avgDt）。后果：任何 >1kHz 的像素在 C++ 下被持续丢事件。建议：改为"放行但不更新 avgDt"，或明确文档化为有意加严。
- **[B · 中] Refractory 非单调事件放行**：jAER `deltat > refractory` 才过 → 时间回退事件被滤掉（Java:100-107）；C++ `e.t < lt` 直接放行（:368）。
- **[B · 中] DWF 单窗（FWF）模式**：jAER 扫描整 wLen=512 窗且所有事件无论通过与否都入窗（Java:196-223）；C++ 环容量恒为 wLen/2=256 且失败事件不入任何窗（:408-409, 580-583）→ 单窗模式窗口减半且只含信号事件（默认 double 模式不受影响）。另：jAER 填充期为前 512 个事件全滤，C++ 只填 256 个。
- **[B · 中] AgePolarity 中心自身默认计入得分**：jAER 默认 filterHotPixels=true → 跳过自身；C++ 仅当 `filter_hot_pixels_`（默认 false）才跳过（:429）→ 周期性热像素可白拿 ~1 分。
- **[B · 中] Harmonic f0 默认值注释失实**：C++ 注释"jAER f0 (natural frequency, Hz)"配默认 50（:659）；jAER 默认 100（HarmonicFilter.java:135）。
- **[B · 中] tau 默认值三方不一致**：AgePolarity C++ 10000us vs jAER 25000us vs design 3000；Refractory C++ 1ms vs jAER 25ms（design 已记载，有意但与 jAER 差 25 倍）。
- **[B · 低]** Harmonic：jAER power 混合 alpha 不钳位（长间隙后 power 可变负 → 恒滤除），C++ 钳到 [0,1] 且 power<=0 放行（更稳但未声明）；非单调时 jAER 保留状态，C++ 全清零。
- **[B · 低]** BAF `subsample_by` 语义不同：jAER 对地址图右移；C++ 只改邻域步长（subsample=1 时只查 4 个对角方向邻居），与注释"3x3 neighbour correlation"不符。GUI 已暴露该参数（默认 0 不受影响）。
- **[B · 低]** `filter_hot_pixels_` 与 jAER `filterHotPixels` 同名不同义：jAER=相关性测试排除自身；C++=额外 n-sigma 统计抑制（:525-551），且 BAF/STCF 恒排除自身与该开关无关、AgePolarity 又用该开关控制自身排除——三个模式语义不统一，建议统一。
- **[C · 中]** SpatialBP 半径默认 2/10 vs jAER 0/1（design 已记载，空间尺度完全不同）；Repetitious 死参数 `rep_period_us_`/`rep_tolerance_us_`（jAER 亦无对应）。

### 1.4 hot_pixel_filter ↔ HotPixelFilter + ProbFPNCorrectionFilter（`algo/cv/hot_pixel_filter.h`）

- **[B · 中] 热像素集合不跨学习期累积**：jAER `hotPixelSet` 只增不清（Java:238）；C++ 每个学习窗 `recompute_mask` 先全清零（:146），周期重学丢掉旧热像素。
- **[B · 中] ProbFPN 行内注释过度声称**：注释 `C1: probability proportional to ISI (jAER prob = alpha*isi/avgIsi)`（:111），但 jAER 用 IIR 平滑 ISI + 全局自适应 avgIsi + alpha=0.9 + 按 (x,y,type) 维护；C++ 是原始 dt + 固定目标率 + 隐含 1.0 + 按 (x,y)。属声明过的重设计，但行内注释应弱化。
- **[A · 低] 学习窗锚点**：`learn_start_s_` 初值 0（:177），非零基时间戳（实况相机 t≈1e9us）首窗立即结算，用极少事件重算掩码。jAER 锚到学习期首个事件。建议首事件时初始化锚点。
- **[B · 低]** 学习窗默认 5s vs jAER 500ms 且 jAER 为手动触发（design 已记载 5s）；键控粒度 jAER 按 address（含极性）→ C++ 按像素两路极性全滤（等价 jAER `use2DBooleanArray=true` 变体，未声明）；jAER resetFilter 保留热像素集，C++ reset 连掩码清。

### 1.5 background_mask_filter ↔ Histogram2DFilter（`algo/cv/background_mask_filter.h`）

核心公式逐项核对**一致**（直方图累计、`count > threshold`、腐蚀"任一邻居 < threshold 即拒绝"、边界 clip 复制、threshold 20、erosionSize 0）。

- **[B · 低]** jAER `collect` 初值 false（启动不学习，手动触发）；C++ `collect_` 初值 true（启动即学习）——配合自动冻结属重设计，但此点未单独声明。
- **[B · 低]** jAER setThreshold 在 collect 期间也重算 bitmap；C++ 仅 `!collect_` 时重算（:113），注释不完全准确。jAER `invertFiltering` 未移植。

### 1.6 trigger_synced_filter ↔ FilterSyncedEvents（`algo/cv/trigger_synced_filter.h`）

门条件逐运算符一致；t0/t1 默认 500/500 ✓。

- **[B · 中] 首个触发前的行为**：jAER 无守卫（lastTriggerTimestamp 初值 0，等效 t=0 隐式触发），流开始处满足窗口的 OFF 事件可通过；C++ 显式要求已见触发（:108），此前全滤。
- **[B · 低]** reset 范围：jAER 只清两张时间图；C++ 全清。
- 另见 §五 G3：该算法在 GUI 中触发源未接线，输出恒空。

### 1.7 common/filter/{lowpass, highpass, bandpass, angular_lowpass} ↔ util/filter/*.java

核心公式**逐项核对一致**（LowPass 的 fac 钳位与首样本初始化；HighPass 直接式与 jAER `x−LP(x)` 代数等价；Bandpass LP→HP 顺序；AngularLowpass 三分支距离与回卷逐行一致）。差异仅：C++ 用固定 sample_dt 而非每调用时间戳（头注释已声明）；退化输入（tau=0）行为不同（低）。

### 1.8 bandpass_filter ↔ util/filter/BandpassFilter（`algo/cv/bandpass_filter.h`）

- **[B · 低]** 首批样本被丢弃（:59 `last_t_=t; return`，jAER 会用首样本初始化 LP）；`dt<=0` 直接返回旧值。均未声明。

---

## 二、jAER 移植差异（跟踪/运动类）

### 2.1 optical_gyro ↔ OpticalGyro + RectangularClusterTracker（`algo/cv/optical_gyro.h`）

核心 LS 求解（SmallAngleTransformFinder 的 aden/anum/平移公式）、cosAngle/sinAngle 用旧 rotationAngle 的时序、mass 衰减公式——逐字符一致。

- **[B · 高] 跟踪器参数块注释"(jAER RectangularClusterTracker defaults)"完全不实**（:349-355）：mixing 0.2 vs jAER 0.05（4×）、mass_decay_tau 100000 vs 10000（10×）、min_visible_mass 2.0 vs 10~30、max_clusters 100 vs 10（10×）、cluster_size 15px vs 0.1×maxSize；`cluster_time_us_` 在 jAER 中无对应物。后果：C++ 簇更长寿、更易可见、位置响应慢 4 倍，陀螺估计的时间尺度与 jAER 完全不同。建议：改值对齐 jAER，或注释改为"自研默认值"。
- **[B · 中] 位移估计加入速度外推，注释虚构 jAER 公式**（:221-231, 42-44）：jAER 位移就是 `location − birthLocation`（OpticalGyro.java:175-176），`velocityPPt` 仅用于 transformEvent 的 gainVelocity 项，不存在"velocityPPt = velocity×(t−lastUpdateT)"。静止较久的簇位移被外推放大。建议：删除外推或改注释。
- **[A · 中] 缺少 jAER birthLocation 首次可见时重置语义**（RCT.java:2358-2361）：jAER 簇首次可见时把 birthLocation 重置为当前位置；C++ 固定为种子事件位置（:150-152），位移基线含簇未成熟期噪声。
- **[B · 中] "Equivalent to jAER LowpassFilter"声明错误**（:325-329）：C++ 用 `alpha = 1−exp(−dt/tau)`（dt≥tau 只走 63%），jAER 用线性 `fac=min(1,dt/tau)`（dt≥tau 直接吸附）；时间回退时 jAER 保持、C++ 跳变到新值。
- **[B · 低]** rotation 开启但可见簇<3（或 LS 近奇异）时：jAER 完全冻结估计；C++ 仍用 mass-weighted 平移更新滤波器（未声明）。簇速度单位 px/us vs jAER px/s、无 velocityTauMs=100 低通（注释却写"jAER Cluster.velocity"）。

### 2.2 sparse_optical_flow ↔ rbodo opticalflow 套件（`algo/cv/sparse_optical_flow.h`）

模式对应：LocalPlanes↔LocalPlanesFlow、LucasKanade↔LucasKanadeFlow（✅ 移植声明）、BlockMatch↔PatchMatchFlow、ClusterOF↔ClusterBasedOpticalFlow。

**LucasKanade 模式**：
- **[B · 高] 中心差分多了 0.5 因子，注释却点名"jAER CentralFiniteDifferenceFirstOrder"**（:328,337-338）：jAER 无 0.5（LucasKanadeFlow.java:287-292）。结构张量 ×0.25 → 特征值缩小 4 倍而门限 thr=1 相同 → 拒绝率大幅上升（相当于 jAER thr=4）；解幅值 ×2。
- **[B · 高] 时间导数缩放改为 1e6/win 未声明**（:294,339）：jAER 为 `it = cnt×40` 启发式；默认 win=20000 时 C++ 为 50·cnt。与上一条叠加，默认窗口下 C++ 幅值 ≈ 2.5× jAER。建议：决定"忠实移植"（去 0.5、用 cnt×40）还是显式声明"单位清理版"。
- **[B · 低]** `time_window` 默认注释错（写 20000，jAER `maxDtThreshold` 默认 50000，design.md:1058 同错）；`search_radius_px_` 默认 4 vs jAER 3；被门控拒绝的事件不输出 vs jAER 输出零向量。
- 已核对一致：2D/1D 解公式逐字符一致；currPix 公式、deque 剪枝一致。

**LocalPlanes 模式**：
- **[B · 中] 缺少 jAER LP 的 50ms per-pixel refractory**（LocalPlanesFlow.java:53,113-115）：C++ 每事件都拟合，输出密度高 1-2 个数量级，拟合样本构成不同。
- **[B · 低]** 当前事件不进邻域（jAER 进）；SAE 无极性分离（jAER 按 type 分）；`grad_thr_=1e-6` 与 jAER `th3=3e-3` 语义/数值完全不同。

**BlockMatch 模式**：
- **[B · 中] 块几何不同**：C++ 恒定 9×9（:426）vs jAER 按尺度放大块（粗尺度 7×7 → 全分辨率 25×25，PatchMatchFlow.java:1863）。
- **[B · 低]** 缺 nonZeroMatchCount 重叠像素门控（jAER :1932-1935 要求两块重叠非零像素 ≥ minValidPixNum）；搜索半径随尺度缩小（`sr0>>s`）vs jAER 恒定。
- 已核对一致：符号约定、归一化 SAD+dispersion 混合公式、全部默认值、LDSP→SDSP 菱形搜索结构。

**ClusterOF 模式**：
- **[B · 低]** `cluster_ema_alpha` 注释张冠李戴（:667,732 称"jAER spatialSmoothingFactor"，该参数实为网格流场平滑系数；质心 EMA 对应 RCT 的 locationMixingFactor，数值巧合一致）。design.md:1056 同错。

### 2.3 object_tracker ↔ RCT/MedianTracker/KalmanEventFilter（`algo/cv/object_tracker.h`）

- **[B · 高] MultiHypothesis 模式"✅ 移植自 jAER KalmanEventFilter"夸大**（:8-12）：相同的只有"逐事件 Mahalanobis 最近分配 + 阈值内 correct 否则新建 + 每包 predict"骨架；jAER 门限 3.0（C++ χ²=5.99）、无概率概念（剪枝靠 `Sigma[0][0] > maxPositionVariance`）、无合并、无上限、best 选择要求同一滤波器连续 10 包最多更新。C++ 的概率池/0.9 衰减/归一化/剪枝/合并/池上限 5 全为自研。建议注释改为"借鉴分配/生成骨架"。
- **RCT 模式速度未低通**（:554-564，jAER 有 velocityTauMs=100 LowpassFilter）——此项与 §四 S1 的严重 bug 直接相关，见该节。
- 已核对一致：RCT 参数默认值与 jAER 字段默认一致（mixing 0.05、mass tau 10000、threshold mass 10）；mass 更新公式一致；合并判据与 `isTouching` 一致。

### 2.4 line_segment_detector ↔ ELiSeD（`algo/cv/line_segment_detector.h`）

- **[B · 高] "✅ 移植自 jAER ELiSeD"夸大**：只移植了时间戳 Sobel 梯度计算；jAER 核心是**持久 LineSupport 区域**（逐事件按 22.5° 容差生长/合并/分裂，8000 事件环形缓冲驱动），C++ 改为每包角度量化 4 bin + 矩拟合——输出线段的稳定性/持续性/抗噪性完全不同量级。建议头注释限定到梯度部分。
- **[B · 中] predictTimestamps（jAER 默认开启）未移植**：邻居缺失/过期时 jAER 取对侧像素时间戳保持梯度对称（ELiSeD.java:617-625）；C++ 直接跳过 → 活动边缘处梯度缺失率更高。
- **[B · 低]** Sobel 核符号与 jAER 相反（mod-180 量化下无输出影响）；jAER 默认 5×5 核 vs C++ 固定 3×3；`orientation_threshold_` 死参数。
- 已核对一致：角度公式、矩拟合（length=sqrt(12·λ1)、质心/端点）精确一致。
- 文档错误：design.md:1114 把文献写成"Cartucho 2018 IROS"，应为 Brandli et al. EBCCSP 2016（C++ 头注释正确）。

### 2.5 hough_line_tracker ↔ HoughLineTracker（`algo/cv/hough_line_tracker.h`）

- **[B · 中] 头注释声称"按时间常数 accumulatorDecayUs 指数衰减"，实现与注释不符**（:5-6 vs :166-171 每包乘固定因子，与 jAER 一致）；`accumulator_decay_us_` 是死参数（:282），且经 GUI 注册暴露给用户（调了无效，见 §三-32）。design.md:1130 同错。
- **[B · 低]** 衰减时机：jAER 先投票再衰减且峰检测在衰减后的值上进行；C++ 先衰减再投票（:64-66）。ρ 未中心化（jAER 先减 sx2/sy2）；输出角度与 jAER 差 90°（C++ 输出线方向角，jAER 输出法线角）——GUI 内部自洽但不可与 jAER 对比。
- **[B · 中] 输出语义重写未声明**：jAER 单峰 + LowpassFilter/AngularLowpassFilter 平滑（tauMs=10）+ favorVertical 裁剪；C++ 多峰 + NMS + ≤16 条线 + 航迹（新特性，头注释有描述但"丢弃输出低通"未声明）。

### 2.6 hough_circle_tracker ↔ HoughCircleTracker（`algo/cv/hough_circle_tracker.h`）

**最忠实的移植**：8 扇区整数椭圆绘制、非指数衰减 1/(0.0001·decay·dt)、FIFO 反向投票、双阶段 maxValue 重置、islocmax 边界——逐段一致。

- **[B · 低]** threshold 默认 30 vs jAER 15，差异清单未列。
- **[B · 低]** 悄悄修复了 jAER locDepression 的笔误（jAER `[x-1][y-1]` 写两次、`[x+1][y-1]` 缺失；C++ 8 邻域全部正确）——行为更好但与原版不同，建议在差异清单中注明。

### 2.7 orientation_filter ↔ SimpleOrientationFilter（`algo/cv/orientation_filter.h`）

核心路径逐项核对**一致**（先写时间图再算 dt、RF offsets、avg/max+outlier rejection、WTA+decideHelper 平局裁决，头注释 jAER 行号引用全部吻合）。

- **[B · 中] oriHistory 语义不同 + 默认因子注释错误**（:192-204）：C++ 把历史当**平滑器**（输出混合值）；jAER 是**门控**（float map 初值 -1 避免水平偏置，|f−dir|>0.5 拒绝事件，输出原始 dir）；jAER `oriHistoryMixingFactor` 默认 **0.1**（C++ 注释称 0.25）。默认关闭，影响有限。
- **[B · 低]** dt_reject_threshold 默认 200000 vs jAER 500000；死参数/未实现：`multi_ori_output_`（setter 承诺了 jAER 行为但未实现）、`pass_all_events_`（两分支都 return −1）、`min_neighbors_`、`time_window_us_`；越界 RF 像素处理与 jAER 不同（C++ 更合理）。

### 2.8 orientation_cluster ↔ rccar/OrientationCluster（`algo/cv/orientation_cluster.h`）

**逐项核对，核心算法与默认值全部一致**（向量累加、旋转表、三条件门控、history 更新顺序、边界内缩、全部默认值）。

- **[B · 低]** useOppositePolarity=false 时：jAER 因未重置 xx/yy 产生陈旧向量（jAER bug）；C++ 改为跳过（更好但未声明）。
- 文档：design.md §4.3.17 仍描述旧的连通域聚类版本（min_cluster_size/coherence_threshold），与现实现不符。

### 2.9 kalman_filter / particle_filter ↔ labyrinthkalman / util ParticleFilter

**已逐项核对，公式正确，无 A/B 类问题**。Kalman 的 F·P·Fᵀ 展开、Q 矩阵与 LabyrinthBallKalmanFilter 位置-速度块精确一致（C++ 为 4 态 CV vs jAER 6 态 CA，头注释已声明）；ParticleFilter 为重采样族等价实现（systematic vs 随机抽样），头注释准确。

### 2.10 hough_line/hough_circle 以外其余：analytics 系（flow_statistics、auto_bias、particle_counter、isi_analyzer）为 "Inspired by" 级借鉴，未发现虚假移植声明。

---

## 三、死代码

验证基础：全量重建通过（0 error）、CTest 注册 296 个测试；所有 `.cpp` 均被 CMake 目标收录；符号引用经 `git grep -w` 至少两种拼写交叉验证（含 Qt 信号槽）。旧文档 `doc/dead_code_audit.md` 的结论已独立复核，以下为当前残留/新增。

### 3.1 确定死代码（32 处）

**A. 整文件/整个类无引用（5 项）**

| # | 位置 | 说明 |
|---|---|---|
| 1 | `gui/widgets/multiline_text.h:33` | `MultilineText` 类（~150 行）无任何 include/实例化；CMakeLists.txt:69 仅一句注释。删除或按 design §1.6.6 接入 HUD。 |
| 2 | `gui/widgets/mouse_adaptor.{h,cpp}` | 已编译但从未实例化；仅 pixel_probe.h:9 一句注释提及。删除（连同 CMakeLists.txt:74）或接入显示鼠标映射。 |
| 3 | `gui/widgets/pixel_probe.{h,cpp}` | 同上，从未实例化（CMakeLists.txt:75）。删除或接入（jAER 像素探针功能）。 |
| 4 | `gui/widgets/target_labeler.{h,cpp}` | 同上（CMakeLists.txt:73）。删除。 |
| 5 | `algo/tests/noise_tester.h` + `signal_noise_event.h` | jAER NoiseTesterFilter 移植的评估框架，无任何测试使用（design §4.6.1 的 TP/FP 评估从未落地）。删除两个文件或补真正的 gtest。 |

**B. 注册了但运行时不可达的算法后端（4 项，合计约 1000 行）**

| # | 位置 | 说明 |
|---|---|---|
| 6 | `backends/openeb_filter_backends.cpp`、`openeb_frame_backends.cpp`（7 类）、`openeb_preproc_backends.cpp`（5 类）、`openeb_util_backends.cpp`（6 类）+ `backend_factory.cpp:22-25` + `algo_bridge.cpp:495-667` 注册元数据 | 20 个 openeb_* 后端类：GUI 创建实例的路径只有 AlgorithmsPanel 复选框（`algorithms_panel.cpp:70,91` 明确跳过 `source != "self"`）和两处特例，Preprocess/Frame Mode 菜单已删除。注意 `main_window.cpp:421` 注释声称"frame mode selection lives in the DisplayPanel"，但 display_panel.cpp 全文无此功能——**注释已失实**。二选一：UI 给出入口，或删除 4 个 backend 文件 + 注册项 + 工厂分支。 |
| 7 | `algo_bridge.cpp:615-620` | `preproc_hw_diff`/`preproc_hw_histo` 注册了却连后端工厂分支都没有。 |
| 8 | `cv_backends.cpp:162` + `algo/cv/perspective_undistort.h:78` | `perspective_undistort` 链条整体失效：(a) backend 从不调 `set_calibration()`/`precompute_lut()` → `K_` 恒空 → `process()` 恒早退，启用该算法**永远无效果**；(b) `PerspectiveUndistortBackend(int w, int h) : algo_(w, h)` 把宽高传给 `(bool use_lut, bool undistort, bool rectify)` 两个 bool 形参（隐式 int→bool），几乎肯定是误以为构造函数接收宽高。且 caee2c0 已在 Preprocessor 实现了真正能用的 undistort 链路。建议删除该注册项+backend，或修复构造并接入标定 YAML。 |
| 9 | `analytics_backends.cpp:328` | `FlowStatisticsBackend::algo_` 从不被喂事件（`push_events` 只累加计数），FlowStatistics 的统计/渲染 API 在 GUI 路径完全不可达。删成员或实现 GT 流程。 |

**C. 声明+定义齐全但零调用的方法（20 项，编号 10-29）**

`CameraController::last_timestamp_us()`（连带 `last_ts_` 成员只写不读）、`save_config()`/`load_config()`（main_window 用 ConfigManager，此路径已死）、`is_cd_broadcast()`、`FramePipeline::file_is_playing()/is_file_mode()`、`FileFrameGenerator::has_events()`、`RecorderController::current_path()`、`PlaybackController::is_playing()/available()`、`FileConverter::is_running()`、`ExporterController::is_running()`、`FilterChain::stage_names()`、`AlgoBridge::push_events(inst,…)/pull_result(inst)` 静态薄包装、`ThemeController::effective_background_hex()`、`SettingsPanel::current_title()`+`current_title_changed` 信号（有 emit 无 connect）、`DisplayPanel::color_palette_index()`、`AlgoWindow::status_label()`、`FrameAnnotator::set_pen_width()/pen_width()/set_default_color()/default_color()`（4 个存取器均无调用）、`ChessboardDisplay::stop_flicker()/square_size_px()`、`playback_controls.h:51` 信号 `crop_range_requested`（从未 emit/从未 connect）、`main_window.h:195` 成员 `m_calibration_`（零使用）、`m_tools_`（只写不读，可改局部变量）。

- `algo/calibration/intrinsic.{h,cpp}` 的 undistort LUT 死链（**28**）：`board_size()`、`undistort_map_x/y()`、`build_event_undistort_lut()`/`event_undistort_lut()`/`undistort_point()`、`IntrinsicResult::undistort_map_x/y` 字段——全无调用方，且 `run()` 每次对全图白跑 `cv::initUndistortRectifyMap`+`cv::undistortPoints`（intrinsic.cpp:151-153），结果无人消费（Preprocessor 用自己的实现）。建议删除方法/字段及 run() 内的 LUT 预计算。
- `HoughCircleTracker::hough_threshold()/set_hough_threshold()` 兼容别名（**29**）：连测试都不用。
- `FilteredEventPacket` 整个类（event_packet.h:106-188，~85 行）+ `EventCDPacket`/`MutableEventCDPacket` 别名（**30**）：零引用；项目滤波统一用原位压缩而非 jAER 的 filteredOut 标记。

**D. 只写不读成员 + 仍接线的 legacy 参数（2 项，用户可见）**

| # | 位置 | 说明 |
|---|---|---|
| 31 | `hot_pixel_filter.h:46,52,168` `n_sigma_` | 头文件自标 "deprecated, unused"，但注册表（algo_bridge.cpp:693）暴露 "n_sigma" 参数、backend 转发、三个测试文件都在设置它——**用户调这个参数没有任何效果**。删全链路或真正实现 σ 阈值。 |
| 32 | `hough_circle_tracker.h:472,475` `min_radius_px_`/`accumulator_decay_us_` | 算法只用 `max_radius_px_` 和 `decay_`；但注册表暴露 "min_radius"（:774）和 "accumulator_decay_us"（:777），backend 转发，测试还在测这个 no-op setter。同 31 处理。 |

### 3.2 疑似死代码（8 项，需设计决策）

- **S1** `CalibrationPattern::CircleGrid/AsymmetricCircles` 分支：wizard 硬编码 Chessboard，两个枚举值永不执行（预留库 API）。
- **S2** "calibration" 类目残余（algo_bridge.cpp:362,401,428 三处恒真检查 + algorithms_panel.cpp:77 映射）：注册表已无 calibration 类算法。
- **S3** `DirectionSelectiveFilter` 朝向感知路径（`classify(e,ori)`、`process(events,oris,count,out)`）：设计用于接 OrientationFilter 输出，但零外部调用（且该路径有 §一-1.2 的 90° 错位潜伏 bug）。
- **S4** `PerformanceMeter` 大部分 API：生产只用 4 个方法；`fps()` 每次 `tick_frame` 白算 IIR fps；`EventRateEstimator` 连测试都不用。
- **S5** 各算法类未接线的 jAER 对齐 setter/getter（约 20 组，sparse_optical_flow 11 个、freq_detector 9 个、corner_detector 6 个等）——库 API 面，删除与否属库设计决策；另注意 `freq_detector` 注册显示模式是 Standalone 却从不产帧。
- **S6** 算法库储备三件套（angular_lowpass/periodic_spline/particle_filter 仅测试引用）：旧审计已声明"保留储备"，维持原判。
- **S7** 7 个 standalone 诊断程序（test_raw_e2v 等）被编译但未注册 CTest——补注册或删除。
- **S8** `filter_chain.h:49-53` 注释引用 `chain_mutex()` 但该函数无头文件声明——陈旧注释（main_window.cpp:1344 同）。

---

## 四、算法库 BUG 与逻辑错误

### 严重（1 项）

**S1. ObjectTracker RCT/Median 模式：速度无低通 + 包级外推 → 默认配置下跟踪完全失效**（`algo/cv/object_tracker.h:543-565, 278-282`）
- `update_velocity` 每事件直接覆盖速度 `vx_ = (x_ − prev_x_)/dt_s`：x_ 被 IIR(0.05) 平滑，单事件位移 ~0.05px，dt_s 为单事件间隔（µs 级）→ 瞬时速度 ~±5×10⁴ px/s，且**无任何低通**（jAER RCT 用 velocityTauMs=100ms LowpassFilter，RectangularClusterTracker.java:2080）。
- `age(dt)` 每包末尾外推 `x_ += vx_*dt_s`，dt_s 是整个包间隔（如 10ms）→ 每包漂移 ~10² px → 匹配失败 → 每包新建簇 → 对象数单调 +1、坐标飞出传感器。
- 动态复现（静止目标）：默认参数 pkt0 `x=7.8, vx=−16026` → pkt3 共 3 个簇 `x=−467.7`；关掉 `enable_velocity_prediction` 后 4 包稳定 `x=11.0`、1 个簇。
- **佐证**：`algo/tests/test_raw_algos.cpp:308-313` 的注释已在真实数据上观察到该现象（"x=−13849 on a 640-wide sensor"）但被误认为"合法速度外推"而豁免断言——这本身就是该 bug 的表现。
- 修复：① update_velocity 恢复 jAER 式低通（或对瞬时速度 clamp 到物理上限）；② age() 外推位移 clamp（如 ±cluster_size_px）；③ 修正 test_raw_algos.cpp 的错误注释，补"静止目标位置不漂移"回归断言。

### 中（7 项）

- **M1. ObjectTracker：`prev_batch_t_` 无首包哨兵**（:94,123-127）：初值 0，大时间戳首包（录制裁剪回放、切相机）时 `age(t0)` → 簇 `since_last_us_=t0 > max_lost_us_` → 第二包全部簇被删并以新 id 重建（实测 t0=1e9 时 pkt0 objects=0、pkt1 起 id 从 1 开始）。修复：改 -1 哨兵，首包用首事件时间初始化。
- **M2. TimeSurface Split 模式亮度恒减半**（`time_surface.h:116-118`）：`cv::Vec3b` 是**饱和**加（255+255=255），`(c_off+c_on)*0.5` 使单极性满亮像素输出仅 128。修复：改 `cv::max(c_off, c_on)` 或饱和加后不乘 0.5。
- **M3. E2VID 热像素掩码与 downsample 网格坐标错位**（`e2vid_inference.h:116,282-285` + `event_voxel_grid.h:67-72`）：voxel grid 按半分辨率 (W/2)×(H/2) 构造，调用方按全分辨率 W×H 提供 mask；`build()` 以 `(y/2)*(W/2)+(x/2)` 索引全尺寸 mask——读到 mask 前 1/4 区域，热像素不屏蔽、正常像素被误屏蔽。当前 GUI 未接线该 API（潜伏）。修复：set_hot_pixel_mask 时校验/重采样尺寸，或 build() 按全分辨率查 mask。
- **M4. intrinsic.cpp AsymmetricCircles 物点网格公式错误**（:43-51）：OpenCV 官方应为 `x=(2c + r%2)*square, y=r*square`；现实现 `x=c*square, y=(2r+(c&1))*square`——奇偶偏移放到 y 方向、x 向间距少一倍，物点序与 `findCirclesGrid` 检测点序不匹配 → 选 AsymmetricCircles 时标定结果错误或异常。当前 GUI 只用 Chessboard（潜伏）。修复：`pts.emplace_back((2*c+(r&1))*square, r*square, 0)`。
- **M5. ParticleCounter：`prev_cy_` map 与 `size_hist_` 无界增长**（:266-271,119）：删 track 不清 map，id 单调递增不复用 → map 只增不减；`size_hist_` 每轮追加无上限。长时间运行内存持续增长。修复：删 track 时 `prev_cy_.erase(id)`；size_hist_ 加容量上限。
- **M6. LineSegmentDetector/HoughLineTracker：track 列表只增不剪**（`line_segment_detector.h:260-281`、`hough_line_tracker.h:252-272`）：无陈旧 track 删除（对比 object_tracker 有 prune_lost）。LineSegment 关联容差仅 5px，运动线段每包新建 track → 内存 + O(n) 关联退化。修复：Track 增加 last_seen，定期剪除。
- **M7. EventToVideo：`decay_tau_ms_` 衰减语义无效**（`event_to_video.h:106-117`）：`get_frame()` 先全量重建 `log_intensity_` 再乘 `exp(−dt/tau)`——衰减不跨帧累积（下一帧又被重建覆盖），实际效果只是按帧间隔对当帧亮度做全局缩放，与注释声称的"时间平滑"不符。修复：删除该参数或将衰减应用于跨帧状态。

### 低（14 项）

1. `hot_pixel_filter.h:70-75`：`learn_start_s_` 初值 0 而非首事件时间（同 §一-1.4 A 项，低频热像素首轮漏检）。
2. `hough_line_tracker.h:282`、`hough_circle_tracker.h:475`：`accumulator_decay_us_` 死参数（见 §三-32）。
3. `lif_integrator.h:101-108`：`leak_global()` 不更新 `last_ts_`，之后 `add_event` 按完整间隔再衰减一次 → 双重衰减（仅测试调用，公共 API 陷阱）。
4. `orientation_filter.h:187-190`：`return pass_all_events_ ? -1 : -1;` 两分支相同，参数无效。
5. `noise_filter.h:319-336`：BAF `subsample_by` 邻域步长语义与注释/jAER 均不符（见 §一-1.3）。
6. `corner_detector.h:327-331`：`num_orientations_` 允许 [4,8] 但 `kBaseDx/kBaseDy` 仅 4 项按 `%4` 取模——>4 时 EndStopped 行走方向与 bin 语义错位（GUI 未暴露，潜伏）。
7. `histogram_ring_buffer.h:78-87`：`percentile(q)` 未 clamp q∈[0,100]，越界 UB（当前调用方传常量）。
8. t==0 哨兵：`time_surface.h:66`、`corner_detector.h:264`、`direction_selective_filter.h:233` 用 0 表示"从未见过"，但 0 是合法时间戳；TimeSurface reset 后前 decay_time_us 内未激活像素显示近白。建议 -1。
9. `orientation_cluster.h:254-255`：`set_rf_width/height` 无 clamp，负值 → `assign(size_t(-1)*rows)` 巨量分配抛异常；`thr_gradient>1` 时 `dy_thr` 变负 → neighbor_thr 失效（GUI 允许 [0,100]，jAER 语义 [0,1]）。
10. `hough_circle_tracker.h:328`：`aa = max_radius_px_*max_radius_px_` int 平方，radius≥46341 溢出 UB（需极端参数）。
11. `intrinsic.cpp:139-158`：`result.ok=true` 先于 `precompute_undistort_lut()`；后者抛异常时 catch 只设 error 不回滚 ok → `ok==true` 与 error 并存（wizard 只查 ok）。
12. dead params 登记：`blob_detector.h` `histogram_window_s_`、`line_segment_detector.h` `orientation_threshold_`、`noise_filter.h` `rep_period_us_`/`rep_tolerance_us_`。
13. `sparse_optical_flow.h:464-465`：`motion_x = (init_x − dx) << s` 对可能为负的值左移，C++17 下 UB；改 `*(1<<s)`。
14. `isi_analyzer.h:86-93`、`particle_counter.h:170`：bins 接近图宽时 `bw−1` 可为 0/负；`render_size_hist(bins=0)` 除零（GUI 未暴露）。

### 待验证疑点（algo 层）

1. **TriggerSyncedFilter 触发源未接线**——已在 GUI 层确认（见 §五 G3），输出恒空；且 `triggers_` 列表只增不减、`process()` 每包从 0 重扫，一旦接线会 O(n²) 退化。
2. **ClusterOF 无簇数上限**（`sparse_optical_flow.h:650-657`）：噪声突发时段内可膨胀到数千簇 → 每事件 O(n) 关联卡顿。验证：高噪 raw 文件统计 `of_clusters_.size()` 峰值。
3. **OrientationCluster 时间回退**：`t = float(e.t − nb.ts)+1` 在 seek 未 reset 时可为 0/负 → inf/大负权重（jAER 原版同样不防），依赖 backend 在 seek 时 reset。
4. **多算法 `latest_t_`/`current_t_` 只增不减**：回放 seek 后窗口锚点停留在未来，依赖每条 seek 路径都触发 reset（GUI 层已确认 seek/loop/换源均 reset，见 §五 E4）。

---

## 五、GUI–算法衔接与参数传递

### 5.1 参数注册完整性

- **A1【严重】corner_detector 的 mode 枚举 GUI 标签与算法枚举完全错位**（`algo_bridge.cpp:749-751` 注册 `{"0=Harris","1=FAST","2=AGAST"}`；`cv_backends.cpp:304-307` 裸 `static_cast<Mode>(m)`；而 `algo/cv/corner_detector.h:55` 枚举是 `{EndStopped, TypeCoincidence, Harris}`）。选 "Harris" 实际运行 EndStopped，"FAST"→TypeCoincidence，"AGAST"→Harris；默认 "0" 让算法以 EndStopped 运行而界面显示 Harris。**标签全部为假，一行注册表改动可修**（或 backend 做名称→枚举映射，禁止裸 static_cast）。
- **A2【中】24 个 OpenEB 注册项无 GUI 入口**（死注册，详见 §三-B6/7）；`algo_bridge.h:10` 的"OpenEB 包装算法返回 nullptr → 透传"注释已与现实矛盾；wiki/Algorithms.md:6 宣称"30 OpenEB-wrapped capabilities"可用——口径需统一。
- **A3【中】三个"空转"控件**：orientation_cluster 的 `min_events`（算法端 :269,343 明确 "Stored, unused"）；hough_circle 的 `min_radius`/`accumulator_decay_us`（§三-32）；xyt_visualizer 的 `max_points`（backend 只存不用，且 3D 显示用的是 SpaceTimeDisplay 自有的另一个实例，此参数对任何东西都无影响）。
- **A4【中】算法有公开参数但 GUI 调不到**：corner_detector 9 个 setter（accumulation_ms/track_radius_px/min_track_len/output_hz/coincidence_window_us/neighborhood_radius/end_stopped_distance/max_age_us/num_orientations）；blob_detector 3 个；noise_filter 的 `rep_averaging_samples`；time_surface `refresh_rate_hz`（backend 硬编码 30）；isi_analyzer `bin_count`（硬编码 32）；E2VID `normalize_input`（硬编码 false）；xyt `color_mode/point_size/auto_rotate/depth_shade`；trigger_synced `t1_us`（backend 支持但未注册）。
- **A5【低】** AlgorithmsPanel 的 "Calibration" 分组永远为空（§三-S2）。

### 5.2 参数传递正确性

- **B1【中】浮点控件完全不应用注册的 min/max/step，int 控件却裁剪——两套策略**（`algorithms_panel.cpp:167` vs `:176`）：float 控件 `setRange(-1e9,1e9)`、step=1.0。后果：用户可输入越界值；裁剪发生在算法端但 `param_values_` 存原始字符串 → **GUI 显示值 ≠ 算法实际值**，ConfigManager 保存越界值；step=1.0 对 0.001 量级参数一点就飞出范围。而有些 setter 不裁剪（如 orientation_cluster::set_dt）——同面板三种行为。修复：float 控件也用注册 min/max，step 按 (max−min)/100。
- **B2【中】background_mask 的 "learning_rate" 实际接到 `set_learning_window_s`**（`filter_backends.cpp:250`）：控件名是"学习率"实为"学习窗口（秒）"；注册默认 0.05、上限 1.0——**用户永远无法设到 jAER 风格的 5s 工作点**。修复：重命名 `learning_window_s`，默认 5.0，范围 0.1–60。
- **B3【中】注册默认值与算法默认值系统性漂移**（AlgoInstance 构造时注册值全覆盖算法值，algo_bridge.cpp:29-31）。清单（注册值 vs 算法值）：object_tracker `mass_decay_tau_us` 100000 vs 10000；blob `threshold` 10 vs 50；sparse_flow `search_radius` 8 vs 4；corner `min_score` 0.01 vs 0.1；cluster_lif tau 10/threshold 1.0 vs 22/15；line_segment 10/3 vs 20/5；freq_detector 50/0.5 vs 3/1.0。噪声预处理一整组：baf_dt 1000 vs 25000、STCF corr 0.005 vs 0.025、pol_match/coincidence false vs jAER true、**dwf_wlen 默认 2 且上限 100 vs jAER 512（用户无法恢复设计工作点）**、dwf_double false vs true、agep_tau 3000 vs 10000、agep_thresh 2.0 vs 5.0、harm Q 5 vs 3、rep_ratio 10 vs 2、sbp_dt 10000 vs 8000。修复：逐项确认"刻意调参"还是笔误；至少放开 dwf_wlen 上限；刻意值写进 design.md。
- **B4【低】** particle_counter `min_area` GUI 范围 1–10000 但算法 clamp [1,1000]；`line_y` 默认 360 硬编码覆盖算法的 height/2（非 720p 传感器错位）；模式枚举靠 `currentText()`+`std::stoi` 前缀解析传递（panel→backend→ConfigManager 三方隐性约定，脆弱）。

### 5.3 线程安全

- **C1【高】重量级重建在 GUI 线程执行，与 SDK 推送共用同一把实例锁——双线程同步卡死**（`algo_bridge.cpp:138-140` push_events 持锁 / `:178-182` pull_result 持同一把锁；E2VID 推理/Bardow 迭代发生在 pull_result→get_frame()）。live + E2VID 时：GUI 线程做推理期间 SDK 数据线程的 push_events 阻塞 → 事件采集反压，同时整个 Qt UI 冻结。修复：get_frame()/重建移入 worker 线程（push 只入队、pull 取最新完成帧、try_lock 跳帧）；至少将推理移出锁（双缓冲结果）。
- **C2【中】全局 ROI/预处理下发持 `live_mutex_` 执行可能重载 ONNX 的 set_param**（`algo_bridge.cpp:386-440`；`analytics_backends.cpp:62-105` rebuild 内 set_model_path 重载 ONNX）。E2VID 启用时拖动 ROI → 数百 ms~秒级所有 `list_live/find_live` 阻塞 → GUI 冻结。修复：缓存快照后锁外逐个 set_param，或 rebuild 延迟到下一个 push。
- **C3【中】** FramePeriodicBackend 输出回调（SDK 内部线程写 `last_frame_`）与 pull_result 读无同步（`openeb_frame_backends.cpp:287-288,311-313`）——当前不可达（无 GUI 入口），潜在问题。
- **C4【低】** `FilterChain::chain_mutex()` 是进程级静态互斥，所有实例共享（当前只有一个实例，无实际影响）。
- 正面确认：SDK→GUI 的 XYT 投喂走 QueuedConnection；frame_ready/事件窗信号跨线程均走队列；显示策略对 AlgoWindow 的写也排队。未发现直接跨线程触控件。

### 5.4 ROI 传递

- **D1【高】`set_sensor_dimensions` 在 24 个自研后端中仅 8 个实现，换源后 ROI/内部缓冲不更新**。未实现的包括 hot_pixel_filter、optical_gyro、object_tracker、corner_detector、blob_detector、sparse_optical_flow、orientation_filter、direction_selective、background_mask、bandpass、ultra_slow_motion、line_segment、orientation_cluster、cluster_lif、flow_statistics、auto_bias、particle_counter、active_marker、trigger_synced 等。触发：未连相机时在面板编辑参数（实例按 1280×720 创建），然后连接 640×480 相机/文件——`main_window.cpp:727-730` 调 `set_sensor_dimensions+reset`，但对上述后端是 no-op → RoiFilter 区域仍按 1280×720 计算（中心 128×128 → x∈[576,704)），640 宽传感器上只剩右缘一条带通过 → **算法"假死"（静默丢事件而非崩溃）**；`draw_roi_overlays` 画的黄框与后端实际过滤区域不一致 → 视觉误导。修复：RoiFilter 增加 `set_sensor_dimensions`（`roi_.init(w,h)`），持 w×h 缓冲的算法增加 resize；基类注释把该接口从"可选"改为"必须"。
- **D2【中】** FreqDetector/XYTVisualizer 的 set_sensor_dimensions 只更新 roi_ 不更新算法本体（`analytics_extra_backends.cpp:216-218`、`display_backends.cpp:221-223`）。
- **D3【低】** 显示区拖拽设置的是**硬件** ROI，算法 ROI 只能在 AlgorithmsPanel 输数字——两套体系名字相近且都会在主显示画框，用户极易混淆。坐标变换本身经核无误。
- **D4【中】** ROI 参数下发的重建粒度：HoughLine/HoughCircle/TimeSurface/ISI 对任何 roi_* 变化无条件 rebuild()，而 `apply_global_roi` 一次发 5 个 set_param → 最多 5 次重建（累加器状态反复清零）。修复：后端加"尺寸是否实际变化"判断，或 bridge 合并成一次原子 set。

### 5.5 算法切换/复位语义

- **E1【高】flood guard 按"每批 5 万事件"校准，文件播放时一次 push 是整个累积窗 → 误杀**（`algo_bridge.cpp:153-167` kMaxBatchEvents=50000、kFloodStrikes=4；文件路径每次 push 一整个 33ms 窗口，`main_window.cpp:1505-1517`）。文件播放 + ROI 关闭 + 事件率 >1.5Mev/s → 连续 4 帧后算法被自动禁用，且过程中每帧静默丢弃窗口前部事件。live 模式等效 50Mev/s 才触发——对文件模式敏感 33 倍。修复：按事件速率（events/s 滑动窗）而非批大小；或对文件模式单独阈值。
- **E2【中】** flood guard 自动禁用后侧栏复选框仍勾选（内部 `enabled_=false` 不发信号），用户以为算法还在跑；且 statusBar 每帧重复刷 5s 消息。修复：发信号让面板同步取消勾选。
- **E3【中】** 互斥切换不释放旧算法资源：禁用只调 `set_enabled(false)`，E2VID ONNX session、Hough 累加器等由 `live_instances_` 长期持有 → 反复切换大内存算法只涨不跌。修复：增加 `release_resources()` 或禁用超时自动销毁。
- **E4【低】** 已确认良好：seek/loop/换源均 reset 全部 live 实例。两个张力：① seek/loop 的 reset 作用于所有 live 实例，包括用户有意"禁用但保留状态"的，与 pause-resume 语义冲突；② Config 恢复 `enabled=true` 时直接 `set_enabled(true)`——不同步面板 checkbox、不开 AlgoWindow、且绕过互斥（配置里两个 enabled 会同时运行）。

### 5.6 预处理链

- **F1【高】`preproc_downsample` 默认 ON 且全局生效，但只有 5 个后端做坐标减半；其余 ~19 个后端被静默丢弃 75% 事件**（`backend_common.h:266` `halve_coords_` 仅 EventToVideo/ISI/TimeSurface/HoughLine/HoughCircle 为 true；默认注册值 "true" + 面板默认勾选）。`halve_coords_=false` 时只按 x/y 奇偶保留 1/4 事件、**坐标不变**——对检测/跟踪/学习类算法是 4× 的静默输入损耗，无任何界面提示；wiki/Algorithms.md:31 的"halves both dimensions…output upsampled back"对这批后端不成立。修复：downsample 默认改 OFF；或拆两个参数（"抽稀"与"降分辨率"）；至少面板标注"仅抽稀事件"。
- **F2【中】** 预处理参数在面板（`algorithms_panel.cpp:423-462` pdefs）与注册表（`algo_bridge.cpp:261-305`）两处重复定义，手工同步，有漂移风险（当前一致；`rep_averaging_samples` 两边都漏）。
- **F3【低】** undistort 阶段：YAML 加载失败静默清空 K（无用户反馈）；`cv::undistortPoints` 抛异常会一路抛到 MainWindow 的 `catch(...)` 被吞 → 算法静默死亡；`backend_common.h:362-367` "仅 undistort 启用且 LUT 无效"时 `out.assign(out.data(),...)` 自赋值（UB）。

### 5.7 输出路径

- **G1【高】`AlgoResult::filtered_events` 全仓库无消费者——滤波/变换类算法的输出不可见**。hot_pixel_filter、ultra_slow_motion、trigger_synced、overlay、perspective_undistort、optical_gyro（EIS 稳定后的事件）这些以"输出事件流"为唯一产物的算法，启用后主显示毫无变化，只有 AlgoWindow 状态行在变；且每帧白拷贝最多 5 万事件。修复：把启用中算法的 filtered_events 回注显示管线，或移除该字段并文档声明。
- **G2【高】orientation_filter/direction_selective 着色事件错位（默认配置下必现）**（`filter_backends.cpp:61-91,161-183`）：分类针对 **ROI 过滤后**的子序列 `ev[i]` 存入 `last_orientations_[i]`；pull 时却与**未过滤**的 `passthrough_[i]` 按下标配对。着色的像素是原始流前 N 个事件（大多在 ROI 外），颜色标签却属于 ROI 内事件——位置和颜色全错。修复：缓存过滤后事件为成员，pull 时与该缓存配对（参照 HotPixelFilterBackend 回拷模式）。
- **G3【高】trigger_synced 恒零输出：全库无人调用 `add_trigger`**（`trigger_synced_filter.h:71` 是唯一注入入口，grep 确认无调用方；GUI 也没有把硬件 Trigger In 事件接入的通路）。修复：camera_controller 注册 Trigger In 回调转发到该后端；或面板标注"需要外部触发源，当前未接入"。
- **G4【高】HoughCircle 节流直接丢弃整批事件，且注释与代码矛盾**（`cv_vector_backends.cpp:257-262`：距上次处理 <50ms 时 return——注释声称"累加器仍接收事件"，但 `process()` 是累积+峰值扫描一体，提前 return 意味着这批事件根本没进累加器）。后果：20Hz 节流之间的事件全丢；衰减项 dt 被拉长到 50ms+ → `decay_factor=1/(0.0001·decay·dt)` ≈0.2，每处理一次把累加器乘 ~0.2 → 检测能力大幅下降。修复：拆出 `accumulate_only()`：节流只跳过 find_peaks 扫描。
- **G5【中/低】** `main_window.cpp:1806-1808` 把 background_mask 列在 Standalone 分支但其 display_mode 是 Replace——死代码，误导维护者；`mat_to_qimage` 仅接受 1/3 通道 Mat，其他类型静默返回空图 → Replace 模式会黑屏无提示。

### 5.8 错误处理

- **H1【中】E2VID 模型加载失败静默降级**（`e2vid_inference.h:177-187`）：失败仅返回 false → 启发式重建，status 行不显示模型状态；默认路径 `models/e2vid_lightweight.onnx` 是相对路径依赖启动 CWD。用户很容易把启发式粗糙输出误当成"E2VID 效果"。修复：status 追加 `model=loaded/heuristic`；失败经 error_message 提示一次。
- **H2【中】** `apply_strategy` 未包异常（`main_window.cpp:1589-1595`）：策略内 mat_to_qimage/frame.copy 抛 cv::Exception 会逃出 Qt slot。
- **H3【低】** 算法异常被整体吞掉（`main_window.cpp:1375,1520` `catch(...){}`）：算法在 SDK 线程抛异常后无声死亡，无计数、无日志、无界面提示。

### 5.9 桥接待验证疑点

1. hough_circle 衰减公式在 live 小批次（dt≈1ms → factor≈10/批）下是否会吹胀累加器——与 jAER 公式一致但批节奏不同，需实测。
2. `FileFrameGenerator` 全文件事件常驻内存无上限（见 §六 C2）。
3. ultra_slow_motion 输出事件时间戳指向未来——若将来接上 filtered_events 消费端需注意。
4. Config 加载后 AlgorithmsPanel 控件不刷新（只写实例/缓存）——面板显示值与算法运行值脱节（与 B1 同族）。

---

## 六、GUI 功能性问题

### 6.1 录制/播放

- **P1【高】播放中切换文件/相机后，新文件永不自动播放（状态机残留）**（`playback_controller.cpp:119-122,84-117`）：`playing_` 仅在 on_file_eof/pause/set_camera 中清除；播放文件 A 期间打开文件 B（或经 live 切换），`open_file()` 末尾调 `play()` 但 `if (!available() || playing_) return;` 直接返回——B 已加载但不播放，画面静止在第 0 帧，播放按钮仍显示 "Pause"。修复：`open_file()` 在 connect_file 成功后显式 `playing_=false; at_eof_=false;`。
- **P2【中】播放速率超过文件读取速率时误判 EOF**（`file_frame_generator.cpp:164-173`）：`duration_us_` 只是"已缓冲的最大时间戳"，游标追上读取进度即判 EOF（非 loop 停止播放/loop 回绕重播已缓冲片段）。大文件 + 快进参数时用户看到播放到"中间"突然停止。修复：SDK 相机 EOF 时置 `loading_complete` 标志，未完成时游标追上进度应暂停等待。
- **P3【中】文件 EOF 时 `stopped()` 双发，且播放中状态栏被误置 "Idle"**（`camera_controller.cpp:245-254` + SDK status 回调也会发 STOPPED）：(a) stopped() 发射两次（下游幂等无实质损害）；(b) `main_window.cpp:811-823` 的 stopped lambda 不区分文件源，把状态栏置 "Idle"——但 FileFrameGenerator 播放仍在继续，与 PlaybackController 的文件源豁免不一致。修复：stopped lambda 加 `if (camera_.is_file_source()) return;`。
- **P4【低】** `FileFrameGenerator::play()` 的 EOF 自重启路径不发射 seeked/looped（当前唯一调用方已补发，API 陷阱）。
- **P5【低】** Step 按钮可 seek 越过文件末尾（seek 只钳下界不钳上界），再按 Play 被当 EOF 从头重播。
- **P6【低】** 统计面板 "Display FPS" 在文件模式按事件时间戳差计算，慢放时显示 10000 fps 等失真数值（应改 wall-clock 或直接显示设定 fps）。

### 6.2 导出

- **E1【严重】AVI 导出使用 `process_all_frames=false` 的 CDFrameGenerator——绝大多数帧被跳过、尾部事件被丢弃**（`exporter_controller.cpp:200`，第三参数缺省 false）。SDK 源码确认两重丢失：① `cd_frame_generator.cpp:96-99` 每批缓冲只生成最后一帧，中间帧全部跳过 → 输出视频时长被严重压缩；② `stop()` 走 abort()，`events_back_` 未处理事件整体丢弃 → 视频结尾缺失。修复：显式传 `CDFrameGenerator(w, h, /*process_all_frames=*/true)`。**注意**：修复后编码慢于读取时 `events_back_` 无界堆积，长文件导出内存增长显著，需配合限速喂入（按消费进度节流或分块 seek 导出）。
- **E2【低】** 导出/转换/裁剪不检查源路径==输出路径 → 可覆盖正在读取的源文件（数据损坏）。修复：比较 canonicalFilePath 并拒绝。
- **E3【低】** `ExporterController::start()` 的 worker 只 catch `std::exception`（对照 FileConverter 有双层 catch）——非 std 异常 → std::terminate。
- **E4【低】** 取消导出/转换后半成品文件残留且无"文件不完整"提示。

### 6.3 相机控制

- **C1【中】过期的 EOF/错误 lambda 会停掉新连接的相机**（`camera_controller.cpp:248-254,258-264`）：源 A 的 runtime error 触发 QueuedConnection lambda；执行前用户已连接源 B 并 start → lambda 内 `if (!camera_) return;` 通过（此时已是 B）→ **stop 源 B 并 emit stopped()**。修复：err_cb 内捕获 `Metavision::Camera* cam`，lambda 开头比较 `if (camera_.get() != cam) return;`。
- **C2【中】文件模式事件缓冲无上限——大文件 OOM**（`file_frame_generator.cpp:22-37` 全量驻留）：GB 级 RAW（数亿事件×16B）可耗尽内存；打开期间无进度/容量提示。修复：预估事件数超阈值时拒绝加载并提示，或回退分段缓冲方案。
- **C3【中】** 连接失败/打开文件失败出现连续两个错误弹窗（camera_controller emit error → main_window 弹一次；playback_controller 返回 false → main_window 再弹一次）。
- **C4【低】** `frame_pipeline_.start()/start_file()` 失败仅状态栏提示，连接流程照常完成——状态栏 Connected 但画面永远黑屏。
- **C5【低】** 录制允许在相机未 streaming 时开始 → 录出空文件。

### 6.4 标定

- **B1【中】`CalibrationEventTap::attach()` 重复 connect——每开关一次向导，事件重复注入一份**（`calibration_event_tap.cpp:28-40`；每次打开向导都调 `set_camera→attach`）：第 N 次打开后同一批事件被 buffer_.insert N 次（DirectConnection 同步执行 N 次），缓冲增长/窗口计数全部 ×N。修复：attach 先 disconnect，或 connect 加 `Qt::UniqueConnection`。
- **B2【中】标定默认导出路径目录不存在，"Export" 必失败**（`calibration_wizard.cpp:363-370`）：首次使用接受默认路径即踩中（与 Preprocessing 面板的默认 undistort 路径是同一文件）。修复：写前 `QDir().mkpath(...)`。
- **B3【中】`intrinsic_->run()` 在 GUI 线程执行**（:326-343）：30 帧采集后 `cv::calibrateCamera` 冻结应用数秒到数十秒。修复：QtConcurrent::run + QFutureWatcher。
- **B4【低】** `QApplication::processEvents()` 重入可重置正在运行的标定（:329-331：processEvents 期间用户点 Start/Reset → 外层 run() 读到被 reset 的空状态）。
- **B5【低】** ChessboardDisplay 窗口模式棋盘绘制位置错乱（recompute_geometry 按屏幕 availableGeometry，paintEvent 用 widget 坐标）——非全屏时大部分棋盘落到窗口外。
- **B6【低】** `attached_screen_` 裸指针，屏幕热拔后悬空（改 QPointer<QScreen>）。

### 6.5 显示

- **D1【低】** paintGL/draw_letterboxed 对 0 高度窗口无防护（受 setMinimumSize 保护不可达，健壮性缺口）。
- **D2【低】** ROI 拖拽映射 off-by-one：clamp 到 `img_w−1` → 满幅拖拽得到 `w=img_w−1`，最右/最下一列永远不会被包含。
- **D3【低】** SpaceTimeDisplay shader link 失败无回退检查（对照 EventDisplayWidget 有检查）。
- **D4【低】** draw_axes_overlay 投影点坐标为 inf 时仍尝试绘制（加 `std::isfinite` 检查）。

### 6.6 面板/控制

- **U1【低】** Biases 滑块滚轮/键盘调整永不写入硬件（apply 仅挂在 `sliderReleased`）：UI 显示值已变但硬件未更新。修复：连 `actionTriggered` 或 valueChanged 去抖 apply。
- **U2【低】** Anti-Flicker low/high 编辑期间连续弹模态错误框（每次 valueChanged 校验 → QMessageBox::warning）。改 editingFinished 校验或降级为状态栏提示。
- **U3【低】** Trigger Out period 直接编辑失败时 UI 值不回滚（BiasesPanel 有 refresh_row_values 回滚，此处没有）。
- **U4【低】** EspPanel populate 中 `setValue` 先于 `setRange`，极端值被旧 range 钳位后显示错误值。
- **U5【低】** Algo params JSON 未校验，`std::stoi` 崩溃路径（`main_window.cpp:1413,1547,1910` 三处无保护；手工编辑的畸形 JSON → `std::invalid_argument` 逃逸 → terminate）。
- **U6【低】** `FileConverter::info()` 在 GUI 线程同步打开大文件（无 .tmp_index 时冻结数秒）。

### 6.7 启动/退出

- **M1【中，待实机验证】** ResizeGrip 创建后未立即布局（`main_window.cpp:144-149` 创建 vs `:244-251` reposition 仅在 resizeEvent 中）：若 xcb show 后不派发 resizeEvent，8 个 grip 保持默认 geometry (0,0,100,30) 且 raise()——覆盖标题栏 File 菜单区域。修复：创建循环结束后立即 reposition 一次（一行）。
- **M2【低】** 无单实例机制（双开同时写同一录制路径无防护）。
- **M3【低】** 退出时若 sensor self-test 窗口开着，关闭流程被模态报告框阻塞。
- **M4【低】** ExportDialog 的 Close 不取消后台导出（无取消途径，仅 UX）。
- 复核确认无问题：teardown 的回调移除→stop→reset→pipeline stop 顺序消除了 UAF 竞态；FilterChain 互斥；FrameGenerator/CDFrameGenerator 生命周期；EventDisplayWidget GL 资源清理；FileConverter/ExporterController 的 cancel/join。

### 6.8 GUI 待验证疑点

1. M1 ResizeGrip 初始位置（实机确认，若成立升至高）。
2. `Camera::stop()` 的 join 语义未在 SDK 源码逐行确认（`I_EventDecoder::remove_callback` 不等待在飞回调——若 stop 不 join，teardown 存在理论竞态窗口）。
3. E1 修复后长文件导出的内存峰值需实测。
4. `PeriodicFrameGenerationAlgorithm` 在 `process_all_frames=true` 时对灌入式事件流的逐帧生成成本未实测。
5. `FileFrameGenerator` 假设事件严格按 t 排序：`NonMonotonicTimeHigh` glitch 若真让 t 倒退，`lower_bound` 窗口切片会漏事件。

---

## 七、跨主题共性问题

### 7.1 默认值四方漂移（jAER / algo 成员默认 / GUI 注册 / design.md）

这是最系统的质量问题。典型例子：
- DWF 窗长：jAER 512 / C++ 成员 512 / GUI 注册默认 **2、上限 100**（用户无法恢复设计工作点）。
- cluster_lif：jAER tau 22000us/threshold 15 / C++ 成员 22ms（即 22000us 换算）/ GUI 注册 tau 10、threshold 1.0。
- AgePolarity tau：jAER 25000 / C++ 10000 / design 3000。
- Harmonic f0：jAER 100 / C++ 50（注释声称对应 jAER）。
- object_tracker mass_decay_tau：jAER 10000 / C++ 成员 10000 / GUI 注册 100000。
- design.md §4.3.5 多处默认值（STCF 5ms、BAF 1ms、DWF 2/2/2/false、AgePol 3000/2.0/2、Q=5、ratio 10/10、SBP 10ms）与 algo/ 成员默认值不一致。

**建议**：建立单一真源——算法类成员默认值对齐 jAER 或 design；GUI 注册表只覆盖"刻意调参"的项并在 design.md 中逐条登记理由；CI 加一个"注册默认值 vs 算法默认值"一致性测试。

### 7.2 移植注释失实（文档可信度）

`optical_gyro.h:349` "(jAER RCT defaults)"（5/6 参数不符）、`cluster_lif.h:21` 初始膜电位注释、`sparse_optical_flow.h:337` 点名 jAER 差分器却多 0.5 因子、`object_tracker.h:8` 与 `line_segment_detector.h:3` 的"✅ 移植"夸大、`main_window.cpp:421` frame mode 注释失实。**建议**：对所有 "✅ 移植自 jAER X" 注释做一轮真实性普查，改为三档标注：`faithful`（逐项核对一致）/ `adapted`（列出差异清单）/ `inspired`（仅借鉴思想），差异清单与 Java 行号放在头注释。

### 7.3 "调了没反应"的用户可见参数

n_sigma、min_radius、accumulator_decay_us（×2）、max_points、min_events、orientation_threshold、rep_period/rep_tolerance、histogram_window_s、pass_all_events、multi_ori_output、subsample_by（语义不符）、decay_tau_ms（语义无效）——**建议**：全部从注册表移除或接通，并在 test_algo_bridge.cpp 增加"每个注册参数必须可观测生效"的契约测试。

### 7.4 时间戳哨兵不统一

0（time_surface/corner/direction_selective/hot_pixel learn_start）、-1（lif_integrator）、INT_MIN（jAER）、`prev_batch_t_` 初值 0（object_tracker M1）。大时间戳源（实况相机 t≈1e9us、裁剪回放）下多处首包行为错误。**建议**：全库统一 -1 哨兵 + 首事件初始化锚点的惯用法。

---

## 八、修复优先级路线图

**P0（立即修，小改动大收益）**
1. §五-A1 corner mode 枚举错位（一行注册表）。
2. §六-E1 AVI 导出 `process_all_frames=true`（一个构造参数；连带评估节流）。
3. §四-S1 ObjectTracker 速度低通 + 外推 clamp（默认配置跟踪失效，且有测试注释掩盖）。
4. §五-G2 orientation/direction 着色错位（默认配置必现的显示错误）。
5. §六-P1 播放中切文件 playing_ 残留（两行状态重置）。
6. §三-31/32 + §五-A3 "调了没反应"参数全链路清理。
7. §六-B1 CalibrationEventTap 重复 connect（加 UniqueConnection）。
8. §五-B2 background_mask "learning_rate" 误接（重命名+默认值 5.0）。

**P1（本迭代修）**
9. §五-F1 downsample 默认值改 OFF 或拆分参数。
10. §五-D1 set_sensor_dimensions 补齐 16 个后端（换源假死）。
11. §五-E1 flood guard 改按事件速率。
12. §五-G1/G3 filtered_events 无消费者 + trigger_synced 触发源：接通或下架（二选一，含 §三-B6~9 的 1000 行死后端处置）。
13. §五-G4 HoughCircle 节流拆 accumulate_only。
14. §一-1.2 direction_selective ori 路径 90° 错位（接 ori 流水线前必须修）。
15. §一-1.3 Repetitious min_dt 分支语义。
16. §四-M1~M7 算法中层 bug。
17. §一-1.1 cluster_lif 初始膜电位取舍 + 注释修正。
18. §六-C1 过期 lambda 停新相机（捕获相机指针比较）。
19. §六-B2/B3 标定导出目录 + 标定移出 GUI 线程。

**P2（计划修，架构性）**
20. §五-C1 算法 worker 线程化（E2VID/Bardow 移出 GUI 线程与实例锁）。
21. §六-C2 大文件缓冲上限/分段方案。
22. §七-1 默认值单一真源 + 一致性测试。
23. §七-2 移植注释三档标注普查。
24. §三 死代码清理（32 处确定项）。
25. design.md/wiki 与代码同步（§4.3.17、§4.3.5 默认值表、Algorithms.md 的 59 算法口径、frame mode 注释）。

**潜伏 bug 备忘（接线/启用前必须修）**：§四-M3 E2VID 掩码坐标错位、§四-M4 AsymmetricCircles 物点网格、§一-1.2 ori 90° 错位、§三-8 perspective_undistort 构造参数绑定错误、§四-低6 corner num_orientations>4 错位。

---

## 九、标定工具与锐度工具专项：现状评估与优化思路

> 针对提交 caee2c0（闪烁棋盘标定 + 锐度计 + 去畸变预处理）的专项评估。本节取代 `.trae/documents/` 下两份计划文档（已删），作为该功能的权威问题清单与优化路线。证据均标注文件:行号。

### 9.1 需求实现对照

| # | 需求 | 状态 | 说明 |
|---|---|---|---|
| 1 | 标定工具放在 Tools 菜单 | ✅ | Tools → Intrinsic Wizard（main_window.cpp:1653） |
| 2 | 按屏幕尺寸/分辨率绘制标准棋盘 | ⚠️ | 尺寸按 `availableGeometry` 计算，但坐标系错位（9.2-B）；DPI 不可靠（9.2-G） |
| 3 | 棋盘 20Hz 闪烁 | ⚠️ | 定时器是 50ms，但全屏下不能正确闪烁（9.2-B 根因） |
| 4 | 1ms 窗口累积 + 自动抓拍 | ✅ | kWindowUs=1000（calibration_wizard.cpp:39） |
| 5 | 每 50ms 取 50 个候选中事件最多的一张，低复杂度 | ⚠️ | O(N) 单遍已实现（calibration_event_tap.cpp:88-104），但窗口锚定有相位缺陷（9.2-F） |
| 6 | 检测不出网格 → 弃图 | ⚠️ | 已实现，但检测在噪声原图上做，无去噪（9.2-E） |
| 7 | 与最近合格图 MSE 过高 → 弃图 | ❌ | MSE 查重被 20Hz 翻转机制击败（9.2-D），静止相机也能刷满 30 帧 |
| 8 | 进度条随合格帧更新 | ⚠️ | 已实现但全屏时不可见（9.2-C） |
| 9 | 够数量自动结束 + 计算 + 弹窗导出（带默认路径） | ⚠️ | 已实现；但有目录不存在必失败、GUI 线程标定冻结等问题（§六-B2/B3/B4） |
| 10 | Tools 锐度工具 | ❌ | 已实现但度量错误，不能反映线条锐利程度（9.3） |
| 11 | 侧栏去畸变复选框 + 默认路径一致 + 顺序在滤波/降采样后 | ✅ | backend_common.h:283+；默认路径已对齐（calibration_wizard.cpp:363-370）；失败路径无声（§五-F3） |
| 12 | 完善文档并提交不推送 | ✅ | caee2c0 已含 design.md/wiki 更新 |

### 9.2 标定工具问题分析

**A. 极性策略：不能"忽略极性"——当前极性分离渲染是必须保留的，真正要修的是极性失衡与噪声**

需求方提出"标定程序的事件应该忽略极性"。经分析该方向不可行，论证如下：

- 事件相机没有绝对灰度，棋盘图案只能由"翻转时哪些像素变亮（ON）、哪些变暗（OFF）"来编码。全板翻转时**每个格子的每个像素都会发放**（LCD 黑白对比度远超阈值），若渲染时忽略极性（所有事件一律点亮），所有格子同时变亮 → 得到的是**实心矩形**，棋盘角点在信息层面不存在，`findChessboardCorners` 无解。
- 当前实现（calibration_wizard.cpp:417-427：bg=128、ON=255、OFF=0）在单次翻转的 1ms 窗口内直接重建出新棋盘状态（转白的格子→白点，转黑的格子→黑点），这是单帧重建棋盘的**唯一**途径，必须保留。
- "只需要白色块的黑白交替"若理解为"每个格子黑白交替"（= 全板取反），即当前实现，正确；若理解为"只让白色格子闪烁、黑色格子恒黑"，则每次翻转只有半数格子发放，事件图只是对角半网格（格子仅角点相接），不构成棋盘，不可行。
- 真正的痛点有两个，应在保留极性分离的前提下解决：
  1. **极性增益失衡**：相机 ON/OFF 阈值不对称时，一种极性的事件远少于另一种 → 重建图中一类格子稀疏甚至缺失 → 检测失败。优化：渲染前统计窗口内 ON/OFF 计数，若比例失衡（如 <1:3），对稀疏极性做增益补偿（复制稀疏极性事件或按极性分别做形态学闭运算）；或对 ON、OFF 两幅单极性图分别闭运算后再合成。
  2. **噪声污染**：热像素/BA 噪声在 1ms 窗口内以随机极性出现，在白格上打黑洞、黑格上打白洞。优化见 9.2-E。

**B. 全屏下"不会正确闪烁"的根因：GUI 线程饱和（主因）+ 几何坐标错位（次因）**

- **主因——所有耗时工作都在 GUI 线程**：翻转定时器（chessboard_display.cpp:33-45）、抓拍定时器（calibration_wizard.cpp:66-69）、`findChessboardCorners`（经 `intrinsic_->add_frame`，calibration_wizard.cpp:300）、棋盘绘制，全部跑在 GUI 线程。`findChessboardCorners` 在 640×480~1280×720 含噪图上单次耗时 10~200ms，每 50ms 调一次 → GUI 线程被打满，翻转定时器饿死（QTimer 超时在同一事件循环里合并），屏幕上的棋盘就表现为停顿、乱跳、不闪烁。全屏放大了这个效应：每次翻转的 fillRect 面积随棋盘变大（chessboard_display.cpp:122,138），绘制本身也加重 GUI 负担。
- **次因——几何与坐标系错位**：`recompute_geometry` 用 `attached_screen_->availableGeometry()`（屏幕坐标系）计算 origin，但 `paintEvent` 在 widget 坐标系绘制（chessboard_display.cpp:74-101,134-136），且没有把 `availableGeometry().topLeft()` 加回去——有顶/左面板时棋盘整体偏移；窗口模式（按 F 退出全屏）下 widget 只有 800×600 而棋盘按全屏尺寸画 → 大部分棋盘在窗口外（即 §六-B5）。几何应按 `resizeEvent` 时的 widget `rect()` 计算，两个模式都自然正确。
- **修复思路**：
  1. 把 drain+渲染+`findChessboardCorners` 移入 worker 线程（QThreadPool/QThread），GUI 线程只保留翻转定时器与进度显示；接受帧经信号回投。翻转与绘制不再被检测阻塞，20Hz 稳定。
  2. 棋盘几何改为响应 `resizeEvent` 按 widget `rect()` 重算；全屏/窗口两模式统一。
  3. 翻转后如需更强实时性，可用 `repaint()`（同步）替代 `update()`，避免事件循环排队延迟。
  4. （可选）棋盘改用 QOpenGLWidget 或 `WA_StaticContents` + 双缓冲 pixmap，消除全屏部分更新在 compositor 下的撕裂风险。

**C. 全屏时看不到进度条/按钮 → 把控件搬到棋盘窗口（采纳需求方方案）**

- 现状：棋盘 `showFullScreen()`（calibration_wizard.cpp:220）盖住同屏的向导对话框，进度条、状态、按钮全部不可见。
- **方案**：在 ChessboardDisplay 内嵌半透明 HUD（棋盘区外的黑边处）：进度条（N/30）、状态行（检测失败原因）、Start/Stop/Reset 按钮、square_mm 显示；快捷键 S 开始/停止、Esc 关闭。向导对话框保留为配置入口（屏幕选择、行列数、目标帧数、MSE/位移阈值）。
- **注意**：HUD 自身变化（进度条前进）会产生事件被相机看到——HUD 必须放在棋盘区外的边缘黑带、面积尽量小、只在接受帧时更新（低频）；检测端把 HUD 所在屏幕区域对应的事件丢弃意义不大（相机视野未必对准它），依靠"棋盘居中、HUD 贴边"即可，文档中提示用户取景对准棋盘。

**D. MSE 查重被 20Hz 翻转机制击败（设计级缺陷）**

- 相邻合格帧分别处于棋盘的两个相反相位：第 k 帧是棋盘状态 S，第 k+1 帧是 ¬S（全板取反）。`mse_gray(S, ¬S)` 每个格子都差 255 → MSE 极大（calibration_wizard.cpp:308-315），必然通过"非重复"检查。结果：**相机静止不动也能把 30 帧刷满**，且全部是同一视角的两个相位副本 → 标定视角覆盖严重不足、标定条件数差。这是当前工具"不算成功"的核心功能 bug。
- **修复**：改用**角点位移查重**——`add_frame` 已经检测出 9×6=54 个角点，直接比较本次与上次合格帧对应角点的平均/中位位移（如 <2px 判重复）。优点：对翻转相位天然免疫（两种相位下角点位置相同）、直接度量视角多样性（标定真正需要的多样性）、计算量比全图 MSE 低三个数量级。注意处理角点顺序对齐（同板同序即可，OpenCV 对非方阵棋盘返回顺序稳定）。MSE 可保留为辅助，但必须先对其中一帧取反再比（`min(mse(a,b), mse(255-a,b))`）。

**E. 检测鲁棒性：原始含噪渲染直接送检**

- 当前链路：1ms 窗口原图（椒盐噪声 + 热像素）→ `findChessboardCorners`（calibration_wizard.cpp:299-300）。失败率高，表现为反复 "Rejected — chessboard not detected"。
- **修复**（按收益排序）：
  1. 渲染后做 3×3 中值滤波或形态学开运算（cv::medianBlur 足够便宜），消除孤立噪声点；
  2. 复用项目已有的热像素屏蔽（hot_pixel_filter 的掩码概念）：标定会话开始时统计一段"非翻转期"内高频发放的像素，渲染时置灰；
  3. 换用 `cv::findChessboardCornersSB`（对噪声/低对比显著更稳）；
  4. 大传感器（1280×720）先 1/2 降采样再检测，角点坐标 ×2 还原——检测提速 4 倍（也缓解 9.2-B 的线程压力）；
  5. 极性失衡补偿（见 9.2-A）。

**F. 抓拍相位缺陷：50ms 切片与翻转相位无对齐**

- 切片锚定在"缓冲区首个事件时间"（calibration_event_tap.cpp:76-77），翻转锚定在墙钟定时器——两者无同步关系，相位固定但任意。若翻转爆发恰好横跨切片边界，则每片各得半个爆发，max-event 窗口强度减半（稳定相位下**永远**减半）。
- **修复**：放弃固定切片，在整个 50ms 缓冲上做滑窗最大值搜索——按 100µs 细桶计数（500 桶）+ 长度为 10 的滑动和取最大（O(N)+O(桶数)，仍是低复杂度），允许 1ms 最优窗口落在任意偏移，跨边界爆发也能完整捕获。或进一步：由向导统一驱动翻转与抓拍（翻转后 25ms 触发 drain），相位严格对齐。

**G. square_size_mm 依赖不可靠的 DPI**

- `QScreen::physicalDotsPerInch()`（chessboard_display.cpp:95-101）在 X11 上常返回 96 或错误的 EDID 值 → 格子物理尺寸错误。影响范围：只影响平移向量的米制尺度（fx/fy/cx/cy/畸变系数不受影响，去畸变功能不受影响）。
- **修复**：界面上把计算值显示为**可编辑** spinbox（用户用尺子量一个格子直接填），默认取 DPI 计算值。

**H. 已在本报告其他章节列出的关联问题（不重复展开）**

- §六-B1 `CalibrationEventTap::attach()` 重复 connect（每开关一次向导事件重复注入 N 份）——**必须随本轮一起修**（加 `Qt::UniqueConnection`）。
- §六-B2 默认导出目录不存在 → 首次 Export 必失败（写前 `mkpath`）。
- §六-B3 `cv::calibrateCamera` 在 GUI 线程跑（冻结数秒~数十秒）——与 9.2-B 一并移入 worker。
- §六-B4 `processEvents()` 重入（calibration_wizard.cpp:330）。
- §六-B6 `attached_screen_` 裸指针（改 `QPointer<QScreen>`）。
- §三-28 intrinsic LUT 死链（顺带清理）。

### 9.3 锐度工具失败分析

当前实现（sharpness_dialog.cpp:250-323）：10Hz 轮询 `EventDisplayWidget::current_frame()` → 转灰度 → Laplacian 方差 σ²。**它不能反映累积事件帧的线条锐利程度**，原因有四，按严重程度排序：

- **R1（致命）噪声主导度量**：事件帧是稀疏图（大面积均匀背景 + 稀疏边缘）。孤立噪声点（热像素）在均匀背景上产生最大可能的 Laplacian 响应（|L|=4×255），其贡献与真实边缘同量级甚至更大 → σ² 主要反映**噪声密度**而非边缘锐度。用户调 bias 降噪声时 σ² 下降（看似"变糊"），噪声升高时 σ² 上升（看似"变锐"）——指示方向与直觉相反。
- **R2 事件密度与锐度混淆**：σ² ∝（强梯度像素数）×（幅度²）。累积窗口加大、事件变多时，边缘更密 → σ² 单调上升，与线条是否锐利无关；"密集但模糊"与"稀疏但锐利"无法区分。
- **R3 测量对象被污染**：`current_frame()` 是**渲染后的显示帧**——(a) 调色板映射（4 种 palette）改变灰度对应关系，跨 palette 数值不可比；(b) 主显示帧被 `draw_roi_overlays` 画了 ROI 黄框+文字（main_window.cpp:1601→1646-1649），算法启用时还有算法标注（框、光流向量）——这些人造强边缘直接灌入 σ²；(c) 8-bit 饱和渲染丢失了"每像素事件计数"信息（1 个事件和 10 个事件都是 255），而计数集中度恰恰是锐度的核心信号。
- **R4 无双边缘结构感知**：事件帧中一条物理边缘是 ON/OFF 两条平行线，RGB→灰度后剖面不是阶跃，Laplacian 对该结构的响应与离焦程度的关系很弱。

**优化思路（按推荐顺序）**：

- **S1 换数据源**：不在显示帧上测量，改为从事件流直接构建**计数图**（count image）：复用 `CameraController::cd_events_ready` 广播（SharpnessDialog 自带一个 100ms 累积窗，极性忽略、不去饱和——每像素计事件数）。计数图无调色板/标注/饱和污染，且保留密度信息。
- **S2 先去噪**：计数图上做 3×3 中值滤波，或丢弃"计数=1 且 8 邻域为空"的孤立像素（极便宜，专杀热像素）；也可复用预处理链的 BAF。去噪后再算任何梯度类指标。
- **S3 主指标改用归一化对比度（CMax 风格）**：σ²(I)/μ(I)²（变差系数平方）。这是事件视觉对比度最大化框架（Gallego et al. CMax）的标准锐度代理：边缘越锐利，事件越集中于少数像素 → 对比度越高；对事件率/窗口长度一阶不变（除以均值平方）。配 log 坐标图表。
- **S4 辅助指标：平均线宽（直接回答"线条锐利程度"）**：计数图阈值化得边缘二值图 → 距离变换 → 边缘像素处距离均值×2 = 平均线宽（px）。离焦/运动模糊 → 线变宽；对焦准确 → 线窄。该指标物理意义直观，与 S3 互补（S3 越高越锐，S4 越低越锐）。
- **S5（可选）Tenengrad-per-edge**：Sobel 能量仅在超过自适应阈值（如中位梯度×3）的像素上取平均，避免背景稀释——比全局 σ² 更贴近"边缘处的锐度"。
- **S6 面板设计**：同时显示 事件率（参考）+ S3 对比度 + S4 线宽 两条曲线；图表 Y 轴按近期分位数自适应或 log 坐标；标注"调对焦看线宽下降，调 bias 先看噪声不干扰对比度"。
- **验证方法**：固定三脚架对准高对比印刷品，转动对焦环，S4 线宽应先降后升、S3 对比度先升后降，且在焦点处同时取极值；再调 refractory/deadtime 改变噪声水平，两指标波动应 <10%（验证 R1 已消除）。

### 9.4 实施路线（建议顺序与验收）

- **阶段 1（正确性，小改动）**：D 角点查重替换 MSE；§六-B1 UniqueConnection；§六-B2 mkpath；G 可编辑 square_mm；9.2-E 的中值滤波 + findChessboardCornersSB。验收：静止相机无法刷帧；移动相机时进度条单调前进；首次导出成功。
- **阶段 2（线程架构）**：drain/渲染/检测/标定全部移入 worker 线程；棋盘几何按 widget rect 重算（修 9.2-B 与 §六-B5）。验收：全屏 20Hz 稳定闪烁（秒表目测或用相机事件率验证翻转爆发间隔 50±5ms）；检测期间 UI 不卡。
- **阶段 3（HUD）**：进度/状态/按钮内嵌棋盘窗口（9.2-C）；相位对齐滑窗（9.2-F）。验收：全屏下无需回看主窗口即可完成全流程；跨边界爆发不再减半。
- **阶段 4（锐度计重写）**：S1→S2→S3/S4。验收：按 9.3 验证方法，离焦单调性与抗噪性达标。
- **阶段 5（收尾）**：§三-28 死链清理、design.md §4.5 与 wiki 更新、补测试（tap 滑窗、角点查重、计数图指标的单测）。
