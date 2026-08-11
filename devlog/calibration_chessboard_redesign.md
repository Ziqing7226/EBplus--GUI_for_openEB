# 标定重设计：Zhou's Screw-Head Grid（十字螺丝头点阵 + 圆环几何验证）

> 已实现。弹窗标题括号内为 `Zhou's Screw-Head Grid Method`（"Intrinsic Calibration (Zhou's Screw-Head Grid Method)"）。

## 概述

沿用非对称 6×5 网格布局（30 位置，朝向内置消歧），每个标记为"螺丝头" = 虚线十字（定中心）
+ 外层实心薄环（圆环几何验证源）。检测：十字定中心 → 圆环局部 Hough 优化 + 角度覆盖率验证 →
非对称网格加权拟合。全程局部 → 旋转/畸变鲁棒。

## 标定板

- 30 标记，位置 `(2c+(r&1), r)·S`（S=半格；用户测量同行/同列相邻标记圆心距=2S，算法 `S=测量值/2`，无√2）。
- 每标记：实心白环（外 `R+g/2`、内 `R-g/2`，厚度 `g`）+ 虚线十字（1px 点、周期 `1+g`、臂长 `L=R_in-g-1`）。
- `R = 0.20·S`（0.30 时外环与标定板上下边界相切，0.20 留出间距；现场可调），`g = dot_gap`。
- 渲染：QPainter-on-QPixmap + `cache_dirty_`（沿用 D11 性能路径，禁 QImage 全图上传）。

## 检测（四阶段，十字/圆环同级联合定中心）

1. **累积** 200–20000µs 事件 → **三值彩色帧**：背景黑、ON=金、OFF=白；同位置多极性事件取金与白的**简单平均**。
2. **十字定中心**：8 邻域去孤立点 → 各向同性膨胀 + 连通域 + 面积过滤 → distance-transform 峰值 = 十字中心候选。
3. **圆环局部 Hough + 覆盖率验证**（替代旧的极性半圆验证）：
   - 在十字候选 ±2px 邻域内搜索使径向直方图峰值最大的圆心 = 圆环中心（局部 Hough 优化）。
   - 收集环像素（峰值半径 ±2px），按极角分 36 bin，计算角度覆盖率（有像素的 bin 占比）。
   - 覆盖率 ≥ `kRingCoverFrac`(0.60) = 检测到圆环。
   - **放弃极性半圆验证**：相机运动时圆环外沿/内沿在每个角度极性相反（leading:外ON/内OFF；trailing:外OFF/内ON），
     旧的"一半全ON、一半全OFF"模型不成立。极性仅用于着色，验证改为纯几何（环信号强度 + 角度覆盖率）。
4. **加权 RANSAC 网格拟合**（无空缺 slot 恢复）：
   - 用十字+圆环联合标记（双特征）的环半径一致性过滤 + 最近邻对角步长估网格半格间距 d；
   - **置信度模型**：十字 50% + 圆环 50% + 8 邻域 20%（见下节），RANSAC 加权搜索最优 6×5 块；
   - **无空缺 slot 二阶段环搜索**（已删除）：所有位置必须有匹配的检测特征（十字或圆环），
     不允许凭空补全，也不在预测位置做二次环搜索——空缺直接导致拟合失败。
   - 最终按 (y, x) 排序解决 8 重方向歧义。

### 联合定中心 + 置信度模型（十字/圆环同级）

- **十字中心**：distance-transform 峰值（每个螺丝头组件中最厚的点 = 十字交叉）。
- **圆环中心**：局部 Hough 优化（十字候选邻域内使环信号最强的位置）。
- **同级检测**：十字检测和圆环检测是同级别的检测——检测到十字或检测到圆环都说明找到了可能的中心。
- **置信度贡献**：
  - 十字检测 → 坐标贡献 50% 置信度（`kCrossConfidence` = 0.5）。
  - 圆环检测 → 坐标贡献 50% 置信度（`kRingConfidence` = 0.5）。
  - 最终中心的 8 邻域有前景信号 → 贡献 20% 置信度（`kNeighborConfidence` = 0.2）。
  - 双特征 + 邻域支持 = 1.2；仅十字 + 邻域 = 0.7；仅圆环 = 0.5。
- **最佳中心**：有环时用环中心（局部优化更准），仅十字时用十字中心。
- **RANSAC 加权**：权重高的中心（双特征）锚定网格，权重低的（单特征）填补缺口。

### 为什么放弃极性半圆验证

旧设计假设：环边在平移下 leading/trailing 半圆极性相反、各半一致；噪声随机交错→拒。
实测发现：相机运动时圆环**无论哪一侧都有两种极性同时存在**——leading 方向外沿 ON/内沿 OFF，
trailing 方向外沿 OFF/内沿 ON。这是四象限相反结构，不是半圆一致结构，所以 half/half 测试
对真标记也会失败。新设计用环的角度覆盖率（几何特征）替代极性验证，运动无关。

### d 值估算（鲁棒，详见下方"d 值估算"小节）

- 优先用十字+圆环联合标记（双特征，最可靠）的环半径中位数 r_med 过滤噪声半径（[0.5, 2.0]×r_med）。
- 用半径估 d_radius 作为 NN 带通滤波的参考（详见下方"d 值估算（鲁棒，v3 NN 带通滤波）"小节）。
- 网格间距 = |s|×√2（s=对角步长），与 d 无关——d 仅影响匹配容差和步长过滤。

## 参数

- `dot_gap`：1/2/3，**默认 2**。
- 捕获窗口：200–20000µs，步进 1µs（任意整数），**默认 5000µs**。
  - 200µs = 稀疏环信号下限；5000µs = 默认（足够收集密集环）；20000µs = 慢运动场景上限。
  - 检测器用几何验证（非极性），窗口可跨多个运动方向。
- 标记间距：用户测量同行（或同列）相邻两标记圆心距，算法 `d = 测量值/2`（无√2）。
- 三值帧：黑/金/白，多极性事件取金白简单平均（不加权）。
- 圆环实心，十字虚线。
- hint（英文）：`Hold the camera steady; rely on hand micro-tremor to trigger events. Press Space to capture.`

## 检测调参常量（screwhead_detect.cpp）

| 常量 | 值 | 含义 |
|---|---|---|
| `kRingAngleBins` | 36 | 角度分辨率（bin 数），越高越严格 |
| `kRingCoverFrac` | 0.60 | 接受环的最低覆盖率——用户要求≥0.6 |
| `kMinRingPixels` | 6 | 最小环像素数 |
| `kRingSearchMargin` | 12 | 环搜索半径余量（加到十字组件半对角线上） |
| `kSeedSearch` | 2 | 局部 Hough 种子搜索半径（±2px） |
| `kCrossConfidence` | 0.5 | 十字检测的置信度贡献（50%） |
| `kRingConfidence` | 0.5 | 圆环检测的置信度贡献（50%） |
| `kNeighborConfidence` | 0.2 | 最终中心 8 邻域有信号的置信度贡献（20%） |
| `kRingRadiusFrac` | 0.20 | R = 0.20·d（与渲染一致） |
| `kMatchTolFrac` | 0.40 | 网格匹配容差 = 0.40·d |

## 代码改动

### 新增

| 文件 | 内容 |
|---|---|
| `algo/calibration/screwhead_detect.{h,cpp}` | `detect_screwheads(color_frame, ...) → (found, vector<Point2f>)`。四阶段管线；输入三值彩色帧（CV_8UC3），前景=非黑 |
| `algo/tests/test_screwhead_detect.cpp` | 合成螺丝头帧+平移极性+噪声 → 30 中心；旋转/镜像/畸变/纯旋转/缺失/假十字/混合色排除用例 |

### 修改

| 文件 | 改动 |
|---|---|
| `gui/calibration/circle_grid_display.{h,cpp}` | 螺丝头渲染（实心环+虚线十字）；`set_dot_gap`（clamp 1..3）；QPainter-on-QPixmap + cache_dirty_ |
| `algo/calibration/intrinsic.{h,cpp}` | 枚举加 `ScrewHeadGrid`；`detect_only` 调 `detect_screwheads`；`make_object_grid` 非对称公式 |
| `gui/calibration/calibration_wizard.{h,cpp}` | 标题 `"...(Zhou's Screw-Head Grid Method)"`；`dot_gap_`（1/2/3 默认 2）；`capture_window_`（200–20000 步 1 默认 5000）；"Circle spacing"→"Marker spacing"；三值彩色帧渲染；hint 英文；固定 6×5 |
| `gui/calibration/calibration_event_tap.h` | `kKeepWindowUs` 1100 → **21000**（≥ 20000 窗口 + 1000 余量） |
| `gui/calibration/calibration_worker.{h,cpp}` | 默认 pattern `ScrewHeadGrid`；`configure()` 固定 6×5；`process_frame` 接 CV_8UC3 |
| `algo/tests/calib_capture_probe.cpp` | 适配螺丝头/三值帧/可调窗口；默认窗口 500→5000 |

### 删除（死代码）

- `CalibrationPattern::AsymmetricCircles` 枚举值 + `findCirclesGrid` 分支。
- wizard 中 `cols_/rows_` spinbox、`prev_cols_/prev_rows_`、正方形拒绝逻辑。
- `test_intrinsic.cpp` 中 `AsymmetricCircles` 相关测试。

## 现场调试

- `GUI_SCREW_DEBUG=1` 环境变量启用 stderr 调试日志（stage2/3/4 统计 + 每候选环检测结果）。
- `calib_capture_probe <file.raw> [window_us] [sample_period_us] [max_samples]` 在 raw 录制上离线验证检测，输出 PNG 到 `/tmp/screwhead_probe/`。`dot_gap` 固定为 2（wizard 默认）。
- 检测失败时检查：cross+ring 联合标记数量（需覆盖大部分网格）、d 值（应与物理间距/2 一致）、best_score（需 ≥ need×0.5）、EMPTY slot 的 nearest 距离（若 > match_tol 则为透视畸变）。

## d 值估算（鲁棒，v3 NN 带通滤波）

- 优先用十字+圆环联合标记（双特征，最可靠）的环半径中位数 r_med 过滤噪声半径（[0.5, 2.0]×r_med）。
- **NN 带通滤波**（v3 新增）：用半径估算 d_radius = (r_med / 0.20) × 0.72（过估计 ~25%），
  计算 expected diagonal dexp = d_radius × √2，然后只保留 NN 距离在 [0.5, 1.5] × dexp 范围内的。
  下界拒绝聚集误报的小 NN 距离（原 v2 中把中位数拉到 19），上界拒绝 2 步距离（标记的最近网格
  邻居在 2 格外，常见于网格边缘）。d = median(filtered_nn) / √2。
- 实测：r_med=19 → dexp=96.7 → filtered 18/26 个 NN → d=61（真实 d≈43.5，过估计 ~40%，因
  径向直方图峰值外移）。网格间距 = |s|×√2（s=对角步长），与 d 无关——d 仅影响匹配容差和步长过滤。
- 不直接用半径估 d（径向直方图 3-bin 平滑 + 环厚度导致峰值外移 ~1.74×，校正因子不稳定）。

## Raw 数据测试结果（screw.raw，10 样本/窗口，dot_gap=2）

> v3 实现：十字/圆环同级置信度模型（50%+50%+20%），加权 RANSAC，**无空缺 slot 恢复**。
> 对比 v2（含确认/疑似分类 + 二阶段环搜索）。采样间隔 100000µs（10 样本覆盖 ~1s 录制）。

| 捕获窗口 | v3 检测率 | v2 检测率 | 说明 |
|---|---|---|---|
| 20000µs | 2/10 (20%) | 7/9 (78%) | v3 删除恢复后，透视畸变 >24px 的 slot 直接失败 |

### v3 检测率下降原因分析

1. **二阶段恢复已删除**（用户要求）：v2 在预测位置做二次环搜索（覆盖率≥0.40、±5px）恢复遗漏标记，
   v3 空缺 slot 直接导致拟合失败。这是检测率下降的主要原因。
2. **透视畸变是剩余瓶颈**：板倾斜导致局部 d 变化，全局 d 估计的预测位置与实际位置偏移 26-49px，
   超过匹配容差 0.40×d_estimated ≈ 24px。成功样本的偏移 5-21px（在容差内），失败样本 26-49px（超出）。
3. **d 过估计**：径向直方图峰值外移（r_med=15-24，真实 R=8.7），导致 d_estimated=58-64（真实 d=43.5）。
   但网格间距由对角步长 |s| 决定（= |s|×√2，与 d 无关），所以 d 过估计不影响位置预测，
   仅使匹配容差变宽（0.40×61=24.4 vs 0.40×43.5=17.4）——反而有助于匹配。
4. **匹配容差不可再放宽**：kMatchTolFrac 从 0.40 提到 0.45 会导致错误假设被选中（假匹配 inflate
   错误放置的 score，使预测位置偏移 >1 格）。0.40 是安全上限（0.56×d_true < 0.707×d_true）。

### 改善方向（未实现）

- **透视感知网格拟合**：用单应矩阵估计代替全局 d，适应板倾斜。这是解决透视畸变的根本方案。
- **候选过滤加强**：当前 524-1085 个 cross 候选（仅 ~30 真实），过多噪声候选增加 RANSAC 计算量
  和假匹配风险。可在 stage 2 加强面积/形状过滤。

### 关键发现（保留）

1. **窗口大小是关键参数**：5000µs 默认偏短，15000µs 以上检测率显著提升。
2. **半径一致性过滤不应用于标记分类**：大半径"噪声"标记（r=40-169）实为运动鬼影标记——
   位置与真实网格位置重合（有助于网格拟合锚定），但半径因合并弧而膨胀。d 估计中的半径过滤保留
   （仅用于间距估算，不影响投票权重）。
3. **角度覆盖率 ≥0.60** 已实现，符合用户要求。
4. **dot_gap=2**（wizard 默认）比 dot_gap=1 的去噪/膨胀核半径更大（3 vs 2），能更好地桥接十字点间距，
   减少 stage 2 假候选数量（1070→524）。
