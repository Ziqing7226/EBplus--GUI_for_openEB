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

## 检测（四阶段，无冗余）

1. **累积** 200–20000µs 事件 → **三值彩色帧**：背景黑、ON=金、OFF=白；同位置多极性事件取金与白的**简单平均**。
2. **十字定中心**：8 邻域去孤立点 → 各向同性膨胀 + 连通域 + 面积过滤 → distance-transform 峰值 = 十字中心候选。
3. **圆环局部 Hough + 覆盖率验证**（替代旧的极性半圆验证）：
   - 在十字候选 ±2px 邻域内搜索使径向直方图峰值最大的圆心 = 圆环中心（局部 Hough 优化）。
   - 收集环像素（峰值半径 ±2px），按极角分 36 bin，计算角度覆盖率（有像素的 bin 占比）。
   - 覆盖率 ≥ `kRingCoverFrac`(0.60) = 检测到标记（疑似）；≥ `kConfirmedCoverFrac`(0.70) = 确认标记。
   - **放弃极性半圆验证**：相机运动时圆环外沿/内沿在每个角度极性相反（leading:外ON/内OFF；trailing:外OFF/内ON），
     旧的"一半全ON、一半全OFF"模型不成立。极性仅用于着色，验证改为纯几何（环信号强度 + 角度覆盖率）。
4. **加权网格拟合 + 空缺slot二阶段环搜索**：
   - 用确认标记（覆盖率≥0.70）的环半径一致性过滤 + 最近邻对角步长估网格半格间距 d；
   - 确认标记权重 2.0、疑似权重 1.0，RANSAC 搜索最优 6×5 块；
   - **空缺slot二阶段环搜索**：对匹配容差内无标记的预测位置，直接用预测坐标为种子搜索环信号
     （覆盖率≥0.40、±5px种子搜索、max_r=中位确认环半径+6px），恢复因十字过稀疏或环信号弱而遗漏的标记。
   - 所有位置必须有匹配特征或恢复成功（不允许凭空补全）。
   - 最终按 (y, x) 排序解决 8 重方向歧义。

### 联合定中心 + 确认/疑似分类

- **十字中心**：distance-transform 峰值（每个螺丝头组件中最厚的点 = 十字交叉）。
- **圆环中心**：局部 Hough 优化（十字候选邻域内使环信号最强的位置）。
- **确认标记**：圆环覆盖率高（≥0.70）→ 权重 2.0，用于锚定网格。
- **疑似标记**：圆环覆盖率低（0.25–0.70）或仅十字 → 权重 1.0，用于补全网格缺口。
- **最佳中心**：有环时用环中心（局部优化更准），仅十字时用十字中心。

### 为什么放弃极性半圆验证

旧设计假设：环边在平移下 leading/trailing 半圆极性相反、各半一致；噪声随机交错→拒。
实测发现：相机运动时圆环**无论哪一侧都有两种极性同时存在**——leading 方向外沿 ON/内沿 OFF，
trailing 方向外沿 OFF/内沿 ON。这是四象限相反结构，不是半圆一致结构，所以 half/half 测试
对真标记也会失败。新设计用环的角度覆盖率（几何特征）替代极性验证，运动无关。

### d 值估算（鲁棒）

- 优先用确认标记的环半径中位数过滤噪声（半径在 [0.5, 2.0]×中位数外的是噪声，如 r=80）。
- 用过滤后的确认标记的最近邻对角步长估 d（d = median_NN / √2），不受噪声点位置污染。
- 环半径 R=0.20·d 的关系存在峰值偏移（径向直方图 3-bin 平滑 + 环厚度），d=r/0.20 会高估 ~40%，
  所以不直接用半径估 d，仅用作后备（带 0.72 校正因子）。

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
| `kRingCoverFrac` | 0.60 | 接受环的最低覆盖率（疑似标记）——用户要求≥0.6 |
| `kConfirmedCoverFrac` | 0.70 | 确认标记的覆盖率阈值 |
| `kMinRingPixels` | 6 | 最小环像素数 |
| `kRingSearchMargin` | 12 | 环搜索半径余量（加到十字组件半对角线上） |
| `kRecoverCoverFrac` | 0.40 | 二阶段恢复的最低覆盖率（空缺slot填充） |
| `kRecoverMinPixels` | 6 | 二阶段恢复的最小环像素数 |
| `kRecoverSeedSearch` | 5 | 二阶段恢复的种子搜索半径（±5px，初始±2px） |
| `kRingRadiusFrac` | 0.20 | R = 0.20·d（与渲染一致） |
| `kMatchTolFrac` | 0.40 | 网格匹配容差 = 0.40·d（0.35→0.40：允许透视畸变下略偏的标记匹配） |
| `kConfirmedWeight` | 2.0 | 确认标记投票权重 |
| `kSuspectedWeight` | 1.0 | 疑似标记投票权重 |

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
- `calib_capture_probe <file.raw> [window_us] [dot_gap]` 在 raw 录制上离线验证检测，输出 PNG 到 `/tmp/screwhead_probe/`。
- 检测失败时检查：confirmed 数量（需覆盖大部分网格）、d 值（应与物理间距/2 一致）、best_score（需 ≥ 30）。

## Raw 数据测试结果（screw.raw，10 样本/窗口）

| 捕获窗口 | 检测率 | 说明 |
|---|---|---|
| 5000µs（默认） | 3/9 (33%) | 事件稀疏，仅 15-21 确认标记（需30）；二阶段恢复可填补部分空缺 |
| 10000µs | 3/9 (33%) | 事件增加但噪声也增加，净效果不变 |
| 15000µs | 6/9 (67%) | 事件足够密集，确认标记达 20-24，恢复成功率高 |
| 20000µs | 7/9 (78%) | 最佳检测率；剩余失败为透视畸变导致网格拟合位置偏移 28-39px |

### 关键发现

1. **二阶段环搜索有效**：5000µs 下从 0% → 33%，通过在预测位置直接搜索弱环信号恢复遗漏标记。
2. **窗口大小是关键参数**：5000µs 默认偏短，15000µs 以上检测率显著提升。
3. **透视畸变是剩余瓶颈**：全局 d 估计无法适应板倾斜导致的局部 d 变化（r=10-15 → d=50-75），
   预测位置偏移 28-39px 导致部分 slot 空缺。需透视感知网格拟合（单应矩阵估计）才能进一步改善。
4. **半径一致性过滤不应用于标记分类**：20000µs 下大半径"噪声"标记（r=40-169）实为运动鬼影标记——
   位置与真实网格位置重合（有助于网格拟合锚定），但半径因合并弧而膨胀。降级它们会丢失有用的位置投票
   （测试：7/9 → 5/9）。d 估计中的半径过滤保留（仅用于间距估算，不影响投票权重）。
5. **角度覆盖率 ≥0.60（疑似阈值）**已实现，符合用户要求。二阶段恢复阈值为 0.40（更宽松，用于空缺填充）。
