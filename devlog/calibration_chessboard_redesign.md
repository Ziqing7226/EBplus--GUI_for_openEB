# 标定重设计：Zhou's Screw-Head Grid（十字螺丝头点阵 + 极性验证）

> 已实现。弹窗标题括号内为 `Zhou's Screw-Head Grid Method`（"Intrinsic Calibration (Zhou's Screw-Head Grid Method)"）。

## 概述

沿用非对称 6×5 网格布局（30 位置，朝向内置消歧），把每个圆点改为"螺丝头"标记 = 虚线十字（定中心）
+ 外层实心薄环（极性验证源）。检测：十字定中心 → 圆环极性半圆验证 → 非对称网格拟合 + cornerSubPix。
用事件极性（三值帧：黑/金/白）做强抗噪声验证，全程局部 → 旋转/畸变鲁棒。

## 标定板

- 30 标记，位置 `(2c+(r&1), r)·S`（S=半格；用户测量同行/同列相邻标记圆心距=2S，算法 `S=测量值/2`，无√2）。
- 每标记：实心白环（外 `R+g/2`、内 `R-g/2`，厚度 `g`）+ 虚线十字（1px 点、周期 `1+g`、臂长 `L=R_in-g-1`）。
- `R = 0.20·S`（0.30 时外环与标定板上下边界相切，0.20 留出间距；现场可调），`g = dot_gap`。
- 渲染：QPainter-on-QPixmap + `cache_dirty_`（沿用 D11 性能路径，禁 QImage 全图上传）。

## 检测（四阶段，无冗余）

1. **累积** 100–1000µs 事件 → **三值彩色帧**：背景黑、ON=金、OFF=白；同位置多极性事件取金与白的**简单平均**（不按事件数加权）。
2. **十字定中心**：8 邻域去孤立点（前置减计算量）→ 各向同性膨胀（圆盘 `g+1`，前景=非黑）+ 连通域 + 面积过滤（拒小噪声块、拒大环块）
   → 十字质心 = 中心候选。
3. **圆环极性半圆验证**：每候选小邻域径向 Hough 找强圆信号半径 `R*` → 收集 `R*` 附近像素（含平均色，**不排除**）→
   按极角排序检查"连续半圆同极性、另半反极性"（跳变 ≤2、两段各约半；平均色为合法颜色，不影响判断）→ 真标记。
4. **网格拟合+亚像素**：非对称 6×5 RANSAC 拟合 + cornerSubPix → 30 有序中心。

极性半圆原理：环边在平移下 leading/trailing 半圆极性相反、各半一致；噪声随机交错→拒。

## 参数（用户已定）

- `dot_gap`：1/2/3，**默认 2**。
- 捕获窗口：100–1000µs，步进 100µs，**默认 500µs**。
- 标记间距：用户测量同行（或同列）相邻两标记圆心距，算法 `d = 测量值/2`（无√2）。
- 三值帧：黑/金/白，多极性事件取金白简单平均（不加权）；平均色为合法颜色、不排除，不影响极性半圆判断。
- 圆环实心，十字虚线。
- 死代码删除（`AsymmetricCircles` 路径、cols/rows 等；`circle_grid_display` 文件**保留原改**）。
- hint（英文）：`Hold the camera steady; rely on hand micro-tremor to trigger events. Press Space to capture.`

## 代码改动

### 新增

| 文件 | 内容 |
|---|---|
| `algo/calibration/screwhead_detect.{h,cpp}` | `detect_screwheads(color_frame, ...) → (found, vector<Point2f>)`。四阶段管线；输入三值彩色帧（CV_8UC3），前景=非黑，极性由颜色判 |
| `algo/tests/test_screwhead_detect.cpp` | 合成螺丝头帧+平移极性+噪声 → 30 中心；旋转/镜像/畸变/纯旋转/缺失/假十字/混合色排除用例 |

### 修改

| 文件 | 改动 |
|---|---|
| `gui/calibration/circle_grid_display.{h,cpp}` | **华夫饼渲染 → 螺丝头渲染**（实心环+虚线十字，原改不删文件）。`set_dot_size` → `set_dot_gap`（clamp 1..3）；保留 `set_square_size_mm`、QPainter-on-QPixmap + cache_dirty_ |
| `algo/calibration/intrinsic.h` | 枚举加 `ScrewHeadGrid`；**删** `AsymmetricCircles` |
| `algo/calibration/intrinsic.cpp` | `detect_only` 加 `ScrewHeadGrid` 分支调 `detect_screwheads`（接收 3 通道帧）；`make_object_grid` 用非对称公式；**删** `AsymmetricCircles` 分支 |
| `gui/calibration/calibration_wizard.h` | `dot_size_` → `dot_gap_`；加 `capture_window_` spinbox；**删** `cols_/rows_/prev_cols_/prev_rows_`（固定 6×5） |
| `gui/calibration/calibration_wizard.cpp` | 标题 → `"...(Zhou's Screw-Head Grid Method)"`；`dot_gap_`（1/2/3 默认 2）；`capture_window_`（100–1000 步 100 默认 500）；"Circle spacing"→"Marker spacing"；`spacing_note_` 英文改"nearest marker centers"；`kCaptureWindowUs` 改用 `capture_window_->value()`；`render_event_frame` 二值(CV_8UC1) → **三值彩色(CV_8UC3)**（黑/金/白、多极性事件取金白简单平均）；hint 英文（见上）；**删** cols/rows spinbox 及正方形拒绝逻辑；`configure_requested` 信号去 cols/rows |
| `gui/calibration/calibration_event_tap.h` | `kKeepWindowUs` 6000 → **1100**（≥ 1000 窗口 + 余量） |
| `gui/calibration/calibration_worker.{h,cpp}` | 默认 pattern `ScrewHeadGrid`；`configure()` 去 cols/rows（固定 6×5）；`process_frame` 接 CV_8UC3；错误文案 `"Markers not detected — re-aim and try again."` |
| `gui/CMakeLists.txt` | 无需改（`circle_grid_display.cpp` 保留）；如改类名则更新 |
| `algo/CMakeLists.txt` | 加 `calibration/screwhead_detect.cpp` |
| `algo/tests/test_intrinsic.cpp` | 加 `ScrewHeadGrid` 物方点测试（30 点非对称）；**删** `AsymmetricCircles` 测试 |
| `algo/tests/calib_capture_probe.cpp` | 适配螺丝头/三值帧/极性/可调窗口 |

### 删除（死代码）

- `CalibrationPattern::AsymmetricCircles` 枚举值 + `findCirclesGrid` 分支（intrinsic.cpp）。
- wizard 中 `cols_/rows_` spinbox、`prev_cols_/prev_rows_`、正方形拒绝逻辑。
- `test_intrinsic.cpp` 中 `AsymmetricCircles` 相关测试。
- （`circle_grid_display.{h,cpp}` 文件**保留**，仅改渲染内容。）

## 提交计划（增量，每步编译+ctest+实测）

1. **C1 渲染**：`circle_grid_display` 华夫饼→螺丝头 + 接入 wizard（不接检测）。目视确认 30 螺丝头。
2. **C2 检测+单测**：`screwhead_detect` + `test_screwhead_detect`（三值帧输入）。ctest 全绿。
3. **C3 算法层**：`intrinsic` 加 `ScrewHeadGrid` + 物方点测试。ctest 全绿。
4. **C4 wizard 集成**：dot_gap、Marker spacing、窗口可调、三值帧渲染、标题、固定 6×5、hint、worker 默认。现场实测。
5. **C5 删死代码**：`AsymmetricCircles`、cols/rows、旧测试。ctest 全绿。
6. **C6 probe 适配**：现场 raw 验证。

## 待现场定

- `kR = 0.30`（C1 目视、C4 实测定）。
- 极性半圆跳变阈值/纯色判定容差（C2 合成调、C4 实测定）。
