<div align="center">

# EB plus

基于 [openEB](https://github.com/prophesee-ai/openeb) v5.2.0 的开源 Qt 6 事件相机桌面应用。

实时可视化 · 相机控制 · 录制回放 · 标定 · 59 个算法 · 可定制主题

![License](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue)
![Language](https://img.shields.io/badge/C%2B%2B17-Qt%206-orange)
![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey)
![Version](https://img.shields.io/badge/version-1.9.0-blue)

![主界面](pic/1.9.0.png)

</div>

---

## 这是什么？

**EB plus** 是一个美观、开源、功能丰富的事件相机 GUI 工具，支持 Prophesee / CenturyArks 事件相机。事件相机不采集帧——它以微秒级时间分辨率逐像素报告亮度变化。EB plus 提供完整的事件数据桌面工作流：

- **实时显示** 事件流（OpenGL，60+ FPS）
- **控制相机** —— biases、ROI、抗闪烁、触发
- **录制与回放** RAW 事件文件，支持速度控制与跳转
- **运行算法** —— 噪声过滤、光流、目标跟踪、事件转视频等
- **标定相机** —— 棋盘格向导
- **导出** 为 HDF5 / CSV / AVI

本项目完全开源，欢迎 fork 并按需修改。

> **什么是事件相机？** 与传统帧相机不同，事件相机输出异步的逐像素亮度变化——"事件"——具有微秒级时间分辨率、高动态范围和低功耗。

---

## 快速开始

```bash
# 编译
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j$(nproc)

# 运行（启动脚本会自动设置所有必需的环境变量）
./run.sh
```

启动脚本会自动处理 Wayland 兼容、HAL 插件路径和 OpenGL 后端选择。

> **环境要求**：Ubuntu 22.04+ · GCC 13+ · Qt 6 · OpenCV 4。详见 [doc/compile.md](doc/compile.md)。

---

## 功能特性

### 实时事件显示
- OpenGL 加速渲染（GLSL 3.30 core profile，letterbox 视口）
- 7 种帧模式：Integration、Diff、Histogram、Time Decay、Contrast Map、Periodic、On-Demand
- 4 种色彩主题：Dark、Light、CoolWarm、Gray
- 实时统计：事件率、ON/OFF 比、FPS、时间戳

### 相机控制面板
- **Biases 面板** —— 动态枚举所有 HAL bias，滑块 + 精确输入 + Reset，保存/加载 `.bias` 文件
- **ROI 面板** —— 多矩形 ROI / RONI（`I_ROI`），显示区拖拽选区
- **ESP 面板** —— Anti-Flicker（模式/频带/预设/占空比/阈值）、Trail Filter（类型/阈值）、ERC（目标事件率）
- **Trigger 面板** —— Trigger In（逐通道启用）+ Trigger Out（启用/周期/占空比）

所有面板在设备不支持对应 HAL facility 时优雅降级（如文件回放时四个硬件面板自动禁用）。

### 录制与回放
- RAW 录制 —— 实时相机流录制，带实时缓冲刷新
- 文件回放 —— 速度控制、跳转、暂停/恢复、位置追踪
- 文件裁剪 —— 从事件文件中提取时间段

### 数据导出与转换
- RAW、HDF5、CSV 格式互转
- 事件数据导出为 AVI 视频（可配置 FPS、累积时间、画质、色彩模式）

### 事件预处理滤波链
8 级可叠加阶段，线程安全管线：Polarity Filter、Polarity Invert、Flip X、Flip Y、Rotate、Transpose、Rescale、ROI Filter。从侧栏切换。

### 算法（共 59 项）
EB plus 内置 **29 个自研算法** + **30 项 openEB 封装能力**，全部注册在统一的 `AlgoBridge` 注册表中。

| 类别 | 示例 |
|------|------|
| **滤波** | Hot Pixel Filter、Background Mask、Bandpass Filter、Trigger Synced |
| **运动** | Sparse Optical Flow（4 模式）、Direction Selective、EIS / Optical Gyro |
| **检测** | Blob Detector、Corner Detector（Harris/FAST/AGAST）、Line Segment（ELiSeD）|
| **跟踪** | Object Tracker（RCT/Median/Kalman/MultiHypothesis）、Hough Circle、Hough Line、Active Marker |
| **重建** | Event-to-Video —— **E2VID**（默认，DL）、BardowVariational、InteractingMaps |
| **分析** | Frequency Detector、Flow Statistics、ISI Analyzer、Particle Counter、Auto Bias |
| **可视化** | Time Surface、XYT 3D 点云、Ultra Slow Motion、Orientation Cluster |
| **标定** | Intrinsic Calibration（棋盘格 / 圆阵列 / aruco）|

算法**互斥**——启用一个会禁用上一个。每个自研算法支持**全局 ROI**（默认中心 128×128）和共享的 **"ROI → 噪声滤波 → 1/4 下采样"** 预处理阶段以控制计算量。所有算法参数仅在**侧栏**（`AlgorithmsPanel`）调节；算法显示窗口只展示标题与输出，避免两处独立参数面板不同步。

#### 噪声滤波（共享预处理）
8 种模式按所选滤波器在侧栏暴露：BAF、STCF、Refractory、DWF、AgePolarity、Harmonic、Repetitious、SpatialBP。

#### E2VID 神经网络重建（默认模式）

Event-to-Video 算法默认使用 **E2VID** —— 从原始事件流重建灰度图像的深度学习模型，移植自 [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid)，通过 ONNX Runtime（多线程 CPU）推理。

**部署**（一次性，约 5 分钟）：

```bash
# 1. 下载 ONNX Runtime 1.19.2（Linux x64 CPU）到 third_party/
cd /path/to/GUI-for-openEB
mkdir -p third_party/onnxruntime && cd third_party/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-1.19.2.tgz
tar xzf onnxruntime-linux-x64-1.19.2.tgz --strip-components=1
cd ../..

# 2. 创建 Python 转换环境
python3 -m venv .venv && . .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu onnx onnxscript onnxruntime numpy
deactivate

# 3. 下载 PyTorch 预训练权重（约 41 MB）
wget -P models/ http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar

# 4. 转换为 ONNX（生成 models/e2vid_lightweight.onnx）
. .venv/bin/activate && python models/convert_to_onnx.py && deactivate

# 5. 重新编译（CMake 自动检测 ONNX Runtime）
cmake --build build -- -j$(nproc)
```

完成后启动 EB plus，启用 **Algorithm → Event → Video** 即默认 E2VID 模式（128×128 ROI、30fps、1/4 下采样：64×64 推理 → 上采样回 128×128）。GUI 暴露可调参数：模型路径、auto-HDR、锐化强度、双边滤波。

> **无 ONNX Runtime 时**：E2VID 自动回退到启发式模式（体素网格求和 + Sigmoid）。BardowVariational 和 InteractingMaps 模式无需任何额外依赖——BardowVariational 通过 Chambolle-Pock 原始-对偶优化联合估计光流与亮度（六个 λ 正则化项），InteractingMaps 使用六张互连图（I/G/V/F/C/R）交替松弛，旋转由线性最小二乘估计。

详见 [doc/design.md §4.4.2](doc/design.md)。

### 主题
- **5 种背景色**：Gray、Green、Yellow、Pink、Blue（默认）
- **3 种模式**：Follow System（默认）、Always Light、Always Dark
- 暗色模式使用所选颜色的**暗色变体**——而非纯黑
- 文字颜色自动适配（浅色背景用黑，暗色背景用白）
- 设置跨重启持久化；标题栏跟随主题

### 多窗口与布局
- XYT 3D 事件点云（GPU 加速）
- 额外算法显示窗口（可停靠）
- 布局保存/恢复到 JSON

---

## 目录结构

```
GUI-for-openEB/
├── gui/              # Qt 6 应用
│   ├── main_window.*     # 主窗口：标题栏菜单、dock、信号连接
│   ├── display/          # OpenGL 视口、叠加层、3D 点云
│   ├── panels/           # VSCode 风格侧栏面板（5 组 11 个面板）
│   ├── app/              # 控制器（相机、管线、主题…）
│   ├── algo_bridge/      # 算法注册表 + 滤波链
│   ├── recorder/         # RAW 录制 & 回放
│   ├── exporter/         # HDF5/CSV/AVI 导出
│   ├── calibration/      # 内参向导
│   └── widgets/          # 标题栏、ActivityBar、AlgoWindow、像素探针
├── algo/              # 自研算法库（29 模块）
├── openeb/            # openEB SDK（Apache 2.0，v5.2.0）
├── models/            # E2VID PyTorch → ONNX 转换
├── run.sh             # 启动脚本（环境变量设置）
├── doc/               # 设计规格 + 编译指南 + wiki
│   └── macos.md       # macOS Apple Silicon 打包指南
└── pic/               # 截图
```

---

## 环境要求

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 26.04（或兼容 Linux）；macOS Apple Silicon 见 [doc/macos.md](doc/macos.md) |
| 编译器 | GCC 15+ |
| CMake | 4.x |
| Qt | 6.x（Widgets、OpenGL、OpenGLWidgets） |
| OpenEB SDK | 5.2.0（已作为子树包含） |
| OpenCV | 4.x |
| Python | 3.12（仅在从源码编译 openEB 时需要） |

详见 [doc/compile.md](doc/compile.md) 和 [doc/macos.md](doc/macos.md)，包含 GCC 15 `<cstdint>` 修复、Python 3.12 说明和 macOS 打包流程。

---

## 编译

```bash
# 1. 确保 openEB SDK 已安装（见 doc/compile.md）
# 2. 配置
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 3. 编译
cmake --build build -- -j$(nproc)
```

编译产物输出到 `build/gui/gui_for_openeb`。

---

## macOS Apple Silicon

macOS 打包路径会优先构建仓库内的 `openeb/`，并生成 `.app` 和 `.dmg`：

```bash
brew install cmake qtbase qtsvg opencv boost libusb protobuf abseil gcc
./scripts/build_macos_app.sh
open build/dist/EBplus.app
```

产物：

- `build/dist/EBplus.app`
- `build/dist/EBplus-macos-arm64.dmg`

打包会携带 Qt SVG 运行时，确保左侧 activity bar 图标正常显示。默认产物只包含通用 OpenEB plugin；silkyCam 等厂商 plugin 仅可作为本地测试时的显式附加项，确认源代码和再分发条款前不会进入 CI 或公开 DMG：

```bash
EXTRA_HAL_PLUGIN_PATHS="/path/to/silkycam/hal/plugins" ./scripts/build_macos_app.sh
```

当前 macOS 包不支持 HDF5；详细说明见 [doc/macos.md](doc/macos.md)，包括签名、notarization、依赖检查、插件分发边界和 HDF5 限制。

---
## 运行

### 方式一：启动脚本（推荐）

```bash
./run.sh
```

脚本自动检测 Wayland，强制 XCB + OpenGL（避免黑屏），并设置 HAL/HDF5 插件路径。

### 方式二：手动启动

```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/local/lib"
export HDF5_PLUGIN_PATH="/usr/local/lib/hdf5/plugin"
export MV_HAL_PLUGIN_PATH=/usr/local/lib/metavision/hal/plugins  # Prophesee
# export MV_HAL_PLUGIN_PATH=/usr/lib/CenturyArks/hal/plugins     # CenturyArks
export QT_QPA_PLATFORM=xcb       # Wayland 对 QOpenGLWidget 渲染黑屏
export QSG_RHI_BACKEND=opengl    # Qt 6 可能默认使用 Vulkan

./build/gui/gui_for_openeb
```

### 相机厂商配置

| 厂商 | HAL 插件路径 |
|------|-------------|
| Prophesee | `/usr/local/lib/metavision/hal/plugins` |
| CenturyArks | `/usr/lib/CenturyArks/hal/plugins` |

---

## 常见问题

**启动后黑屏** —— 使用启动脚本。若手动启动，设置 `QT_QPA_PLATFORM=xcb` 和 `QSG_RHI_BACKEND=opengl`。

**相机未检测到** —— 确认 `MV_HAL_PLUGIN_PATH` 与你的厂商匹配，运行 `metavision_hal_ls` 检查。

**"NonMonotonicTimeHigh" 错误** —— 这是部分 Gen3.x 相机启动时约 50% 概率出现的 Evt3 协议瞬态警告。EB plus 将其视为非致命，保持流运行。无需处理。

**暗色模式不跟随系统** —— 需要 Qt 6.5+。旧版 Qt 请用 Theme → Mode → Dark。

---

## 快捷键

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+O` | 打开文件 |
| `Ctrl+Shift+P` | 切换回放面板 |
| `F11` | 全屏 |
| `Ctrl+Q` | 退出 |

---

## 已知问题与反馈

EB plus 正在持续开发中，可能仍存在 BUG。如果你在使用过程中遇到任何问题——崩溃、渲染异常、控件失灵或非预期行为——欢迎[提交 issue](../../issues)。来自真实用户的反馈是最直接的帮助。

---

## 许可证

- **项目原创代码**：[MIT](LICENSE)
- **openEB SDK**：[Apache 2.0](openeb/licensing/LICENSE_OPEN) —— 版权归 Prophesee 所有

---

<div align="center">

基于 Qt 6 · OpenCV · openEB SDK 构建

</div>
