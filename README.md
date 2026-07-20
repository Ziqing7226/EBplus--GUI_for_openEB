<div align="center">

# EB plus

A polished, open-source Qt 6 desktop app for event cameras — built on [openEB](https://github.com/prophesee-ai/openeb) v5.2.0.

Real-time visualization · camera control · recording & playback · calibration · 59 algorithms · customizable themes

![License](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue)
![Language](https://img.shields.io/badge/C%2B%2B17-Qt%206-orange)
![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey)
![Version](https://img.shields.io/badge/version-1.9.0-blue)

![Main Window](pic/1.9.0.png)

</div>

---

## What is this?

**EB plus** is a beautiful, open-source, feature-rich GUI for event cameras (Prophesee / CenturyArks). Event cameras don't capture frames — they report per-pixel brightness changes at microsecond resolution. EB plus gives you a complete desktop workflow to work with this data:

- **See** the event stream in real time (OpenGL, 60+ FPS)
- **Control** the camera — biases, ROI, anti-flicker, triggers
- **Record & replay** RAW event files with speed control and seek
- **Run algorithms** — noise filtering, optical flow, object tracking, event-to-video, and more
- **Calibrate** the camera with a chessboard wizard
- **Export** to HDF5 / CSV / AVI

The whole project is open source — feel free to fork it and adapt it to whatever you need.

---

## Quick Start

### Linux

```bash
# Build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j$(nproc)

# Run (the launcher sets all required env vars)
./run.sh
```

That's it. The launcher handles Wayland compatibility, HAL plugin paths, and OpenGL backend selection automatically.

### macOS Apple Silicon

```bash
brew install cmake qtbase qtsvg opencv boost libusb protobuf abseil gcc
./scripts/build_macos_app.sh
open build/dist/EBplus.app
```

The macOS packaging flow builds the vendored `openeb/` tree and creates `build/dist/EBplus-macos-arm64.dmg`. It bundles the Qt SVG runtime so the activity-bar icons render correctly. It ships generic OpenEB plugins; vendor plugins such as silkyCam are opt-in local additions. HDF5 is not supported by the current macOS package. See [doc/macos.md](doc/macos.md) for plugin bundling, signing, and verification notes.

> **Requirements**: Ubuntu 22.04+ / Apple Silicon macOS · Qt 6 · OpenCV 4. See [doc/compile.md](doc/compile.md) and [doc/macos.md](doc/macos.md) for details.

---

## Features

### Real-time Display
- OpenGL-accelerated rendering with letterboxed viewport
- 7 frame modes: Integration, Diff, Histogram, Time Decay, Contrast Map, Periodic, On-Demand
- 4 color palettes: Dark, Light, CoolWarm, Gray
- Live statistics: event rate, ON/OFF ratio, FPS, timestamp

### Camera Control
- **Biases** — all HAL biases with slider + spinbox, save/load `.bias` files
- **ROI** — multi-rectangle ROI / RONI, drag-to-select on the display
- **ESP** — Anti-Flicker, Trail Filter, Event Rate Control
- **Trigger** — Trigger In (per-channel) + Trigger Out

All panels degrade gracefully when the device lacks the corresponding HAL facility (e.g. the four hardware panels auto-disable during file playback).

### Recording & Playback
- RAW recording from live cameras
- File playback with speed control, seek, pause/resume
- File cutter — extract a time range from an event file

### Export & Conversion
- Convert between RAW, HDF5, and CSV
- Export events to AVI video (configurable FPS, accumulation, quality, color mode)

### Preprocessing Filter Chain
8 stackable stages applied in a thread-safe pipeline: Polarity Filter, Polarity Invert, Flip X, Flip Y, Rotate, Transpose, Rescale, ROI Filter. Toggled from the sidebar.

### Algorithms (59 total)
EB plus ships **29 self-developed algorithms** plus **30 OpenEB-wrapped capabilities**, all registered in a single `AlgoBridge` registry.

| Category | Examples |
|----------|----------|
| **Filtering** | Hot Pixel Filter, Background Mask, Bandpass Filter, Trigger Synced |
| **Motion** | Sparse Optical Flow (4 modes), Direction Selective, EIS / Optical Gyro |
| **Detection** | Blob Detector, Corner Detector (Harris/FAST/AGAST), Line Segment (ELiSeD) |
| **Tracking** | Object Tracker (RCT/Median/Kalman/MultiHypothesis), Hough Circle, Hough Line, Active Marker |
| **Reconstruction** | Event-to-Video — **E2VID** (default, DL), BardowVariational, InteractingMaps |
| **Analytics** | Frequency Detector, Flow Statistics, ISI Analyzer, Particle Counter, Auto Bias |
| **Visualization** | Time Surface, XYT 3D Point Cloud, Ultra Slow Motion, Orientation Cluster |
| **Calibration** | Intrinsic Calibration (chessboard / circle grid / aruco) |

Algorithms are **mutually exclusive** — enabling one disables the previous. Each self-developed algorithm supports a **global ROI** (default: center 128×128) and a shared **"ROI → noise filter → 1/4 downsample"** preprocessing stage to bound computational cost. All algorithm parameters are adjusted exclusively in the **sidebar** (`AlgorithmsPanel`); algorithm display windows show only the title and output, preventing parameter drift between two independent control panels.

#### Noise Filter (shared preprocessing)
8 modes exposed in the sidebar based on the selected filter: BAF, STCF, Refractory, DWF, AgePolarity, Harmonic, Repetitious, SpatialBP.

#### E2VID Neural Network Reconstruction (Default)

The Event-to-Video algorithm defaults to **E2VID** — a deep-learning model that reconstructs grayscale images from raw event streams. It is ported from [rpg_e2vid](https://github.com/uzh-rpg/rpg_e2vid) and runs via ONNX Runtime (CPU, multi-threaded).

**Setup** (one-time, ~5 minutes):

```bash
# 1. Download ONNX Runtime 1.19.2 (Linux x64 CPU) into third_party/
cd /path/to/GUI-for-openEB
mkdir -p third_party/onnxruntime && cd third_party/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-1.19.2.tgz
tar xzf onnxruntime-linux-x64-1.19.2.tgz --strip-components=1
cd ../..

# 2. Create Python venv for model conversion
python3 -m venv .venv && . .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu onnx onnxscript onnxruntime numpy
deactivate

# 3. Download pre-trained PyTorch weights (~41 MB)
wget -P models/ http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar

# 4. Convert to ONNX (produces models/e2vid_lightweight.onnx)
. .venv/bin/activate && python models/convert_to_onnx.py && deactivate

# 5. Rebuild (CMake auto-detects ONNX Runtime)
cmake --build build -- -j$(nproc)
```

After setup, launch EB plus and enable **Algorithm → Event → Video** — it defaults to E2VID mode with 128×128 ROI, 30 fps, and 1/4 downsample (64×64 inference → upsampled to 128×128). The GUI exposes toggleable parameters (model path, auto-HDR, unsharp mask, bilateral filter).

> **Without ONNX Runtime**: E2VID falls back to a heuristic mode (voxel-grid sum + sigmoid). BardowVariational and InteractingMaps modes work without any setup — BardowVariational jointly estimates optical flow and intensity via Chambolle-Pock primal-dual optimization (all six λ terms), and InteractingMaps uses six interconnected maps (I/G/V/F/C/R) with rotation estimation via least squares.

See [doc/design.md §4.4.2](doc/design.md) for full algorithm specifications.

### Theming
- **5 background colors**: Gray, Green, Yellow, Pink, Blue (default)
- **3 modes**: Follow System (default), Always Light, Always Dark
- Dark mode uses a **dark variant of the chosen color** — not just black
- Text color auto-adjusts (black on light, white on dark)
- Settings persist across restarts; the title bar follows the theme

### Multi-Window & Layout
- XYT 3D event point cloud (GPU-accelerated)
- Additional algorithm display windows (dockable)
- Save/restore dock layout to JSON

---

## Directory Structure

```
GUI-for-openEB/
├── gui/              # Qt 6 application
│   ├── main_window.*     # Main window: title-bar menus, docks, signal wiring
│   ├── display/          # OpenGL viewport, overlays, 3D cloud
│   ├── panels/           # VSCode-style sidebar panels (5 groups, 11 panels)
│   ├── app/              # Controllers (camera, pipeline, theme, …)
│   ├── algo_bridge/      # Algorithm registry + filter chain
│   ├── recorder/         # RAW recording & playback
│   ├── exporter/         # HDF5/CSV/AVI export
│   ├── calibration/      # Intrinsic wizard
│   └── widgets/          # Title bar, ActivityBar, AlgoWindow, pixel probe
├── algo/              # Self-developed algorithm library (29 modules)
├── openeb/            # openEB SDK (Apache 2.0, v5.2.0)
├── models/            # E2VID PyTorch → ONNX conversion
├── run.sh             # Launcher (sets env vars)
├── doc/               # Design spec + build guide + wiki
│   └── macos.md       # Apple Silicon app/DMG packaging
└── pic/               # Screenshots
```

---

## Running

### Option 1: Launcher (Recommended)

```bash
./run.sh
```

The launcher auto-detects Wayland, forces XCB + OpenGL (avoids black screen), and sets HAL/HDF5 plugin paths.

### Option 2: Manual

```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/local/lib"
export HDF5_PLUGIN_PATH="/usr/local/lib/hdf5/plugin"
export MV_HAL_PLUGIN_PATH=/usr/local/lib/metavision/hal/plugins  # Prophesee
# export MV_HAL_PLUGIN_PATH=/usr/lib/CenturyArks/hal/plugins     # CenturyArks
export QT_QPA_PLATFORM=xcb       # Wayland renders black for QOpenGLWidget
export QSG_RHI_BACKEND=opengl    # Qt 6 may default to Vulkan

./build/gui/gui_for_openeb
```

### Camera Vendor Paths

| Vendor | HAL plugin path |
|--------|----------------|
| Prophesee | `/usr/local/lib/metavision/hal/plugins` |
| CenturyArks | `/usr/lib/CenturyArks/hal/plugins` |

---

## Troubleshooting

**Black screen on startup** — Use the launcher script. If launching manually, set `QT_QPA_PLATFORM=xcb` and `QSG_RHI_BACKEND=opengl`.

**Camera not detected** — Verify `MV_HAL_PLUGIN_PATH` matches your vendor. Run `metavision_hal_ls` to check.

**"NonMonotonicTimeHigh" error** — This is a transient Evt3 protocol warning that occurs ~50% of the time on some Gen3.x cameras at startup. EB plus treats it as non-fatal and keeps the stream running. No action needed.

**Dark mode not following system** — Requires Qt 6.5+. On older Qt, use Theme → Mode → Dark.

---

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+O` | Open file |
| `Ctrl+Shift+P` | Toggle playback panel |
| `F11` | Fullscreen |
| `Ctrl+Q` | Quit |

---

## Known Issues & Feedback

EB plus is under active development and may still contain bugs. If you encounter any issue — crashes, rendering glitches, broken controls, or unexpected behavior — please [open an issue](../../issues). Bug reports from real users are the most direct help.

---

## License

- **Original code**: [MIT](LICENSE)
- **openEB SDK**: [Apache 2.0](openeb/licensing/LICENSE_OPEN) — copyright Prophesee

---

<div align="center">

Built with Qt 6 · OpenCV · openEB SDK

</div>
