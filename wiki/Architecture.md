# Architecture

EB plus is split into two top-level layers: the **GUI application** (`gui/`) and the **self-developed algorithm library** (`algo/`). The openEB SDK (`openeb/`, Apache 2.0) is included as a subtree and provides the camera HAL, event decoding, and 30 wrapped algorithms.

```
┌─────────────────────────────────────────────────────────┐
│                       gui/  (Qt 6)                       │
│  main_window ── widgets ── panels ── display ── recorder │
│        │                                  │              │
│        └──────── algo_bridge ─────────────┘              │
│                  │                                       │
│                  │ AlgoBackend (abstract)                │
│                  ├─ backends/*.cpp (self + openeb)       │
│                  └─ filter_chain                         │
└──────────────────┼──────────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │    algo/ (C++17)     │
        │  cv / analytics /    │
        │  calibration / common│
        └──────────┬──────────┘
                   │
        ┌──────────┴──────────┐
        │  openeb/ SDK v5.2.0  │
        │  HAL · Core · Base   │
        └─────────────────────┘
```

## Directory Layout

```
GUI-for-openEB/
├── gui/                  # Qt 6 application
│   ├── main.cpp              # entry point; env-var defaults, OpenGL format, font
│   ├── main_window.*         # main window: title-bar menus, docks, signal wiring
│   ├── widgets/              # CustomTitleBar, ActivityBar, AlgoWindow, pixel probe,
│   │                         #   target labeler, mouse adaptor
│   ├── panels/               # 11 sidebar panels (AbstractPanel base)
│   │   ├── abstract_panel.*      # base class: camera lifecycle decoupling
│   │   ├── settings_panel.*      # ActivityBar + QStackedWidget container
│   │   ├── devices_panel.*       # camera discovery/connection
│   │   ├── information_panel.*   # sensor metadata
│   │   ├── display_panel.*       # frame mode/fps/palette
│   │   ├── statistics_panel.*    # event rate / drop rate / FPS
│   │   ├── biases_panel.*        # LL-bias sliders
│   │   ├── roi_panel.*           # multi-rect ROI / RONI
│   │   ├── esp_panel.*           # Anti-Flicker / Trail / ERC
│   │   ├── trigger_panel.*       # Trigger In / Out
│   │   ├── preprocessing_panel.* # 8-stage filter chain
│   │   ├── algorithms_panel.*    # algorithm selection + shared preproc + params
│   │   └── file_tools_panel.*    # recording / conversion / export
│   ├── display/              # OpenGL rendering
│   │   ├── event_display_widget.* # QOpenGLWidget, GLSL 3.30 core
│   │   ├── display_strategy.*     # IDisplayStrategy: Passive/Overlay/Replace/Standalone
│   │   ├── frame_annotator.*      # bbox/ID/trajectory/arrow overlays
│   │   └── space_time_display.*   # XYT 3D point cloud (VBO + GLSL)
│   ├── app/                  # controllers
│   │   ├── camera_controller.*    # camera lifecycle, HAL facility access
│   │   ├── frame_pipeline.*       # CD events → QImage rendering
│   │   ├── file_frame_generator.* # file-source frame generation + loop/flip
│   │   ├── statistics_controller.*# event-rate computation
│   │   ├── file_converter.*       # background RAW/HDF5/CSV conversion
│   │   ├── icon_provider.*        # SVG icon cache (theme-adaptive)
│   │   └── theme_controller.*     # 5 colors × 3 modes
│   ├── algo_bridge/          # algorithm registry + filter chain
│   │   ├── algo_bridge.*          # AlgoBridge: registry, list_algos(), enable()
│   │   ├── algo_backend.h         # AlgoBackend base + AlgoResult + AlgoInfo
│   │   ├── filter_chain.*         # thread-safe 8-stage preprocessing
│   │   └── backends/              # backend implementations
│   │       ├── backend_registry.h     # factory map
│   │       ├── backend_factory.cpp    # factory wiring
│   │       ├── backend_common.h       # shared param helpers (pint/pfloat/penum/...)
│   │       ├── cv_backends.cpp        # self CV algorithm backends
│   │       ├── cv_vector_backends.cpp # vector-output CV backends
│   │       ├── analytics_backends.cpp # analytics backends
│   │       ├── analytics_extra_backends.cpp
│   │       ├── display_backends.cpp   # display-mode wiring
│   │       ├── filter_backends.cpp    # self filter backends
│   │       ├── openeb_filter_backends.cpp    # openEB filter wrappers
│   │       ├── openeb_frame_backends.cpp     # openEB frame-mode wrappers
│   │       ├── openeb_preproc_backends.cpp  # openEB preprocessor wrappers
│   │       └── openeb_util_backends.cpp      # openEB utility wrappers
│   ├── recorder/             # RAW recording & playback
│   │   ├── recorder_controller.*
│   │   ├── playback_controller.*
│   │   └── playback_controls.*
│   ├── exporter/             # HDF5/CSV/AVI export
│   ├── calibration/          # intrinsic wizard
│   ├── config/               # JSON config + layout persistence
│   │   ├── config_manager.*
│   │   └── layout_manager.*
│   ├── resources/            # Qt resources (compiled in)
│   │   ├── theme/            #   tokens.h + base.qss.in
│   │   ├── icons/            #   Lucide-style SVG icons
│   │   ├── theme.qrc
│   │   └── icons.qrc
│   └── tests/                # GUI unit tests (GTest + CTest)
├── algo/                  # self-developed algorithm library (29 registered)
│   ├── common/               # event packets, frame generator, filters, Kalman, LIF, ...
│   ├── cv/                   # 21 CV algorithms + noise_filter (8 modes)
│   ├── analytics/            # 7 analytics algorithms + e2vid/ ONNX inference
│   ├── calibration/          # intrinsic calibration
│   └── tests/                # algorithm tests (288 TEST() macros)
├── openeb/                # openEB SDK (Apache 2.0, v5.2.0)
├── models/                # E2VID PyTorch → ONNX conversion (convert_to_onnx.py)
├── third_party/           # ONNX Runtime (user-installed, git-ignored)
├── wiki/                  # this wiki
├── pic/                   # screenshots
├── run.sh                 # launcher (env vars)
├── CMakeLists.txt         # v2.7.0
├── LICENSE                # MIT (original code)
├── README.md              # English
└── README_CN.md           # Chinese
```

## Key Abstractions

### AlgoBridge

The central algorithm registry (`gui/algo_bridge/algo_bridge.cpp`). Holds a `std::unordered_map<std::string, AlgoInfo>` of all 59 algorithms. Each entry has:

- `name` — registry key (e.g. `"object_tracker"`)
- `display_name` — UI label (e.g. `"Object Tracker"`)
- `category` — `"cv"` / `"analytics"` / `"calibration"` / `"openeb_*"`
- `source` — `"self"` or `"openeb"`
- `display_mode` — `Passive` / `Overlay` / `Replace` / `Standalone`
- `params` — parameter metadata (name, label, type, default, min, max, enum options, mode_filter)

`list_algos()` enumerates all; `enable(name)` activates one (disabling others). The GUI reads/writes parameters via `set_param` / `get_param`.

### AlgoBackend

Abstract base (`algo_backend.h`) implemented by each backend in `backends/`. Defines `set_param` / `get_param` / `process` / `reset` / `result`. The bridge owns one `AlgoInstance` per active algorithm, running on the GUI thread (online camera slow algorithms use an async worker thread that discards stale event batches).

### IDisplayStrategy

Four strategies (`display_strategy.h`) controlling how an algorithm's `AlgoResult` reaches the display: `Passive` (nothing), `Overlay` (annotate the live frame), `Replace` (swap the frame), `Standalone` (open an `AlgoWindow`).

### FilterChain

Thread-safe 8-stage preprocessing pipeline (`filter_chain.h`). Applied at render time to both the display and algorithm event windows. Mutex-protected; toggled from the Preprocessing panel.

### AbstractPanel

Base class for all sidebar panels (`panels/abstract_panel.*`). Decouples panels from camera lifecycle — panels react to camera start/stop signals rather than holding direct camera references, so they work correctly across camera/file-mode switches.

## Data Flow

### Online camera mode

```
Camera (HAL) → I_EventsStream callback → FramePipeline
    → FilterChain (display path) → EventDisplayWidget (OpenGL)
    → AlgoBridge.process(events) → AlgoResult → IDisplayStrategy → display/AlgoWindow
```

- Playback rate is locked to 1.
- Slow algorithms (e.g. E2VID without ROI/downsample) run on an async worker thread; old pending event batches are discarded so the display shows recent frames.
- Event drop rate (`total_dropped / total_pushed`) is computed per algorithm instance and shown in the Information/Statistics panel.

### File playback mode

```
RAW file → FileFrameGenerator → FramePipeline
    → FilterChain (applied to window_events) → display + AlgoBridge
```

- Playback rate is auto-calculated; seek/pause/resume supported.
- Loop playback re-signals algorithm `reset()` to clear temporal state each iteration.
- FilterChain is applied to both display and algorithm event windows so flip/rotate/ROI stay consistent.

## Threading Model

- **GUI thread** — all panel interaction, display rendering, most algorithm processing.
- **SDK data thread** — openEB event-stream callback (FramePipeline). FilterChain is mutex-protected.
- **Async worker thread** — used by `AlgoInstance` for slow online-camera algorithms; discards stale batches.
- **File converter thread** — background RAW/HDF5/CSV conversion (`file_converter.cpp`).

## Configuration & Persistence

- `ConfigManager` (`config/config_manager.*`) — JSON serialization for algo params and camera config; versioned schema (`"version": 1`).
- `LayoutManager` (`config/layout_manager.*`) — dock/window geometry to JSON.
- `QSettings` — theme color/mode, sidebar state, recent files.
- `.bias` files — camera bias presets.

## Build System

- `CMakeLists.txt` (root) — project version 2.7.0, C++17, GCC 15 `<cstdint>` fix.
- `find_package` for Qt6, MetavisionSDK 5.2.0, OpenCV.
- ONNX Runtime auto-detected from `third_party/onnxruntime/` (with RPATH configured).
- `enable_testing()` before `add_subdirectory` so GUI/algo tests register with CTest.
- `gui/tests/` and `algo/tests/` use `gtest_discover_tests()`.
