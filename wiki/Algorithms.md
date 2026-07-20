# Algorithms

EB plus ships **36 algorithms** registered in a single `AlgoBridge` registry (`gui/algo_bridge/algo_bridge.cpp`):

- **28 self-developed** algorithms under `algo/` (20 Computer Vision + 8 Analytics, including `sensor_self_test`). `intrinsic_calibration` was removed from the registry — it is now a **Tools → Intrinsic Wizard** dialog. `perspective_undistort` was also removed — undistortion is available as a stackable preprocessing checkbox in the Algorithms panel (`preproc_undistort_enabled` / `preproc_undistort_path`).
- **8 OpenEB-wrapped** filter/transform stages (`polarity_filter`, `polarity_invert`, `flip_x`, `flip_y`, `rotate`, `transpose`, `rescale`, `roi_filter`) — the actual processing lives in the `FilterChain` driven by the Preprocessing panel; the registry entries keep a stable namespace for config capture. The former `frame_*` / `preproc_*` / `util_*` registrations were removed: they had no GUI entry point and were unreachable dead code.

Algorithms are **mutually exclusive** — enabling one disables the previous. Each self-developed algorithm supports a global ROI (default: center 128×128) and a shared preprocessing stage. All parameters are adjusted exclusively in the sidebar's **Algorithms** panel; algorithm display windows show only the title and output.

## Display Modes

Each algorithm declares a display mode that controls how its output reaches the screen:

| Mode | Behavior |
|------|----------|
| **Passive** | No overlay; for in-place event filters (e.g. Hot Pixel Filter) the main display switches to rendering the algorithm's **output event stream** so the filtering effect is directly visible |
| **Overlay** | Draws annotations on top of the live event display (trajectories, vectors, boxes) |
| **Replace** | Replaces the event display with the algorithm's own frame |
| **Standalone** | Opens a separate `AlgoWindow` with its own output canvas |

## Shared Preprocessing

Every self-developed algorithm runs events through a stackable preprocessing stage, configured in the Algorithms panel:

```
ROI (default 128×128 center)  →  Noise Filter  →  1/4 Downsample  →  Undistort
```

- **ROI** — bounds computational cost; default center 128×128.
- **Noise Filter** — 8 modes (see below), exposed based on the selected mode; default OFF.
- **1/4 Downsample** — default **OFF**. Only the E2VID/ISI/TimeSurface/Hough backends halve coordinates (128×128 → 64×64, output upsampled back); for the other backends it merely *thins* events (coordinates unchanged, 3/4 of the input discarded), which is why it is opt-in.
- **Undistort** — default OFF; loads the calibration YAML written by the Intrinsic Wizard and remaps event coordinates (see [GUI Guide](GUI-Guide.md#undistort-preprocessing)).

These stages are **not** mutually exclusive with algorithms — they stack on top.

### Noise Filter Modes

Implemented in `algo/cv/noise_filter.h`. The GUI exposes parameters based on the selected mode:

| Mode | Description |
|------|-------------|
| BAF | Background Activity Filter |
| STCF | Spatio-Temporal Correlation Filter |
| Refractory | Refractory period filter |
| DWF | Directional Weighted Filter |
| AgePolarity | Age + polarity filter |
| Harmonic | Harmonic mean filter |
| Repetitious | Repetitious event filter |
| SpatialBP | Spatial Band-Pass filter |

## Self-Developed Algorithms

### Computer Vision (20)

| Algorithm | Display | Notes |
|-----------|---------|-------|
| Hot Pixel Filter | Passive | FPN correction option; main display renders the filtered stream |
| Orientation Filter | Overlay | jAER SimpleOrientationFilter (min-dt WTA) |
| Direction Selective Filter | Overlay | jAER DirectionSelectiveFilter |
| Sparse Optical Flow | Overlay | 4 modes: LocalPlanes / LucasKanade / BlockMatch / ClusterOF |
| Blob Detector | Overlay | |
| Object Tracker | Overlay | 4 modes: RCT / Median / Kalman / MultiHypothesis |
| Corner Detector | Overlay | 3 modes: EndStopped / TypeCoincidence / Harris |
| Line Segment (ELiSeD) | Overlay | |
| Hough Line Tracker | Overlay | jAER HoughLineTracker |
| Hough Circle Tracker | Overlay | jAER HoughCircleTracker |
| Orientation Cluster | Overlay | jAER OrientationCluster (per-event orientation filter) |
| Cluster LIF | Overlay | LIF neuron clustering |
| Background Mask Filter | Replace | 2D histogram background modeling |
| Trigger Synced Filter | Passive | Requires an external trigger source (none wired — output empty) |
| Bandpass Filter | Overlay | |
| EIS (Optical Gyro) | Overlay | Electronic image stabilization |
| Ultra Slow Motion | Passive | Time dilation |
| XYT 3D Visualizer | Standalone | GPU 3D point cloud |
| Overlay | Overlay | Generic overlay |
| Time Surface | Standalone | Hot / Plasma / Turbo palettes |

### Analytics (7)

| Algorithm | Display | Notes |
|-----------|---------|-------|
| Active Marker Tracking | Overlay | Sliding-window clustering |
| Event -> Video (E2VID) | Standalone | 3 modes (see below) |
| Flow Statistics | Passive | Requires ground-truth |
| ISI Analyzer | Standalone | Inter-spike interval histograms |
| Particle Counter | Overlay | Line-crossing counter |
| Auto Bias Controller | Overlay | Closed-loop bias tuning |
| Frequency Detector | Standalone | Blinking frequency detection |

> An 8th analytics registration, **Sensor Self-Test**, is a full-sensor diagnostic triggered from the Devices panel (no ROI/preprocessing, no tunable params) — it does not appear in the algorithm list.

### Calibration

Not a registered algorithm: intrinsic calibration runs as the **Tools → Intrinsic Wizard** dialog (event-only flickering chessboard), and undistortion is a shared preprocessing stage. See [GUI Guide](GUI-Guide.md#tools-menu).

## Event-to-Video (E2VID)

The Event-to-Video algorithm reconstructs grayscale intensity images from raw event streams. It has **3 modes**, selected via the `mode` parameter:

| Mode | Default | Description |
|------|---------|-------------|
| `0 = BardowVariational` | | Non-DL; joint optical-flow + log-intensity variational optimization |
| `1 = InteractingMaps` | | Non-DL; six interconnected maps (I/G/V/F/C/R) with rotation estimation |
| `2 = E2VID` | ✅ | DL; ONNX Runtime neural-network inference |

**Common parameters**: `output_fps` (1–120, default 30); `window_ms` (modes 0, 1). The per-algorithm `downsample` and `decay_tau_ms` parameters were removed — downsampling is now the shared `preproc_downsample` stage (default OFF), and the decay semantics were ineffective.

### E2VID Setup

E2VID (default mode) requires a converted ONNX model. One-time setup (~5 minutes):

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

On first build the GUI auto-sets a 128×128 center ROI + 24 fps for all three modes. Enable the shared **1/4 Downsample** preprocessing stage (default OFF) to reconstruct at 64×64 → upsampled back to 128×128 (~4× less compute). `num_bins` is auto-determined by the ONNX model's input channel count when a model is loaded.

E2VID inference runs on a **background worker thread** — the UI no longer freezes during reconstruction. When inference is slower than the event rate, the oldest queued batches are dropped; the status line shows `dropped=N` and `model=loaded` / `model=heuristic`.

E2VID parameters exposed in the GUI: model path, `num_bins`, auto-HDR, unsharp amount/sigma, bilateral sigma.

> **Without ONNX Runtime**: E2VID falls back to a heuristic mode (voxel-grid sum + sigmoid). BardowVariational and InteractingMaps work without any extra setup.

### BardowVariational (mode 0)

Joint estimation of optical flow `u` and log-brightness `L` via Chambolle-Pock primal-dual optimization. Uses all six regularization terms:
- `lambda1` (flow TV), `lambda2` (temporal smoothness), `lambda3` (intensity TV)
- `lambda4` (flow constraint), `lambda5` (no-event dead zone), `lambda6` (prior map)

`lambda3` and `num_iterations` are shared with InteractingMaps.

### InteractingMaps (mode 1)

Six interconnected maps updated by alternating relaxation:
- **I** intensity, **G** gradient (= ∇I), **V** time-varying (−V = F·G), **F** optical flow, **C** calibration, **R** rotation (estimated via linear least squares).

V values are clamped to [-1, 1] to prevent NaN divergence. `I_map_` is reinitialized from Vc every frame to prevent ghosting.

## OpenEB-Wrapped Stages (8)

Registered in `AlgoBridge` under the `openeb_filter` category — these wrap existing openEB SDK event filters/transforms without reimplementation. The actual processing lives in the thread-safe `FilterChain` and is toggled from the **Preprocessing** panel (see [GUI Guide](GUI-Guide.md#preprocessing-filter-chain)); the registry entries exist so the stages appear in config capture and keep a stable namespace:

| Stage | Notes |
|-------|-------|
| Polarity Filter | ON / OFF only |
| Polarity Invert | |
| Flip X / Flip Y | |
| Rotate | 0 / 90 / 180 / 270 |
| Transpose | |
| Rescale | Scale X / Scale Y |
| ROI Filter | X0, Y0, X1, Y1, relative-coords option |

The former `openeb_frame` (7), `openeb_preproc` (7) and `openeb_util` (6) registrations were removed together with their backends — they had no GUI entry point and were unreachable dead code. Some of those SDK capabilities are still used internally (e.g. `CDFrameGenerator` for display/AVI export, `TimeSurfaceProcessor` for Time Surface, `CvVideoRecorder` for AVI export).

## Adding a New Algorithm

1. Implement the algorithm in `algo/cv/` or `algo/analytics/` (header-only, operate on event packets).
2. Register it in `AlgoBridge::register_self_cv()` or `register_self_analytics()` with `AlgoInfo` (name, display name, category, display mode, parameters).
3. Implement the backend in `gui/algo_bridge/backends/` (e.g. `cv_backends.cpp`) — wire `set_param` / `get_param` / `process`.
4. If the backend has a factory entry, register it in `backend_registry.h`.
5. The algorithm auto-appears in the Algorithms panel; parameters are read/written via `set_param` / `get_param`.

All algorithms must have both `set_param` and `get_param` implementations so the GUI can read and write values.
