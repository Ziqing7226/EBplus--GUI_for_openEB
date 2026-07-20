# GUI Guide

EB plus uses a VSCode-style layout: a custom title bar with dropdown menus, a left sidebar (ActivityBar + stacked panels), a central OpenGL display, and a right-side algorithm display area. There is no classic menu bar or toolbar — all configuration lives in the sidebar.

## Title Bar

The `CustomTitleBar` (36 px tall) shows the "EB plus" chip on the left, followed by dropdown menus:

| Menu | Contents |
|------|----------|
| **File** | Open File, Open Recent, Save/Load Camera Config, Save/Load Biases, Save/Load Algo Params, Exit |
| **View** | Toggle Playback Panel, Reset/Save/Load Layout, Fullscreen |
| **Theme** | Color submenu (5 colors) + Mode submenu (Follow System / Light / Dark) |
| **Tools** | Intrinsic Wizard (calibration), Sharpness |
| **Help** | About, About Qt |

Window control buttons (minimize / maximize / close) are on the right. The title bar follows the active theme.

## Sidebar (ActivityBar)

The left sidebar is a 48 px icon column (`ActivityBar`) that switches between 5 panel groups via `QStackedWidget`. Each group hosts one or more panels in a scrollable page.

| Group | Icon | Tooltip | Panels |
|-------|------|---------|--------|
| **Camera** | `camera` | Camera devices and connection info | Devices, Information |
| **Display & Stats** | `chart` | Display settings and statistics | Display, Statistics |
| **Hardware** | `cpu` | Biases, ROI, ESP and trigger configuration | Biases, ROI, ESP, Trigger, Preprocessing |
| **Algorithms** | `blocks` | Algorithm selection and preprocessing | File Tools, Algorithms |
| **Tools** | `tools` | File conversion and tools | (Calibration wizard placeholder) |

- The sidebar can be collapsed to 48 px (icon-only) or expanded to the default 380 px.
- Drag the blank area of the ActivityBar to resize (cursor feedback: open hand → closed hand).
- "Reset Layout" expands the sidebar if it was collapsed.

## Panels (11 total)

### Camera group
- **Devices** — camera discovery, connect/disconnect, refresh. Shows available cameras and connection status.
- **Information** — sensor metadata: model, resolution, serial number, firmware version, generation.

### Display & Stats group
- **Display** — accumulation time (1–1 000 000 us, exponential slider + spinbox), frame rate (1–60 fps), FPS limit (1–1000), color palette (Dark / Light / CoolWarm / Gray).
- **Statistics** — live event rate, peak rate, ON/OFF ratio, FPS, timestamp, and (in online camera mode) per-algorithm event drop rate.

### Hardware group
- **Biases** — dynamically enumerates all HAL LL-biases; slider + spinbox + reset per bias; save/load `.bias` files.
- **ROI** — multi-rectangle ROI / RONI via `I_ROI`; drag-to-select on the display; apply/clear.
- **ESP** — Anti-Flicker (mode / band / presets / duty cycle / threshold), Trail Filter (type / threshold), ERC (target event rate).
- **Trigger** — Trigger In (per-channel enable) + Trigger Out (enable / period / duty cycle).
- **Preprocessing** — 8-stage filter chain (see [Preprocessing](#preprocessing-filter-chain)).

All hardware panels auto-disable during file playback (no HAL facility available) and degrade gracefully when a device lacks a facility.

### Algorithms group
- **File Tools** — RAW recording, file cutter, format conversion (RAW ↔ HDF5 ↔ CSV), AVI export.
- **Algorithms** — algorithm selection + shared preprocessing (ROI, noise filter, 1/4 downsample, undistort) + per-algorithm parameters. See [Algorithms](Algorithms.md).

## Display

The central `EventDisplayWidget` is an OpenGL 3.3 core-profile widget with a letterboxed viewport (preserves aspect ratio). It renders events using one of 7 frame modes:

| Frame Mode | Description |
|------------|-------------|
| Integration | Time-integrated event accumulation with decay |
| Diff | Frame-to-frame event difference |
| Histogram | ON/OFF event count histogram |
| Time Decay | Exponential decay visualization |
| Contrast Map | ON-OFF contrast difference |
| Periodic | Fixed-period frame generation |
| On-Demand | Manual/snapshot frame |

Color palettes: Dark, Light, CoolWarm, Gray.

The display also supports overlays drawn by algorithms (bounding boxes, trajectories, vectors, arrows) via `FrameAnnotator`. When an in-place event filter (e.g. Hot Pixel Filter) is enabled, the main display switches to rendering that algorithm's output event stream so the filtering effect is directly visible.

## Preprocessing Filter Chain

Thread-safe pipeline of 8 stackable stages, toggled from the Preprocessing panel. Applied in order:

1. Polarity Filter (OFF / ON)
2. Polarity Invert
3. Flip X
4. Flip Y
5. Rotate (0 / 90 / 180 / 270)
6. Transpose
7. Rescale (Scale X, Scale Y)
8. ROI Filter (X0, Y0, X1, Y1)

The filter chain is applied to both display rendering and algorithm event windows, so flipped/rotated/filtered events stay consistent everywhere.

## Tools Menu

The **Tools** dropdown menu (in the custom title bar) hosts two calibration-adjacent utilities:

### Intrinsic Wizard

A dialog that calibrates the camera intrinsics using only events (no APS frames). Workflow:

1. **Show Chessboard** — opens an independent fullscreen window displaying a black-and-white chessboard that inverts at 20 Hz (one flip every 50 ms). The board geometry is computed from the target screen's pixel dimensions and physical DPI, so the reported square size in millimeters is physically meaningful for `cv::calibrateCamera`'s object-point scale. Press **F** to toggle fullscreen, **Esc** to close.
2. **Start Auto-Capture** — the wizard subscribes to CD events via `CameraController::cd_events_ready`, accumulates them in a 1 ms / 50 ms windowed buffer, and on each 50 ms tick picks the 1 ms sub-window with the most events (the one aligned with the chessboard flip burst). It renders that window to a grayscale frame (ON events white, OFF black, background grey), runs `cv::findChessboardCorners`, and rejects duplicates via MSE against the last accepted frame (default threshold 50.0).
3. **Auto-end + Export** — when the captured frame count reaches the target (default 30), the wizard stops capture, runs `cv::calibrateCamera`, and enables the **Export...** button. The export dialog defaults to `~/Documents/EBplus/calibration/intrinsic.yml` (identical to the undistort preprocessor's default path) and writes `image_width`, `image_height`, `camera_matrix`, `distortion_coefficients`, `rms`.

Controls: target screen selector, inner-corner cols/rows (default 9×6), target frames (default 30), duplicate MSE threshold (default 50.0), progress bar, live preview of the last accepted frame.

### Sharpness

A dialog that plots the **variance of Laplacian** of the current event visualization frame as a rolling 2-second line chart, polled at 10 Hz. Higher values = sharper. Useful for bias tuning and lens focus: point the camera at a high-contrast static scene and watch the line climb as focus improves.

The chart's Y-axis uses a **fixed ceiling** computed from the current frame resolution rather than auto-scaling: the algorithm estimates the theoretical maximum variance for an ideal single-pixel checkerboard under `cv::Laplacian` (ksize=3, `BORDER_REPLICATE`) — `σ²_max = [n_int·1020² + n_edge·765² + n_corner·510²] / (W·H)` — so the scale stays stable across time and the line never jumps as the data range shifts. The ceiling converges to `1020² = 1 040 400` for large sensors and is recomputed each tick from the live frame dimensions.

### Undistort Preprocessing

Not a menu item, but related: in the **Algorithms** panel's Preprocessing group, the **Undistort (apply calibration)** checkbox loads the YAML written by the Intrinsic Wizard and applies a forward event-address LUT (via `cv::undistortPoints`, with K adjusted for the ROI origin and downsample factor) to every event after filter + downsample. Default path: `~/Documents/EBplus/calibration/intrinsic.yml` — identical to the wizard's default export path, so the two defaults always point at the same file. Click **Browse...** to point at any other YAML.

## Recording & Playback

- **RAW recording** — record live camera streams to `.raw` with real-time buffer flushing.
- **Playback** — open `.raw` files; speed control, seek, pause/resume, position tracking. Playback window displays integer microseconds (no scientific notation); playback rate shows 6 decimal places.
- **Loop playback** — cyclic playback; algorithm temporal state resets on each loop to avoid frozen output.
- **File cutter** — extract a time range from an event file.

The playback dock can be toggled with `Ctrl+Shift+P`.

## Export & Conversion

Available from the File Tools panel:

- **Format conversion**: RAW ↔ HDF5 ↔ CSV (background worker thread).
- **AVI export**: render events to a video file via `CDFrameGenerator` + `CvVideoRecorder`. Configurable FPS, accumulation time, quality, color mode.

## Theming

- **5 background colors**: Gray, Green, Yellow, Pink, Blue (default).
- **3 modes**: Follow System (default, requires Qt 6.5+), Always Light, Always Dark.
- Dark mode uses a **dark variant of the chosen color** — not pure black.
- Text color auto-adjusts (black on light, white on dark).
- The "EB plus" title chip uses inverse colors relative to the background for high contrast.
- Settings persist across restarts (`QSettings`).
- Theme changes apply immediately (style unpolish/polish).

## Multi-Window

- **XYT 3D point cloud** — GPU-accelerated 3D event visualization (`SpaceTimeDisplay`, VBO + GLSL).
- **Algorithm display windows** — `AlgoWindow` dockable windows showing algorithm title + output only (no parameters — those live in the sidebar).
- **Layout persistence** — save/restore dock geometry and window positions to JSON (View → Save/Load Layout).

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+O` | Open file |
| `Ctrl+Shift+P` | Toggle playback panel |
| `F11` | Fullscreen |
| `Ctrl+Q` | Quit |

## Configuration Files

- **`.bias`** — camera bias presets (save/load from File menu or Biases panel).
- **Algo params JSON** — per-algorithm parameter snapshots (File → Save/Load Algo Params).
- **Layout JSON** — dock/window geometry (View → Save/Load Layout).
- **QSettings** — theme color/mode, sidebar state, recent files.
