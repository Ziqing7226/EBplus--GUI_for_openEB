# Getting Started

## Prerequisites

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04+ (or compatible Linux) |
| Compiler | GCC 13+ (GCC 15 supported, see [wiki/compile.md](https://github.com/Ziqing7226/EBplus--GUI_for_openEB/blob/main/wiki/compile.md) for the `<cstdint>` fix) |
| CMake | 3.16+ |
| Qt | 6.x (Widgets, OpenGL, OpenGLWidgets) |
| OpenCV | 4.x |
| openEB SDK | 5.2.0 (included as a subtree under `openeb/`) |

See [wiki/compile.md](https://github.com/Ziqing7226/EBplus--GUI_for_openEB/blob/main/wiki/compile.md) for the full build walkthrough (including Python 3.12 setup for building openEB from source, and the GCC 15 compatibility fix).

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j$(nproc)
```

The binary is output to `build/gui/gui_for_openeb`.

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `GUI_BUILD_TESTS` | `ON` | Build GUI unit tests (`gui/tests/`) |
| `CMAKE_BUILD_TYPE` | `Release` | Recommended `Release` |

CMake auto-detects Qt6, OpenCV, openEB SDK, and ONNX Runtime (if placed in `third_party/onnxruntime/`).

## Run

### Option 1: Launcher (Recommended)

```bash
./run.sh
```

`run.sh` automatically:
- Sets `LD_LIBRARY_PATH` to include `/usr/local/lib`
- Sets `HDF5_PLUGIN_PATH` for HDF5 file support
- Sets `MV_HAL_PLUGIN_PATH` (Prophesee default; override for CenturyArks)
- Forces `QT_QPA_PLATFORM=xcb` on Wayland sessions (avoids black viewport)
- Forces `QSG_RHI_BACKEND=opengl` (avoids Vulkan black screen)

To customize for your camera, copy the script:

```bash
cp run.sh run.local.sh
# edit run.local.sh — change MV_HAL_PLUGIN_PATH etc.
./run.local.sh
```

`run.local.sh` is git-ignored.

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

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `MV_HAL_PLUGIN_PATH` | Camera HAL plugin directory | `/usr/local/lib/metavision/hal/plugins` |
| `HDF5_PLUGIN_PATH` | HDF5 plugin directory (for `.hdf5` files) | `/usr/local/lib/hdf5/plugin` |
| `LD_LIBRARY_PATH` | SDK shared library search path | must include `/usr/local/lib` |
| `QT_QPA_PLATFORM` | Qt platform plugin | `xcb` on Wayland; unset otherwise |
| `QSG_RHI_BACKEND` | Qt RHI backend | `opengl` |

> **Wayland note**: Qt 6's Wayland plugin renders a black viewport for `QOpenGLWidget` children. The app and launcher force `QT_QPA_PLATFORM=xcb` (via XWayland) and `QSG_RHI_BACKEND=opengl` to ensure correct rendering.

## Verify Camera Detection

```bash
metavision_hal_ls
```

If this fails, the SDK cannot find your vendor's HAL plugins — check `MV_HAL_PLUGIN_PATH`.

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Black screen on startup | Wayland + Qt 6 rendering issue | Use `./run.sh`, or set `QT_QPA_PLATFORM=xcb` and `QSG_RHI_BACKEND=opengl` |
| Camera not detected | Wrong HAL plugin path | Set `MV_HAL_PLUGIN_PATH` to your vendor's path |
| "GUI shows no camera" but `metavision_hal_ls` works | App default path doesn't match vendor | Export `MV_HAL_PLUGIN_PATH` before launching |
| `NonMonotonicTimeHigh` warning | Transient Evt3 protocol warning on some Gen3.x cameras at startup | Non-fatal; EB plus keeps the stream running. No action needed |
| HDF5 file open fails | HDF5 plugin path not set | Set `HDF5_PLUGIN_PATH` to the HDF5 plugin directory |
| Dark mode not following system | Qt < 6.5 | Use Theme → Mode → Dark |
| E2VID falls back to heuristic mode | ONNX Runtime not installed | See [Algorithms § E2VID](Algorithms.md#e2vid-setup) |

## Tests

```bash
cd build
ctest --output-on-failure
```

Test suites:
- `gui/tests/`: 5 executables, 40 `TEST()` macros (algo_bridge, config_manager, display_strategy, layout_manager, theme_tokens)
- `algo/tests/`: 4 executables, 288 `TEST()`/`TEST_F()` macros (phase6_common, phase7_cv, phase8_10, raw_algos)
