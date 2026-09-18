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

## Live inivation DAVIS / DVXplorer Cameras (Optional, Preliminary)

**Preliminary support**: EB plus can connect to inivation **DAVIS346/640** and **DVXplorer** cameras directly over USB — **events + biases only**. APS frames, IMU samples and trigger markers are parsed and discarded (the GUI is an events-only tool); the RAW recording, ROI and Trigger panels are not available, and other inivation-family features may still have compatibility gaps. **EB plus remains primarily designed and tested for Prophesee cameras.**

Build requirement: `libusb-1.0` development files (the CMake build auto-detects them; without them the inivation device layer is compiled out and everything else works as before).

### One-Time USB Permission Setup

Inivation devices (USB VID `152a`) are not accessible to unprivileged users by default. The rules file ships with the repository:

```bash
sudo cp gui/davis/66-inivation.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then **unplug and replug the camera** (so the session permission tag applies), and connect it from the Devices panel (`Refresh` → `Connect First Available`, or by serial). inivation devices appear in the device list next to Prophesee ones.

### Supported & Not Supported (DAVIS / DVXplorer)

| Feature | DAVIS346/640 | DVXplorer |
|---------|--------------|-----------|
| Live event preview, all display modes | ✅ | ✅ |
| Biases panel (all DAVIS coarse/fine + VDAC biases), save/load `.bias` | ✅ | ✅ (`contrast_on`/`contrast_off` only) |
| Auto Bias controller (`diff_on`/`diff_off`) | ✅ | ❌ (no diff biases) |
| Algorithms (all), unified software ROI, statistics | ✅ | ✅ |
| RAW recording | ❌ (no `I_EventsStream`) | ❌ |
| ROI / Trigger / ESP panels | ROI ❌ · Trigger/ESP auto-hidden (no facilities) | Trigger/ESP auto-hidden (no facilities) |
| IMU stream (checkbox + live readout window) | ✅ | ✅ |
| APS frames (checkbox + live preview window) | ✅ (grayscale) | ❌ (no APS hardware) |
| APS frames / trigger streams | discarded by design | discarded by design |

Firmware requirements — DAVIS: FX3 firmware 6, FX2 firmware 4, FPGA logic version 18 patch ≥ 1; DVXplorer: FX3 firmware 9, FPGA logic version 18 patch ≥ 4 (checked at connect — a clear error is shown otherwise).

### DAVIS Bias Parameters

All DAVIS346/640 biases are exposed in the Biases panel (and to Auto Bias) with these ranges:

| Bias group | Panel range | Meaning |
|------------|-------------|---------|
| Coarse/fine biases (`diff`, `diff_on`, `diff_off`, `photoreceptor`, …) | 0 – 2047 | Linearized `coarse × 256 + fine` (coarse 0–7, fine 0–255) |
| VDAC biases (`aps_overflow_level`, `adc_reference_*`, …) | 0 – 63 | Voltage in 1/64 VDD steps (current index stays at its default) |

Every value is written to the camera's SPI bias registers on change; the structural properties of each bias (N/P type, normal/cascode, current level, enable) are fixed to the reference defaults and preserved across writes. Defaults follow the reference implementation's power-up table and are re-applied at every connect. A hardware round-trip test (`EBPLUS_DAVIS_HW=1 gui/tests/test_davis_protocol`) verifies every register write against an SPI readback.

This full-set exposure matches the reference ecosystem: jAER's own DAVIS346 bias settings expose the same 21 coarse/fine biases (including `ReadoutBufBP`, `ADCcompBp`, `DACBufBp`, `ColSelLowBn`, `PadFollBn`) and DV/libcaer registers all of them as adjustable device options. Bias name cross-reference (jAER → EB plus):

| jAER | EB plus | | jAER | EB plus |
|------|---------|--|------|---------|
| `DiffBn` | `diff` | | `PadFollBn` | `pad_follower` |
| `OnBn` | `diff_on` | | `PixInvBn` | `pixel_inverter` |
| `OffBn` | `diff_off` | | `PrBp` | `photoreceptor` |
| `LocalBufBn` | `local_buffer` | | `PrSFBp` | `photocircuit_follower` |
| `BiasBuffer` | `bias_buffer` | | `RefrBp` | `refractory` |
| `AEPdBn` | `aer_pull_down` | | `ReadoutBufBP` | `readout_buffer` |
| `AEPuXBp` | `aer_pull_up_x` | | `ApsROSFBn` | `aps_readout_follower` |
| `AEPuYBp` | `aer_pull_up_y` | | `ADCcompBp` | `adc_comparator` |
| `LcolTimeoutBn` | `lcol_timeout` | | `DACBufBp` | `dac_buffer` |
| `IFThrBn` | `if_thr_bn` | | `ColSelLowBn` | `col_select_low` |
| `IFRefrBn` | `if_refr_bn` | | | |

The five VDAC biases (`aps_overflow_level`, `aps_cascode`, `adc_reference_high`, `adc_reference_low`, `adc_test_voltage`) are likewise exposed by both references. The two shifted-source biases (SSP/SSN) are initialized to the reference defaults at connect but are not panel-adjustable — matching both dv-processing (init-only) and jAER.

Auto Bias on DAVIS homes toward the **reference default values** (e.g. `diff_on` → 1535, `diff_off` → 1025) rather than toward 0 — Prophesee diff biases are relative offsets whose default is 0, while DAVIS biases are absolute operating points.

Note that the OFF-axis responds with inverted polarity on DAVIS (higher `diff_off` → more OFF events, measured on hardware), which EB plus compensates automatically. Also note the default rate band (1–50 Mev/s) was chosen for Prophesee sensors; DAVIS346 at reference biases runs around 0.1–0.3 Mev/s, so consider a band like 0.05–1 Mev/s for DAVIS.

### DVXplorer Bias Parameters

DVXplorer exposes exactly two sensitivity parameters in the Biases panel: `contrast_on` and `contrast_off` (range 0–17 each). Higher values make the corresponding polarity fire on smaller brightness changes. Auto Bias does not attach to DVXplorer (there are no diff biases to tune). Values are written to the sensor's bias-current registers on change and re-applied at every connect; the 0–17 split maps to two current ranges internally (8 + high-range bit), with the OFF ladder inverted on the register level (reference behavior).

### Other inivation cameras

DAVIS240-family sensors and other unsupported device types are rejected at connect with a clear message. Extending support means porting the respective protocol/bias tables (reference available in `ref/dv-processing-master`).

### Why "DAVIS346" reports 260 × 346 internally

The DAVIS346's sensor die is mounted **rotated 90°** in the camera housing. The device size registers therefore report the sensor-native frame — 260 "columns" × 346 "rows" — together with the orientation bit `0x04` (invert axes). EB plus handles this transparently: event addresses are range-checked in the native frame and swapped for display, so the camera presents itself as **346 × 260** (the documented product resolution). If you probe the device registers directly, do not be surprised by the swapped values and the orientation bit — they are normal for this product.

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
| inivation connect: "failed to open USB device" | Missing udev rule, or device in use by another program (e.g. DV) | Install `gui/davis/66-inivation.rules` (see above), replug, close other apps |
| inivation connect: firmware/logic version error | Camera firmware too old for the protocol checks | Update with inivation's Flashy tool |

## Tests

```bash
cd build
ctest --output-on-failure
```

Test suites:
- `gui/tests/`: 5 executables, 40 `TEST()` macros (algo_bridge, config_manager, display_strategy, layout_manager, theme_tokens)
- `algo/tests/`: 4 executables, 288 `TEST()`/`TEST_F()` macros (phase6_common, phase7_cv, phase8_10, raw_algos)
