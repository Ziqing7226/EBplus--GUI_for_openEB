# EB plus Wiki

Welcome to the wiki for **EB plus** — a polished, open-source Qt 6 desktop GUI for event cameras, built on [openEB](https://github.com/prophesee-ai/openeb) v5.2.0.

EB plus gives you a complete desktop workflow for event-camera data: real-time visualization, camera control, recording & playback, 35 algorithms (28 self-developed + 7 OpenEB-wrapped), calibration, and data export.

---

## Pages

| Page | What's inside |
|------|---------------|
| [Getting Started](Getting-Started.md) | Build, run, environment variables, troubleshooting |
| [GUI Guide](GUI-Guide.md) | VSCode-style sidebar, panels, display modes, theming, shortcuts |
| [Algorithms](Algorithms.md) | Algorithm registry, categories, E2VID, preprocessing, noise filter |
| [Architecture](Architecture.md) | Directory layout, AlgoBridge, data flow, extension points |

---

## At a Glance

- **Platform**: Linux (Ubuntu 22.04+), Qt 6, OpenCV 4, C++17
- **Cameras**: Prophesee / CenturyArks event cameras via openEB HAL
- **Display**: OpenGL 3.3 core, 7 frame modes, 4 palettes, 60+ FPS
- **Algorithms**: 59 total (filtering, motion, detection, tracking, reconstruction, analytics, visualization, calibration)
- **E2VID**: Deep-learning event-to-video reconstruction via ONNX Runtime (default mode)
- **Themes**: 5 colors × 3 modes (Follow System / Light / Dark)
- **License**: MIT (original code) + Apache 2.0 (openEB SDK)

## Camera Vendors

| Vendor | HAL plugin path |
|--------|----------------|
| Prophesee | `/usr/local/lib/metavision/hal/plugins` |
| CenturyArks | `/usr/lib/CenturyArks/hal/plugins` |

Set `MV_HAL_PLUGIN_PATH` to match your camera before launching.
