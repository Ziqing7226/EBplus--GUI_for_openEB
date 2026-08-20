// gui/algo_bridge/backends/backend_common.cpp
//
// Formerly held Preprocessor::rebuild_undistort_lut. The undistort stage
// moved to the shared StreamConditioner (gui/app/stream_conditioner.cpp)
// with the 2026-08-19 conditioning rework; the per-consumer Preprocessor no
// longer undistorts (it only halves coordinates), so this TU is now empty.

#include "backend_common.h"
