// gui/calibration/sharpness_metrics.h — pure sharpness metrics on event
// count images (systematic_audit §9.3, fixes R1/R2).
//
// These functions are deliberately free of any Qt dependency so they can be
// unit-tested headlessly (gui/tests/test_sharpness_metrics.cpp). The sharpness
// dialog builds a count image (per-pixel event counts, polarity ignored, no
// 8-bit saturation) from the raw CD stream and passes it here.
//
// Metrics (audit §9.3 S2/S3/S4):
//   - isolated-pixel denoise (S2): kills hot pixels before any gradient-style
//     measurement, so noise density no longer dominates the metric (R1).
//   - normalized contrast (S3): σ²(I)/μ(I)² — the CMax-style sharpness proxy.
//     First-order invariant to event rate / window length (R2), higher =
//     sharper.
//   - mean line width (S4): distance-transform based average stroke width of
//     the thresholded count image, in pixels. Lower = sharper.

#ifndef GUI_CALIBRATION_SHARPNESS_METRICS_H
#define GUI_CALIBRATION_SHARPNESS_METRICS_H

#include <opencv2/core.hpp>

namespace gui {

/// @brief Sharpness metrics computed from one accumulated count image.
struct SharpnessMetrics {
    /// @brief σ²(I)/μ(I)² of the denoised count image (coefficient of
    /// variation squared). NaN when valid == false.
    double contrast;
    /// @brief Mean stroke width of the binarized count image in pixels
    /// (2 × mean distance-to-background over edge pixels). NaN when
    /// valid == false.
    double mean_line_width;
    /// @brief total_events / window_s. 0 when valid == false.
    double event_rate;
    /// @brief false when the image was empty or contained zero events —
    /// callers should display "—" instead of the metric values.
    bool valid;
};

/// @brief Returns a CV_32F copy of @p count_image with isolated pixels
/// removed: any pixel with 0 < count <= @p max_count whose entire 8-neighbourhood
/// is empty is set to 0. Designed for sparse event count images, where hot
/// pixels show up as lone count-1 specks far from real edges (audit §9.3 S2).
/// Accepts CV_32F or CV_16U input; returns an empty Mat for empty input.
cv::Mat remove_isolated_pixels(const cv::Mat& count_image, float max_count = 1.0f);

/// @brief Computes all sharpness metrics for one accumulation window.
/// @param count_image per-pixel event counts (CV_32F or CV_16U), any polarity
///        mixed; must not be saturated/rendered (no palette, no overlays).
/// @param window_s accumulation window length in seconds (for event_rate);
///        pass <= 0 to report event_rate = 0.
SharpnessMetrics compute_sharpness_metrics(const cv::Mat& count_image, double window_s);

} // namespace gui

#endif // GUI_CALIBRATION_SHARPNESS_METRICS_H
