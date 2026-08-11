// algo/calibration/screwhead_detect.h — Zhou's Screw-Head Grid detector.
//
// Detects an asymmetric 6×5 grid of "screw-head" markers (a dashed cross that
// pins the centre + a solid thin ring that supplies polarity) in a three-valued
// event frame: background black, ON polarity = gold, OFF polarity = white, and
// a pixel hit by both polarities takes the simple average of gold and white.
//
// Four-stage pipeline:
//   1. The frame is already accumulated by the caller (CalibrationWizard).
//   2. Cross-centre candidates: 8-neighbour denoise → isotropic dilate (bridges
//      intra-cross dot gaps) → connected components + area filter → density peak
//      within each component (the cross, where two arms meet, is the densest
//      spot, so its peak is the centre — unbiased by partial ring arcs).
//   3. Ring localisation + confidence: per candidate, a radial histogram finds
//      the ring radius R* and checks angular coverage (≥ 0.60). Cross and ring
//      are PEER detections — detecting EITHER means a possible centre was found.
//      Each contributes 50% confidence; the 8-neighbourhood of the final centre
//      adds 20%. A centre with both features (weight ≈ 1.0–1.2) is most reliable;
//      a single-feature centre (weight ≈ 0.5–0.7) is weaker but still valid.
//   4. Asymmetric grid fit: a weighted RANSAC over nearest-neighbour (diagonal)
//      step hypotheses recovers the grid axes + spacing, matches all cols×rows
//      positions (every slot must have a detected feature — no synthesis, no
//      second-stage recovery), and orders the centres row-major to match
//      make_object_grid().
//
// Everything is local → robust to board rotation and lens distortion.

#ifndef GUI_ALGO_CALIBRATION_SCREWHEAD_DETECT_H
#define GUI_ALGO_CALIBRATION_SCREWHEAD_DETECT_H

#include "algo/calibration/intrinsic.h"  // DetectionResult

namespace gui_algo {

/// @brief Detects Zhou's Screw-Head Grid in a three-valued colour event frame.
/// @param color_frame CV_8UC3 BGR: black background, gold=ON, white=OFF,
///        blend=(gold+white)/2 where both polarities fired.
/// @param cols Markers per row (default 6).
/// @param rows Number of rows (default 5).
/// @param dot_gap Dashed-cross dot period parameter (1/2/3); sets the dilate
///        radius that bridges intra-cross gaps.
/// @param annotate If true, result.image is an annotated BGR copy.
/// @param ring Ring/circle detection parameters (coverage threshold, min ring
///        pixels, ring search margin). Relaxable at runtime — the wizard's
///        capture-review dialog exposes them and re-detects with new values.
/// @return DetectionResult: found=true only if ALL cols×rows markers matched;
///         points are row-major (index = r*cols + c), matching make_object_grid.
///         cross_centers / ring_centers / ring_radii are populated for EVERY
///         detected feature (even when the grid fit fails) so the review dialog
///         can draw the found crosses and rings.
DetectionResult detect_screwheads(const cv::Mat& color_frame,
                                  int cols, int rows,
                                  int dot_gap,
                                  bool annotate,
                                  const RingParams& ring = RingParams());

} // namespace gui_algo

#endif // GUI_ALGO_CALIBRATION_SCREWHEAD_DETECT_H
