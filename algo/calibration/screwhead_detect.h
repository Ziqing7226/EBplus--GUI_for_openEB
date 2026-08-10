// algo/calibration/screwhead_detect.h — Zhou's Screw-Head Grid detector.
//
// Detects an asymmetric 6×5 grid of "screw-head" markers (a dashed cross that
// pins the centre + a solid thin ring that supplies polarity) in a three-valued
// event frame: background black, ON polarity = gold, OFF polarity = white, and
// a pixel hit by both polarities takes the simple average of gold and white.
//
// Four-stage pipeline (see devlog/calibration_chessboard_redesign.md):
//   1. The frame is already accumulated by the caller (CalibrationWizard).
//   2. Cross-centre candidates: 8-neighbour denoise → isotropic dilate (bridges
//      intra-cross dot gaps) → connected components + area filter → density peak
//      within each component (the cross, where two arms meet, is the densest
//      spot, so its peak is the centre — unbiased by partial ring arcs).
//   3. Ring + polarity verification: per candidate, a radial histogram finds the
//      ring radius R*; the ring pixels are collected, sorted by polar angle, and
//      checked for the "half/half" structure (one semicircle one polarity, the
//      other the opposite) that a real ring edge produces under translation.
//      Random noise has no such structure → rejected.
//   4. Asymmetric grid fit: a RANSAC over nearest-neighbour (diagonal) step
//      hypotheses recovers the grid axes + spacing, matches all cols×rows
//      positions, and orders the centres row-major to match make_object_grid().
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
/// @return DetectionResult: found=true only if ALL cols×rows markers matched;
///         points are row-major (index = r*cols + c), matching make_object_grid.
DetectionResult detect_screwheads(const cv::Mat& color_frame,
                                  int cols, int rows,
                                  int dot_gap,
                                  bool annotate);

} // namespace gui_algo

#endif // GUI_ALGO_CALIBRATION_SCREWHEAD_DETECT_H
