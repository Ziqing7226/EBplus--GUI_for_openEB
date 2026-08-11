// algo/calibration/blinking_detect.h — blinking chessboard calibration support
// (header-only, no Qt dependency).
//
// The calibration wizard displays a chessboard pattern on screen that alternates
// with a blank frame at a fixed period (10 ms). Pixels on black squares toggle
// dark↔light every cycle and therefore fire both ON and OFF events, while white
// squares and the background stay constant. Over a capture window that covers at
// least one full blink cycle, the pixels that saw BOTH polarities form a filled
// checkerboard (the silhouettes of the black squares) — exactly the pattern
// cv::findChessboardCorners expects.
//
// This file provides:
//   - accumulate_blink_masks(): per-polarity 0/1 masks from a CD event range
//   - build_blink_frame():      binary blink frame + validity gate
// The masks are kept by the caller so a capture can be re-rendered with relaxed
// blink parameters (the wizard's Re-detect) without re-reading the events.
//
// The blink frame is the INVERSE of the both-polarity mask: pixels that saw
// both polarities (the black squares of the displayed board) become BLACK, all
// other pixels WHITE. That reproduces the original chessboard (black squares on
// a white background), which is the polarity cv::findChessboardCorners is
// reliable on — the raw both-mask (white squares on black) is not detected.

#ifndef GUI_ALGO_CALIBRATION_BLINKING_DETECT_H
#define GUI_ALGO_CALIBRATION_BLINKING_DETECT_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/events/event_cd.h>

namespace gui_algo {

/// @brief Parameters controlling blink-frame construction and its validity gate.
struct BlinkParams {
    int min_blink_pixels{100};   ///< Minimum pixels that saw both polarities for a valid board frame.
    double ratio_on{0.15};       ///< Max acceptable |only-ON| / |both| ratio (rejects polarity-imbalanced noise).
    double ratio_off{0.15};      ///< Max acceptable |only-OFF| / |both| ratio.
    int median_blur_diameter{3}; ///< Median blur diameter on the binary mask (0 or even disables).
};

/// @brief Binary blink frame plus the polarity statistics behind its gate.
/// The frame is INVERTED: both-polarity pixels (the black squares) are black
/// (0), everything else white (255) — the chessboard polarity that
/// cv::findChessboardCorners detects reliably.
struct BlinkFrame {
    bool valid{false};   ///< Passed the min-blink-pixels + ratio gate.
    cv::Mat frame;       ///< CV_8UC1: 0 where both polarities were seen, 255 elsewhere.
    int both{0};         ///< Pixels that saw both polarities.
    int only_on{0};      ///< Pixels that saw only ON events.
    int only_off{0};     ///< Pixels that saw only OFF events.
    int area{0};         ///< Sensor area (w*h).
};

/// @brief Accumulates per-polarity 0/1 masks (CV_8UC1) from a range of CD events.
inline void accumulate_blink_masks(const Metavision::EventCD* begin,
                                   const Metavision::EventCD* end,
                                   int width, int height,
                                   cv::Mat& has_on, cv::Mat& has_off) {
    has_on = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    has_off = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    for (const auto* ev = begin; ev != end; ++ev) {
        // Cast to int first: EventCD coordinates may be unsigned, so `x < 0`
        // would be "always false" under -Wtype-limits.
        const int ex = static_cast<int>(ev->x);
        const int ey = static_cast<int>(ev->y);
        if (ex < 0 || ex >= width || ey < 0 || ey >= height) continue;
        if (ev->p != 0) has_on.at<uchar>(ey, ex) = 1;
        else            has_off.at<uchar>(ey, ex) = 1;
    }
}

/// @brief Accumulates per-pixel ON/OFF event COUNTS (CV_32S) from a CD range.
/// The wizard uses counts (not the binary masks): on a real LCD the backlight
/// PWM alone gives every on-screen pixel both polarities, so the binary
/// both-mask is a solid blob; the black squares of the board fire 2-3x more
/// events than the noise floor, and a count threshold recovers them.
inline void accumulate_blink_counts(const Metavision::EventCD* begin,
                                    const Metavision::EventCD* end,
                                    int width, int height,
                                    cv::Mat& on_cnt, cv::Mat& off_cnt) {
    on_cnt = cv::Mat(height, width, CV_32S, cv::Scalar(0));
    off_cnt = cv::Mat(height, width, CV_32S, cv::Scalar(0));
    for (const auto* ev = begin; ev != end; ++ev) {
        const int ex = static_cast<int>(ev->x);
        const int ey = static_cast<int>(ev->y);
        if (ex < 0 || ex >= width || ey < 0 || ey >= height) continue;
        if (ev->p != 0) ++on_cnt.at<int>(ey, ex);
        else            ++off_cnt.at<int>(ey, ex);
    }
}

namespace detail {

/// @brief Adaptive blink-frame threshold: frac × the 99th percentile of the
/// per-pixel ON+OFF count. The threshold rides above the screen's backlight
/// noise floor (which scales with the screen brightness / viewing distance)
/// and below the black-square toggle counts, so it needs no per-scene tuning.
inline int blink_count_threshold(const cv::Mat& sum, int lo, int hi, double frac) {
    int cmax = 0;
    for (int i = 0; i < sum.rows * sum.cols; ++i) {
        const int c = sum.at<int>(i);
        if (c > cmax) cmax = c;
    }
    if (cmax <= 0) return lo;
    std::vector<int> hist(static_cast<std::size_t>(cmax) + 1, 0);
    int nz = 0;
    for (int i = 0; i < sum.rows * sum.cols; ++i) {
        const int c = sum.at<int>(i);
        if (c > 0) {
            ++hist[static_cast<std::size_t>(c)];
            ++nz;
        }
    }
    const long long need = std::max(1LL, static_cast<long long>(nz) / 100);
    long long acc = 0;
    for (int c = cmax; c >= 1; --c) {
        acc += hist[static_cast<std::size_t>(c)];
        if (acc >= need) {
            const int t = static_cast<int>(std::lround(frac * c));
            return std::max(lo, std::min(hi, t));
        }
    }
    return lo;
}

} // namespace detail

/// @brief Builds the binary blink frame from per-pixel event COUNTS.
///
/// The black squares of the displayed board toggle dark↔light every blink
/// cycle and fire 2-3x more events than the constant white squares; the count
/// threshold separates them where the binary both-mask cannot (backlight PWM
/// noise gives every screen pixel both polarities). The frame is INVERTED like
/// the mask version: strongly blinking pixels are black on white.
inline BlinkFrame build_blink_frame_from_counts(const cv::Mat& on_cnt,
                                                const cv::Mat& off_cnt,
                                                const BlinkParams& params = {}) {
    BlinkFrame out;
    if (on_cnt.empty() || off_cnt.empty() || on_cnt.type() != CV_32S ||
        off_cnt.type() != CV_32S || on_cnt.size() != off_cnt.size()) {
        return out;
    }
    out.area = on_cnt.rows * on_cnt.cols;

    cv::Mat sum;
    cv::add(on_cnt, off_cnt, sum);
    const int T = detail::blink_count_threshold(sum, 12, 64, 0.4);

    out.frame = cv::Mat(on_cnt.rows, on_cnt.cols, CV_8UC1, cv::Scalar(255));
    out.both = 0;
    out.only_on = 0;
    out.only_off = 0;
    for (int y = 0; y < on_cnt.rows; ++y) {
        const int* on = on_cnt.ptr<int>(y);
        const int* off = off_cnt.ptr<int>(y);
        uchar* row = out.frame.ptr<uchar>(y);
        for (int x = 0; x < on_cnt.cols; ++x) {
            if (on[x] + off[x] >= T) {
                row[x] = 0;
                ++out.both;
                if (off[x] < T && on[x] >= T) ++out.only_on;
                else if (on[x] < T && off[x] >= T) ++out.only_off;
            }
        }
    }
    if (params.median_blur_diameter >= 3 && (params.median_blur_diameter & 1)) {
        cv::medianBlur(out.frame, out.frame, params.median_blur_diameter);
    }

    const bool enough = out.both >= params.min_blink_pixels;
    const bool balanced = out.both > 0 &&
        static_cast<double>(out.only_on) / out.both <= params.ratio_on &&
        static_cast<double>(out.only_off) / out.both <= params.ratio_off;
    out.valid = enough && balanced;
    return out;
}

/// @brief Builds the binary blink frame from pre-accumulated masks and applies
/// the validity gate (min blinking pixels + single-polarity ratios).
inline BlinkFrame build_blink_frame(const cv::Mat& has_on, const cv::Mat& has_off,
                                    const BlinkParams& params = {}) {
    BlinkFrame out;
    if (has_on.empty() || has_off.empty() || has_on.type() != CV_8UC1 ||
        has_off.type() != CV_8UC1 || has_on.size() != has_off.size()) {
        return out;
    }
    out.area = has_on.rows * has_on.cols;

    cv::Mat both, not_on, not_off;
    cv::bitwise_and(has_on, has_off, both);
    cv::bitwise_not(has_on, not_on);
    cv::bitwise_not(has_off, not_off);

    cv::Mat only_on, only_off;
    cv::bitwise_and(has_on, not_off, only_on);
    cv::bitwise_and(has_off, not_on, only_off);

    out.both = cv::countNonZero(both);
    out.only_on = cv::countNonZero(only_on);
    out.only_off = cv::countNonZero(only_off);

    // Inverted binary frame: black where both polarities were seen (the
    // displayed board's black squares), white elsewhere. findChessboardCorners
    // is reliable on black-on-white, not on the raw both-mask.
    both.convertTo(out.frame, CV_8UC1, -255.0, 255.0);  // 255 - both*255
    if (params.median_blur_diameter >= 3 && (params.median_blur_diameter & 1)) {
        cv::medianBlur(out.frame, out.frame, params.median_blur_diameter);
    }

    const bool enough = out.both >= params.min_blink_pixels;
    const bool balanced = out.both > 0 &&
        static_cast<double>(out.only_on) / out.both <= params.ratio_on &&
        static_cast<double>(out.only_off) / out.both <= params.ratio_off;
    out.valid = enough && balanced;
    return out;
}

/// @brief Convenience: accumulate masks then build the blink frame in one call.
inline BlinkFrame make_blink_frame(const Metavision::EventCD* begin,
                                   const Metavision::EventCD* end,
                                   int width, int height,
                                   const BlinkParams& params = {}) {
    cv::Mat on, off;
    accumulate_blink_masks(begin, end, width, height, on, off);
    return build_blink_frame(on, off, params);
}

} // namespace gui_algo

#endif // GUI_ALGO_CALIBRATION_BLINKING_DETECT_H
