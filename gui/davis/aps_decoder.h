// gui/davis/aps_decoder.h — APS frame decoder for the DAVIS wire format.
// Ported from dv-processing 2.0.4 parsers/davis_parser.hpp (Apache-2.0):
// frames arrive as marker-driven column readouts —
//   special 8/9     → frame start (global / rolling shutter)
//   special 14      → exposure start (frame timestamp source)
//   special 11/12   → reset / signal column start (two passes per column)
//   special 13      → column end
//   special 10      → frame end (emits when both column counts are complete)
//   code 4 words    → 12-bit pixel value, rows streamed within a column
//   code 5 misc8 1/2 → APS ROI position/size (four 2-part values per frame)
// Signal columns carry the correlated-double-sample: pixel = reset − signal,
// with the reference's cutoff filter for saturated pixels. Reference flip
// controls are not exposed by this port, so no flip conversion is applied
// (the reference skips it too when flip == flip-control).

#ifndef GUI_DAVIS_APS_DECODER_H
#define GUI_DAVIS_APS_DECODER_H

#include <algorithm>
#include <cstdint>
#include <functional>

#include <opencv2/core.hpp>

namespace gui::davis {

/// One completed APS frame (CV_8UC1 grayscale — the raw Bayer pattern on
/// color sensors, matching the reference's "original" color mode).
struct ApsFrame {
    std::int64_t t{0};  ///< Exposure-start stream time (rebased µs).
    std::uint16_t x{0}; ///< ROI position X within the user frame.
    std::uint16_t y{0}; ///< ROI position Y within the user frame.
    int width{0};
    int height{0};
    cv::Mat image;
    bool valid{false};
};

using ApsFrameSink = std::function<void(const ApsFrame&)>;

class ApsDecoder {
public:
    /// @param model one of the SENSOR_CHIP_* identifiers from the
    ///        MODULE_SYSINFO / chip identifier register (DAVIS240A/B/C = 0/1/2,
    ///        DAVIS346 = 5, DAVIS640 = 6, CDAVIS = 7).
    /// @param device_width/@param device_height APS register dimensions
    ///        (MODULE_APS size columns/rows, before orientation swap).
    ///        @param orientation MODULE_APS orientation info (bit 0x04 =
    ///        inverted axes, as on the DAVIS346).
    void configure(int model, int device_width, int device_height, int orientation) {
        model_ = model;
        is_240_ = (model == 0 || model == 1 || model == 2);
        is_cdavis_ = (model == 7);
        invert_xy_ = (orientation & 0x04) != 0;
        flip_x_ = (orientation & 0x02) != 0;
        flip_y_ = (orientation & 0x01) != 0;
        size_w_ = device_width;
        size_h_ = device_height;
        if (invert_xy_) std::swap(size_w_, size_h_);
    }

    void set_sink(const ApsFrameSink& sink) { sink_ = sink; }

    /// Frame start (special 8 = global shutter, special 9 = rolling shutter).
    void frame_start(bool global_shutter) {
        global_shutter_ = global_shutter;
        exposure_us_ = 0;
        roi_tmp_ = 0;
        roi_update_ = 0;
        current_reset_pass_ = true;
        count_x_[0] = count_x_[1] = 0;
        count_y_[0] = count_y_[1] = 0;
        // Defensive full-frame allocation when the device's ROI info has not
        // arrived (real hardware streams it; corrupted streams stay safe).
        if (pixels_.empty()) {
            roi_x_ = 0;
            roi_y_ = 0;
            frame_w_ = size_w_;
            frame_h_ = size_h_;
            // Same expectation swap as update_roi (inverted sensors read the
            // transposed grid).
            expected_x_ = invert_xy_ ? frame_h_ : frame_w_;
            expected_y_ = invert_xy_ ? frame_w_ : frame_h_;
            pixels_ = cv::Mat(frame_h_, frame_w_, CV_8UC1, cv::Scalar(0));
        }
    }

    /// Exposure start (special 14).
    void exposure_start(std::int64_t t) { exposure_us_ = t; }

    /// Reset column start (special 11).
    void reset_col_start() {
        current_reset_pass_ = true;
        count_y_[0] = 0;
        cdavis_column_start();
    }

    /// Signal column start (special 12).
    void signal_col_start() {
        current_reset_pass_ = false;
        count_y_[1] = 0;
        cdavis_column_start();
    }

    /// Column end (special 13) — one per readout pass, counted separately
    /// (the reference keeps per-type countX/countY arrays).
    void col_end() { ++count_x_[current_reset_pass_ ? 0 : 1]; }

    /// One APS pixel word (code 4, 12-bit value).
    void pixel(std::uint16_t value) {
        const int pass = current_reset_pass_ ? 0 : 1;
        // Ignore too-big counts — lost column start/end events (reference).
        if (count_x_[pass] >= expected_x_ || count_y_[pass] >= expected_y_) return;

        // DAVIS240 family: reduced dynamic range from the external ADC
        // reference resistors — compensate with a ×2 shift (reference).
        if (is_240_) {
            value = static_cast<std::uint16_t>(std::clamp<int>(value << 1, 0, 1023));
        }
        const auto data_value = static_cast<std::uint8_t>(value >> 2);

        if (is_cdavis_) {
            pixel_cdavis(data_value);
        } else {
            pixel_standard(data_value);
        }
        ++count_y_[pass];
    }

    /// APS ROI size part 1 (code 5, misc8 code 1): high byte.
    void roi_part1(std::uint8_t high) { roi_tmp_ = static_cast<std::uint16_t>(high << 8); }

    /// APS ROI size part 2 (code 5, misc8 code 2): low byte; every second
    /// pair completes a field (start column/row, end column/row).
    void roi_part2(std::uint8_t low) {
        const auto value = static_cast<std::uint16_t>(roi_tmp_ | low);
        switch (roi_update_ & 0x03) {
            case 0: roi_x_ = value; break;                // start column
            case 1: roi_y_ = value; break;                // start row
            case 2: roi_end_x_ = value; break;            // end column
            case 3: roi_end_y_ = value; update_roi(); break; // end row → complete
            default: break;
        }
        ++roi_update_;
    }

    /// Frame end (special 10): emits a clone of the frame when both readout
    /// passes completed with the expected column counts.
    void frame_end(std::int64_t /*t*/) {
        if (count_x_[0] == expected_x_ && count_x_[1] == expected_x_ && !pixels_.empty()) {
            ApsFrame frame;
            frame.t = exposure_us_;
            frame.x = roi_x_;
            frame.y = roi_y_;
            frame.width = pixels_.cols;
            frame.height = pixels_.rows;
            frame.image = pixels_.clone();
            frame.valid = true;
            if (sink_) sink_(frame);
        }
        pixels_ = cv::Mat();  // Next frame re-allocates (ROI info or fallback).
    }

    void reset() {
        pixels_ = cv::Mat();
        count_x_[0] = count_x_[1] = 0;
        count_y_[0] = count_y_[1] = 0;
        exposure_us_ = 0;
        roi_update_ = 0;
    }

private:
    void update_roi() {
        // The four ROI fields arrive as start/end; convert to position/size.
        const int sx = roi_end_x_ + 1 - roi_x_;
        const int sy = roi_end_y_ + 1 - roi_y_;
        frame_w_ = (sx <= 0 || sx > size_w_) ? size_w_ : sx;
        frame_h_ = (sy <= 0 || sy > size_h_) ? size_h_ : sy;
        if (roi_x_ + frame_w_ > size_w_) roi_x_ = 0;
        if (roi_y_ + frame_h_ > size_h_) roi_y_ = 0;
        // Inverted sensors read the transposed grid (DAVIS346).
        if (invert_xy_) {
            expected_x_ = frame_h_;
            expected_y_ = frame_w_;
        } else {
            expected_x_ = frame_w_;
            expected_y_ = frame_h_;
        }
        // CDAVIS walk start: odd start position ⇒ begin at row 1 (reference
        // startPositionOdd from the ROI start column/row); a single-row read
        // always starts at row 0.
        const auto roi_start = invert_xy_ ? roi_x_ : roi_y_;
        cdavis_start_odd_ = (roi_start & 0x01) != 0;
        if (expected_y_ == 1) cdavis_start_odd_ = false;
        cdavis_column_start();
        pixels_ = cv::Mat(frame_h_, frame_w_, CV_8UC1, cv::Scalar(0));
    }

    void cdavis_column_start() {
        if (!is_cdavis_) return;
        cdavis_y_ = cdavis_start_odd_ ? 1 : 0;
        cdavis_direction_down_ = true;
    }

    void pixel_standard(std::uint8_t data_value) {
        const int pass = current_reset_pass_ ? 0 : 1;
        int xPos = count_x_[pass];
        int yPos = count_y_[pass];
        // Reference order: flips in count space first, then the inverted
        // sensors' transpose swap.
        if (flip_x_) xPos = expected_x_ - 1 - xPos;
        if (flip_y_) yPos = expected_y_ - 1 - yPos;
        if (invert_xy_) std::swap(xPos, yPos);

        auto& cell = pixels_.at<std::uint8_t>(yPos, xPos);
        if (current_reset_pass_) {
            cell = data_value;
        } else {
            const auto reset_value = cell;
            // Saturated pixels: reference cutoff filter (black-spot removal).
            if (reset_value < 96 || data_value == 0) {
                cell = 255;
            } else {
                cell = static_cast<std::uint8_t>(
                    std::clamp<int>(reset_value - data_value, 0, 255));
            }
        }
    }

    void pixel_cdavis(std::uint8_t data_value) {
        const int pass = current_reset_pass_ ? 0 : 1;
        int xPos = count_x_[pass];
        int yPos = cdavis_y_;
        if (flip_x_) xPos = expected_x_ - 1 - xPos;
        if (flip_y_) yPos = expected_y_ - 1 - yPos;
        if (invert_xy_) std::swap(xPos, yPos);

        auto& cell = pixels_.at<std::uint8_t>(yPos, xPos);
        if (!current_reset_pass_) {
            // CDAVIS global shutter reads the signal sample first.
            cell = data_value;
        } else {
            const auto reset_value = data_value;
            const auto signal_value = cell;
            if (reset_value < 96 || signal_value == 0) {
                cell = 255;
            } else {
                cell = static_cast<std::uint8_t>(
                    std::clamp<int>(reset_value - signal_value, 0, 255));
            }
        }

        // CDAVIS interleave: the first half of each column walks the even
        // rows, then the odd rows (reference walk, untested model).
        const bool length_odd = (expected_y_ & 0x01) != 0;
        if ((!cdavis_start_odd_ && !length_odd) || (cdavis_start_odd_ && length_odd)) {
            if (cdavis_y_ == expected_y_ - 2) {
                cdavis_direction_down_ = false;
                ++cdavis_y_;
                return;
            }
        } else {
            if (cdavis_y_ == expected_y_ - 1) {
                cdavis_direction_down_ = false;
                --cdavis_y_;
                return;
            }
        }
        cdavis_y_ += cdavis_direction_down_ ? 2 : -2;
    }

    int model_{5};  // DAVIS346 default.
    bool is_240_{false};
    bool is_cdavis_{false};
    bool invert_xy_{false};
    bool flip_x_{false};
    bool flip_y_{false};
    int size_w_{0};
    int size_h_{0};

    int frame_w_{0};
    int frame_h_{0};
    std::uint16_t roi_x_{0};
    std::uint16_t roi_y_{0};
    std::uint16_t roi_end_x_{0};
    std::uint16_t roi_end_y_{0};
    std::uint16_t roi_tmp_{0};
    std::uint8_t roi_update_{0};

    int expected_x_{0};
    int expected_y_{0};
    int count_x_[2]{0, 0};  // Per readout pass (reset / signal).
    int count_y_[2]{0, 0};
    bool current_reset_pass_{true};
    bool global_shutter_{false};
    std::int64_t exposure_us_{0};

    // CDAVIS odd/even row walk.
    int cdavis_y_{0};
    bool cdavis_start_odd_{false};
    bool cdavis_direction_down_{true};

    cv::Mat pixels_;
    ApsFrameSink sink_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_APS_DECODER_H
