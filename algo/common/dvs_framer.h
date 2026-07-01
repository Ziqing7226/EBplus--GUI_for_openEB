// algo/common/dvs_framer.h — DVS event framer (pure ON/OFF count accumulation).
//
// Inspired by jAER DvsFramer. Accumulates per-pixel ON/OFF event counts over a
// configurable accumulation window and produces a grayscale cv::Mat. This is
// the non-DL reconstruction baseline: brightness ~ event count. Polarity is
// encoded as signed accumulation (ON brightens, OFF darkens) or as separate
// channels depending on mode.

#ifndef GUI_ALGO_COMMON_DVS_FRAMER_H
#define GUI_ALGO_COMMON_DVS_FRAMER_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "event.h"

namespace gui_algo {

/// @brief DVS-style event framer producing grayscale frames from ON/OFF counts.
class DvsFramer {
public:
    enum class PolarityMode {
        /// ON and OFF both brighten the pixel (event-count image).
        UnsignedCount,
        /// ON brightens, OFF darkens (signed accumulation → bipolar image).
        Signed,
        /// Two-channel output: channel 0 = OFF count, channel 1 = ON count.
        SplitPolarity,
    };

    /// @brief Time-slice trigger for declaring a frame "filled" (mirrors jAER
    ///        DvsFramer.TimeSliceMethod, minus AreaEvent which is not ported).
    enum class TimeSliceMethod {
        Manual,          ///< Caller decides when to generate/generate_and_reset.
        EventCount,      ///< Filled when accumulated count >= events_per_frame.
        TimeIntervalUs,  ///< Filled when elapsed time >= time_duration_us_per_frame.
    };

    DvsFramer(int width, int height, PolarityMode mode = PolarityMode::UnsignedCount)
        : width_(width), height_(height), mode_(mode) {
        reset();
    }

    /// @brief Accumulates events in [begin, end) into the current frame.
    template <typename Iter>
    void add_events(Iter begin, Iter end) {
        for (auto it = begin; it != end; ++it) {
            add_one(static_cast<std::uint16_t>(it->x),
                    static_cast<std::uint16_t>(it->y),
                    static_cast<short>(it->p),
                    it->t);
        }
    }

    /// @brief Accumulates a single SDK event.
    void add_event(const Metavision::EventCD& e) {
        add_one(e.x, e.y, e.p, e.t);
    }

    /// @brief Generates the current accumulated frame and resets the accumulator.
    /// @param accumulation_time_us Accumulation window (recorded as metadata;
    ///        actual windowing is driven by caller invoking reset() periodically).
    /// @return cv::Mat of type CV_8UC1 (UnsignedCount/Signed) or CV_8UC2
    ///         (SplitPolarity). Counts are saturated to [0, 255].
    cv::Mat generate_and_reset(Metavision::timestamp accumulation_time_us = 0) {
        (void)accumulation_time_us;
        cv::Mat frame;
        switch (mode_) {
            case PolarityMode::UnsignedCount: {
                // ON + OFF both brighten the pixel (total event-count image).
                frame.create(height_, width_, CV_8UC1);
                for (int i = 0; i < width_ * height_; ++i) {
                    const int v = on_counts_[i] + off_counts_[i];
                    frame.at<std::uint8_t>(i) =
                        static_cast<std::uint8_t>(v > 255 ? 255 : v);
                }
                break;
            }
            case PolarityMode::Signed: {
                frame.create(height_, width_, CV_8UC1);
                for (int i = 0; i < width_ * height_; ++i) {
                    const int v = 128 + on_counts_[i] - off_counts_[i];
                    frame.at<std::uint8_t>(i) =
                        static_cast<std::uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
                }
                break;
            }
            case PolarityMode::SplitPolarity: {
                frame.create(height_, width_, CV_8UC2);
                for (int i = 0; i < width_ * height_; ++i) {
                    cv::Vec2b& px = frame.at<cv::Vec2b>(i);
                    px[0] = off_counts_[i];
                    px[1] = on_counts_[i];
                }
                break;
            }
        }
        reset();
        return frame;
    }

    /// @brief Generates the current accumulated frame without resetting.
    cv::Mat generate(Metavision::timestamp accumulation_time_us = 0) {
        (void)accumulation_time_us;
        cv::Mat frame;
        switch (mode_) {
            case PolarityMode::UnsignedCount: {
                // ON + OFF both brighten the pixel (total event-count image).
                frame.create(height_, width_, CV_8UC1);
                for (int i = 0; i < width_ * height_; ++i) {
                    const int v = on_counts_[i] + off_counts_[i];
                    frame.at<std::uint8_t>(i) =
                        static_cast<std::uint8_t>(v > 255 ? 255 : v);
                }
                break;
            }
            case PolarityMode::Signed: {
                frame.create(height_, width_, CV_8UC1);
                for (int i = 0; i < width_ * height_; ++i) {
                    const int v = 128 + on_counts_[i] - off_counts_[i];
                    frame.at<std::uint8_t>(i) =
                        static_cast<std::uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
                }
                break;
            }
            case PolarityMode::SplitPolarity: {
                frame.create(height_, width_, CV_8UC2);
                for (int i = 0; i < width_ * height_; ++i) {
                    cv::Vec2b& px = frame.at<cv::Vec2b>(i);
                    px[0] = off_counts_[i];
                    px[1] = on_counts_[i];
                }
                break;
            }
        }
        return frame;
    }

    void reset() {
        on_counts_.assign(static_cast<std::size_t>(width_) * height_, 0);
        off_counts_.assign(static_cast<std::size_t>(width_) * height_, 0);
        accumulated_event_count_ = 0;
        first_timestamp_us_ = -1;
        last_timestamp_us_ = -1;
    }

    int width() const { return width_; }
    int height() const { return height_; }
    PolarityMode mode() const { return mode_; }

    // Time-slice configuration & state (design §4.3.13, jAER DvsFramer) --------
    TimeSliceMethod slice_method() const { return slice_method_; }
    void set_slice_method(TimeSliceMethod m) { slice_method_ = m; }

    std::size_t events_per_frame() const { return events_per_frame_; }
    void set_events_per_frame(std::size_t n) { events_per_frame_ = n; }

    Metavision::timestamp time_duration_us_per_frame() const {
        return time_duration_us_per_frame_;
    }
    void set_time_duration_us_per_frame(Metavision::timestamp us) {
        time_duration_us_per_frame_ = us;
    }

    std::size_t accumulated_event_count() const {
        return accumulated_event_count_;
    }
    Metavision::timestamp first_timestamp_us() const {
        return first_timestamp_us_;
    }
    Metavision::timestamp last_timestamp_us() const {
        return last_timestamp_us_;
    }

    /// @brief True when the selected time-slice condition is met. In Manual
    ///        mode this is always false (caller drives generate/generate_and_reset).
    bool is_filled() const {
        switch (slice_method_) {
            case TimeSliceMethod::EventCount:
                return accumulated_event_count_ >= events_per_frame_;
            case TimeSliceMethod::TimeIntervalUs:
                if (first_timestamp_us_ < 0) return false;
                // A negative duration (timestamp wrap) also fills the frame,
                // matching jAER DvsFrame.addEvent semantics.
                return (last_timestamp_us_ - first_timestamp_us_ < 0) ||
                       (last_timestamp_us_ - first_timestamp_us_ >=
                        time_duration_us_per_frame_);
            case TimeSliceMethod::Manual:
            default:
                return false;
        }
    }

private:
    void add_one(std::uint16_t x, std::uint16_t y, short p,
                 Metavision::timestamp t) {
        if (x >= width_ || y >= height_) return;
        const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
        if (p) {
            if (on_counts_[idx] < 255) ++on_counts_[idx];
        } else {
            if (off_counts_[idx] < 255) ++off_counts_[idx];
        }
        if (first_timestamp_us_ < 0) first_timestamp_us_ = t;
        last_timestamp_us_ = t;
        ++accumulated_event_count_;
    }

    int width_;
    int height_;
    PolarityMode mode_;
    std::vector<std::uint8_t> on_counts_;
    std::vector<std::uint8_t> off_counts_;
    TimeSliceMethod slice_method_{TimeSliceMethod::Manual};
    std::size_t accumulated_event_count_{0};
    Metavision::timestamp first_timestamp_us_{-1};
    Metavision::timestamp last_timestamp_us_{-1};
    std::size_t events_per_frame_{2000};
    Metavision::timestamp time_duration_us_per_frame_{10000};
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_DVS_FRAMER_H
