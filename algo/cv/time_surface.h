// algo/cv/time_surface.h — Time Surface window.
//
// Design §4.3.27. 直接引用 OpenEB 源码实现：
//   - Metavision::MostRecentTimestampBuffer 作为时间戳缓冲区
//   - Metavision::TimeSurfaceProcessor 用事件更新缓冲区
//   - MostRecentTimestampBuffer::generate_img_time_surface* 生成线性衰减灰度图
//   - 在灰度图基础上叠加伪彩色调色板 (Gray/Hot/Plasma/Turbo)
//
// OpenEB 的线性衰减公式 (mostrecent_timestamp_buffer_impl.h):
//   ratio = 255 / delta_t
//   pixel = (delta_t - (last_ts - ts)) * ratio   // clamped to [0, 255]
// 即 last_ts → 255 (亮), last_ts - delta_t → 0 (暗)。

#ifndef GUI_ALGO_CV_TIME_SURFACE_H
#define GUI_ALGO_CV_TIME_SURFACE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/core/preprocessors/time_surface_processor.h>
#include <metavision/sdk/core/utils/mostrecent_timestamp_buffer.h>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief Time Surface renderer — 直接引用 OpenEB MostRecentTimestampBuffer。
class TimeSurface {
public:
    enum class Channels {
        Merged = 1,   ///< 1 channel: both polarities share one buffer.
        Split = 2,    ///< 2 channels: separate ON/OFF buffers.
    };

    enum class Palette {
        Gray,
        Hot,
        Plasma,
        Turbo,
    };

    /// @brief Constructs the time surface.
    /// @param width,height Sensor dimensions.
    /// @param channels Merged or split polarity buffers.
    /// @param decay_time_us Linear decay time window in us (OpenEB delta_t).
    /// @param palette Pseudo-color palette.
    /// @param refresh_rate_hz Target render refresh rate in Hz.
    TimeSurface(int width, int height,
                Channels channels = Channels::Merged,
                Metavision::timestamp decay_time_us = 100000,
                Palette palette = Palette::Hot,
                int refresh_rate_hz = 30)
        : width_(width), height_(height), channels_(channels),
          decay_time_us_(clamp_decay(decay_time_us)),
          palette_(palette),
          refresh_rate_hz_(clamp_refresh(refresh_rate_hz)),
          // OpenEB MostRecentTimestampBuffer: rows=height, cols=width,
          // channels = 1 (merged) or 2 (split polarity).
          ts_buf_(height, width, static_cast<int>(channels_)) {
        ts_buf_.set_to(-1);  // -1 = sentinel for "never hit" (0 is a legal ts)
    }

    /// @brief Updates the MostRecentTimestampBuffer with a batch of events.
    /// 直接调用 OpenEB TimeSurfaceProcessor::process_events 的内联逻辑
    /// (time_surface_processor_impl.h::compute):
    ///   buffer[channels * (width * y + x) + c] = t
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        const int ch = static_cast<int>(channels_);
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const int c = (ch == 1) ? 0 : (e.p & 1);
            ts_buf_.at(e.y, e.x, c) = e.t;
            if (e.t > current_t_) current_t_ = e.t;
        }
    }

    /// @brief Renders the time-decay encoded pseudo-color image (CV_8UC3).
    /// 先用 OpenEB MostRecentTimestampBuffer::generate_img_time_surface*
    /// 生成 CV_8UC1 线性衰减灰度图，再用调色板映射为伪彩色。
    cv::Mat render() const {
        cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        if (width_ <= 0 || height_ <= 0) return img;
        if (current_t_ < 0) return img;  // no events yet: all pixels "never hit"

        // OpenEB 线性衰减灰度图生成。
        cv::Mat gray;
        if (channels_ == Channels::Merged) {
            // 合并极性: generate_img_time_surface_collapsing_channels
            ts_buf_.generate_img_time_surface_collapsing_channels(
                current_t_, decay_time_us_, gray);
        } else {
            // 分极性: generate_img_time_surface 输出并排 (W*2) 灰度图。
            // 我们只取左半 (OFF) 和右半 (ON) 合并为单帧彩色。
            ts_buf_.generate_img_time_surface(
                current_t_, decay_time_us_, gray);
        }

        if (gray.empty()) return img;

        if (channels_ == Channels::Split && gray.cols == width_ * 2) {
            // OpenEB split 模式输出 [OFF | ON] 并排。取两半叠加。
            cv::Mat off_roi(gray, cv::Rect(0, 0, width_, height_));
            cv::Mat on_roi(gray, cv::Rect(width_, 0, width_, height_));
            for (int y = 0; y < height_; ++y) {
                auto* dst = img.ptr<cv::Vec3b>(y);
                const auto* off = off_roi.ptr<std::uint8_t>(y);
                const auto* on = on_roi.ptr<std::uint8_t>(y);
                for (int x = 0; x < width_; ++x) {
                    const cv::Vec3b c_off = map_color(off[x] / 255.0);
                    const cv::Vec3b c_on = map_color(on[x] / 255.0);
                    // Per-channel max (§四-M2): cv::Vec3b + is a SATURATING
                    // add, so (c_off+c_on)*0.5 halved the brightness of
                    // single-polarity full-scale pixels (255 -> 128).
                    dst[x] = cv::Vec3b(std::max(c_off[0], c_on[0]),
                                       std::max(c_off[1], c_on[1]),
                                       std::max(c_off[2], c_on[2]));
                }
            }
        } else {
            // Merged: 灰度 → 伪彩色映射。
            for (int y = 0; y < height_; ++y) {
                auto* dst = img.ptr<cv::Vec3b>(y);
                const auto* src = gray.ptr<std::uint8_t>(y);
                for (int x = 0; x < width_; ++x) {
                    dst[x] = map_color(src[x] / 255.0);
                }
            }
        }
        return img;
    }

    /// @brief Returns the minimum render interval in us from refresh_rate_hz.
    Metavision::timestamp refresh_interval_us() const {
        return static_cast<Metavision::timestamp>(1.0e6 / refresh_rate_hz_);
    }

    void set_channels(Channels c) {
        if (c != channels_) {
            channels_ = c;
            ts_buf_.create(height_, width_, static_cast<int>(channels_));
            ts_buf_.set_to(-1);  // -1 = "never hit" sentinel
        }
    }
    Channels channels() const { return channels_; }

    void set_decay_time_us(Metavision::timestamp us) {
        decay_time_us_ = clamp_decay(us);
    }
    Metavision::timestamp decay_time_us() const { return decay_time_us_; }

    void set_palette(Palette p) { palette_ = p; }
    Palette palette() const { return palette_; }

    void set_refresh_rate_hz(int hz) { refresh_rate_hz_ = clamp_refresh(hz); }
    int refresh_rate_hz() const { return refresh_rate_hz_; }

    /// @brief Clears the timestamp buffer.
    void reset() {
        ts_buf_.set_to(-1);  // -1 = "never hit" sentinel (0 is a legal ts)
        current_t_ = -1;
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    static Metavision::timestamp clamp_decay(Metavision::timestamp us) {
        if (us < 10000) return 10000;
        if (us > 5000000) return 5000000;
        return us;
    }
    static int clamp_refresh(int hz) {
        if (hz < 10) return 10;
        if (hz > 120) return 120;
        return hz;
    }

    /// @brief Maps a normalized value in [0, 1] to a palette color.
    cv::Vec3b map_color(double v) const {
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        const auto b8 = [](double x) {
            return static_cast<std::uint8_t>(x * 255.0 + 0.5);
        };
        switch (palette_) {
            case Palette::Gray: {
                const std::uint8_t g = b8(v);
                return cv::Vec3b(g, g, g);
            }
            case Palette::Hot: {
                double r = 0.0, g = 0.0, b = 0.0;
                if (v < 0.33) {
                    r = v / 0.33;
                } else if (v < 0.66) {
                    r = 1.0;
                    g = (v - 0.33) / 0.33;
                } else {
                    r = 1.0;
                    g = 1.0;
                    b = (v - 0.66) / 0.34;
                }
                return cv::Vec3b(b8(b), b8(g), b8(r));
            }
            case Palette::Plasma: {
                double r = v;
                double g = std::max(0.0, (v - 0.5) * 2.0);
                double b = std::max(0.0, 0.5 - 0.5 * v);
                return cv::Vec3b(b8(b), b8(g), b8(r));
            }
            case Palette::Turbo: {
                double r = 0.0, g = 0.0, b = 0.0;
                if (v < 0.25) {
                    b = 1.0;
                    g = v / 0.25;
                } else if (v < 0.5) {
                    b = 1.0 - (v - 0.25) / 0.25;
                    g = 1.0;
                } else if (v < 0.75) {
                    g = 1.0;
                    r = (v - 0.5) / 0.25;
                } else {
                    r = 1.0;
                    g = 1.0 - (v - 0.75) / 0.25;
                }
                return cv::Vec3b(b8(b), b8(g), b8(r));
            }
        }
        return cv::Vec3b(0, 0, 0);
    }

    int width_;
    int height_;
    Channels channels_;
    Metavision::timestamp decay_time_us_;
    Palette palette_;
    int refresh_rate_hz_;
    /// OpenEB MostRecentTimestampBuffer — 直接引用，不自行实现时间戳缓冲区。
    Metavision::MostRecentTimestampBuffer ts_buf_;
    Metavision::timestamp current_t_{-1};  // -1 = no event processed yet
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_TIME_SURFACE_H
