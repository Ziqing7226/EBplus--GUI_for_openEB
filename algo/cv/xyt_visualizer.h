// algo/cv/xyt_visualizer.h — XYT 3D event point cloud data layer.
//
// Design §4.3.25. Provides the data slicing + color mapping for a 3D x-y-t
// event point cloud (X = pixel column, Y = pixel row, T = depth/time axis).
// Inspired by jAER SpaceTimeRollingEventDisplayMethod. The actual GL/VBO
// rendering lives in gui/display/space_time_display.h; this class only
// produces the point buffer. Header-only.

#ifndef GUI_ALGO_CV_XYT_VISUALIZER_H
#define GUI_ALGO_CV_XYT_VISUALIZER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief A single colored point in the XYT point cloud.
struct XYTPoint {
    float x{0.0f};   ///< Pixel column (X axis)
    float y{0.0f};   ///< Pixel row (Y axis)
    float t{0.0f};   ///< Normalized time/depth in [0, 1] within the window
    float r{0.0f};   ///< Red channel [0, 1]
    float g{0.0f};   ///< Green channel [0, 1]
    float b{0.0f};   ///< Blue channel [0, 1]
};

/// @brief Builds a rolling XYT point cloud from events for VBO rendering.
class XYTVisualizer {
public:
    enum class ColorMode {
        Polarity,  ///< ON = red, OFF = green
        Age,       ///< jAER shader: newest=blue, oldest=red, green in middle
    };

    /// @brief Constructs the visualizer.
    /// @param time_window_ms Display window length in ms, [10, 10000].
    /// @param color_mode Polarity- or age-based coloring. Default Age matches
    ///   jAER SpaceTimeRollingEventDisplayMethod shader coloring.
    /// @param point_size GL point size in [0.5, 10].
    /// @param auto_rotate Enable automatic scene rotation (rendering hint).
    /// @param depth_shade Enable depth-based shading. Default true matches
    ///   jAER's always-on fragment shader brightness (0.75*f1+0.25).
    XYTVisualizer(float time_window_ms = 50.0f,
                  ColorMode color_mode = ColorMode::Age,
                  float point_size = 2.5f,
                  bool auto_rotate = false,
                  bool depth_shade = true)
        : time_window_ms_(clamp_window(time_window_ms)),
          color_mode_(color_mode),
          point_size_(clamp_point(point_size)),
          auto_rotate_(auto_rotate),
          depth_shade_(depth_shade) {}

    /// @brief Appends a batch of events to the rolling buffer.
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        for (std::size_t i = 0; i < n; ++i) {
            buffer_.push_back(events[i]);
        }
        latest_t_ = buffer_.back().t;
        prune();
        // Hard cap: even with time-window pruning, a very high event rate can
        // grow the deque faster than prune() trims it, exhausting memory. Drop
        // the oldest events to stay under the cap.
        if (buffer_.size() > kMaxBuffer) {
            const std::size_t drop = buffer_.size() - kMaxBuffer;
            for (std::size_t i = 0; i < drop; ++i) buffer_.pop_front();
        }
    }

    /// @brief Number of events currently held in the rolling buffer.
    std::size_t size() const { return buffer_.size(); }

    /// @brief Slices the rolling buffer to the current time window and applies
    ///        color mapping, filling @p out with the point cloud for VBO
    ///        rendering. Reuses @p out's capacity across calls to avoid
    ///        per-frame heap allocation (important for 60 FPS rendering).
    void render(std::vector<XYTPoint>& out) const {
        out.clear();
        if (buffer_.empty()) return;
        // Normalize tn by the CONFIGURED time window (time_window_ms_), not
        // the buffer's actual event-time range. This gives a STABLE Z axis
        // that always represents the full configured window:
        //
        //   t_lo = latest_t_ - window_us   (oldest visible time)
        //   t_hi = latest_t_               (newest time)
        //   tn   = (e.t - t_lo) / window_us ∈ [0, 1]
        //
        // Rationale (design §4.3.25): When historical events don't exist
        // (e.g., at playback start, or during a low-density period), the
        // older portion of the Z axis is simply an EMPTY SET — no points —
        // rather than a compressed view of the sparse recent events. The
        // Z axis span never changes; only the point density within it does.
        //
        // This matches jAER's SpaceTimeRollingEventDisplayMethod, which uses
        // a fixed time window and lets the point cloud fill it naturally.
        // Events always appear at their true temporal position; when the
        // buffer hasn't accumulated enough history, points cluster near the
        // front (newest) and the back is empty — which is correct, not a bug.
        //
        // The previous approach (normalizing by the buffer's actual range)
        // artificially spread a tiny time range across the full Z axis,
        // making 5us of events look like 50ms — misleading and unstable.
        const Metavision::timestamp window_us =
            static_cast<Metavision::timestamp>(time_window_ms_ * 1000.0f);
        const Metavision::timestamp t_hi = latest_t_;
        const Metavision::timestamp t_lo = t_hi - window_us;
        out.reserve(buffer_.size());
        for (const Event& e : buffer_) {
            XYTPoint p;
            p.x = static_cast<float>(e.x);
            p.y = static_cast<float>(e.y);
            // tn: 0 = oldest (t_lo), 1 = newest (t_hi). After prune() all
            // buffered events satisfy e.t >= t_lo, so tn >= 0. Clamp for
            // floating-point safety.
            const float tn = window_us > 0
                ? static_cast<float>(e.t - t_lo) / static_cast<float>(window_us)
                : 1.0f;
            p.t = tn < 0.0f ? 0.0f : (tn > 1.0f ? 1.0f : tn);
            colorize(e, tn, p);
            out.push_back(p);
        }
    }

    void set_time_window_ms(float ms) { time_window_ms_ = clamp_window(ms); }
    float time_window_ms() const { return time_window_ms_; }

    void set_color_mode(ColorMode m) { color_mode_ = m; }
    ColorMode color_mode() const { return color_mode_; }

    void set_point_size(float s) { point_size_ = clamp_point(s); }
    float point_size() const { return point_size_; }

    void set_auto_rotate(bool r) { auto_rotate_ = r; }
    bool auto_rotate() const { return auto_rotate_; }

    void set_depth_shade(bool d) { depth_shade_ = d; }
    bool depth_shade() const { return depth_shade_; }

    /// @brief Clears the rolling buffer.
    void clear() { buffer_.clear(); }

private:
    static float clamp_window(float ms) {
        if (ms < 10.0f) return 10.0f;
        if (ms > 10000.0f) return 10000.0f;
        return ms;
    }
    static float clamp_point(float s) {
        if (s < 0.5f) return 0.5f;
        if (s > 10.0f) return 10.0f;
        return s;
    }

    void colorize(const Event& e, float tn, XYTPoint& p) const {
        switch (color_mode_) {
            case ColorMode::Polarity: {
                if (e.is_on()) {
                    p.r = 1.0f; p.g = 0.0f; p.b = 0.0f;  // ON = red
                } else {
                    p.r = 0.0f; p.g = 0.7f; p.b = 0.0f;  // OFF = green
                }
                break;
            }
            case ColorMode::Age: {
                // Match jAER SpaceTimeRollingEventDisplayMethod_Fragment.glsl:
                // f = 1 - tn (0 at newest, 1 at oldest)
                // b = max(1-2f, 0), r = max(2(f-0.5), 0), g = f if f<=0.5 else 1-f
                // brightness = 0.75*f1 + 0.25 is applied in the fragment
                // shader (space_time_display.cpp), NOT here — to avoid
                // double-application when depth_shade is enabled.
                const float f = 1.0f - tn;
                const float b = std::max(1.0f - 2.0f * f, 0.0f);
                const float r = std::max(2.0f * (f - 0.5f), 0.0f);
                float g = (f <= 0.5f) ? f : (1.0f - f);
                p.r = r;
                p.g = g;
                p.b = b;
                break;
            }
        }
    }

    void prune() {
        const Metavision::timestamp window_us =
            static_cast<Metavision::timestamp>(time_window_ms_ * 1000.0f);
        const Metavision::timestamp t_lo = latest_t_ - window_us;
        while (!buffer_.empty() && buffer_.front().t < t_lo) {
            buffer_.pop_front();
        }
    }

    float time_window_ms_;
    ColorMode color_mode_;
    float point_size_;
    bool auto_rotate_;
    bool depth_shade_;
    std::deque<Event> buffer_;
    Metavision::timestamp latest_t_{0};

    /// Hard cap on the rolling buffer to bound memory under event flooding.
    static constexpr std::size_t kMaxBuffer = 200000;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_XYT_VISUALIZER_H
