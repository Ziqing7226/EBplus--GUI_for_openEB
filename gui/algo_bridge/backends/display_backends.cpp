// gui/algo_bridge/backends/display_backends.cpp — TimeSurface, UltraSlowMotion,
// XYTVisualizer, Overlay (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algo/cv/time_surface.h"
#include "algo/cv/ultra_slow_motion.h"
#include "algo/cv/xyt_visualizer.h"

using namespace gui::backend_detail;

namespace gui {

class TimeSurfaceBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    int decay_time_us_{100000};
    gui_algo::TimeSurface::Palette palette_{gui_algo::TimeSurface::Palette::Hot};
    gui_algo::TimeSurface::Channels channels_{gui_algo::TimeSurface::Channels::Merged};
    std::unique_ptr<gui_algo::TimeSurface> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    Preprocessor preproc_;
public:
    TimeSurfaceBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        preproc_.init(aw, ah);
        const int f = preproc_.factor();
        algo_ = std::make_unique<gui_algo::TimeSurface>(
            aw / f, ah / f, channels_, decay_time_us_, palette_, 30);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (preproc_.set_param(k, v)) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Only rebuild when the effective dimensions actually change
        // (audit §5-D4): apply_global_roi fires 5 set_param calls, and a
        // rebuild discards the per-pixel timestamp surface each time.
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        bool roi_changed = false;
        if (k == "decay_time_us") {
            decay_time_us_ = to_i(v);
            if (algo_) algo_->set_decay_time_us(decay_time_us_);
        } else if (k == "palette") {
            int p = to_i(v);
            if (p >= 0 && p <= 3) {
                palette_ = static_cast<gui_algo::TimeSurface::Palette>(p);
                if (algo_) algo_->set_palette(palette_);
            }
        } else if (k == "channels") {
            int c = to_i(v);
            channels_ = (c == 2) ? gui_algo::TimeSurface::Channels::Split
                                  : gui_algo::TimeSurface::Channels::Merged;
            if (algo_) algo_->set_channels(channels_);
        } else if (k == "roi_enabled") { roi_.enabled = to_b(v); roi_changed = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); roi_changed = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); roi_changed = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); roi_changed = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); roi_changed = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
        else if (roi_changed) {
            const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
            const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
            roi_.compute(sensor_w_, sensor_h_);
            const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
            if (new_aw != old_aw || new_ah != old_ah) rebuild();
        }
    }
    std::string get_param(const std::string& k) const override {
        auto pp = preproc_.get_param(k); if (!pp.empty()) return pp;
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "roi_x") return from_i(roi_.x);
        if (k == "roi_y") return from_i(roi_.y);
        if (k == "roi_w") return from_i(roi_.w);
        if (k == "roi_h") return from_i(roi_.h);
        if (k == "decay_time_us") return from_i(decay_time_us_);
        if (k == "palette") return from_i(static_cast<int>(palette_));
        if (k == "channels") return from_i(channels_ == gui_algo::TimeSurface::Channels::Split ? 2 : 1);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        const auto* ev = as_events(passthrough_.data());
        std::size_t n = passthrough_.size();
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            crop_to_roi(ev, n, roi_, &preproc_, roi_events_);
            ev = roi_events_.data();
            n = roi_events_.size();
        } else if (preproc_.active() && n > 0) {
            auto [p, m] = preproc_.apply(ev, n);
            roi_events_.assign(p, p + m);
            ev = roi_events_.data();
            n = m;
        }
        algo_->process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        cv::Mat frame = algo_->render();
        const int f = preproc_.factor();
        if (f > 1 && !frame.empty()) {
            const int aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int ah = roi_.enabled ? roi_.rh : sensor_h_;
            cv::resize(frame, frame, cv::Size(aw, ah), 0, 0, cv::INTER_NEAREST);
        }
        r.frame = frame.clone();
        r.status = "time_surface: " + std::to_string(algo_->width()) + "x" +
                   std::to_string(algo_->height()) +
                   (roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

class UltraSlowMotionBackend final : public AlgoBackend {
    gui_algo::UltraSlowMotion algo_;
    std::vector<Metavision::EventCD> last_out_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    UltraSlowMotionBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "factor") algo_.set_dilation_factor(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "factor") return from_d(algo_.dilation_factor());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        auto out = algo_.process(ev, n);
        last_out_.assign(out.begin(), out.end());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = last_out_;
        r.status = "slow_motion: " + std::to_string(last_out_.size()) + " events" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); last_out_.clear(); passthrough_.clear(); roi_buf_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        // The algo holds no sensor-sized state (timestamp dilator) — only
        // the ROI geometry needs updating (audit §5-D1).
        roi_.set_sensor_dimensions(w, h);
    }
};


// ===========================================================================
// Group G: Visualization
// ===========================================================================

/// XYTVisualizer backend — 3D point cloud data for space_time_display.
/// Complex algorithm (design §4.3.25): defaults to the center 128×128 ROI to
/// bound the point-cloud size. Events outside the ROI are dropped.
///
/// NOTE: this backend does NOT produce overlay output. The 3D point cloud is
/// rendered by SpaceTimeDisplay, which owns its own XYTVisualizer instance and
/// receives events directly from the SDK callback (see install_algo_callback).
/// This backend exists only so the AlgoWindow can expose ROI/parameters and
/// reflect the buffer state in its status label.
class XYTVisualizerBackend final : public AlgoBackend {
    gui_algo::XYTVisualizer algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    XYTVisualizerBackend(int w, int h)
        : algo_(500.0f,
                gui_algo::XYTVisualizer::ColorMode::Age,
                2.5f,
                false,
                true) {
        roi_.init(w, h);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "time_window_us") algo_.set_time_window_ms(static_cast<float>(to_i(v)) / 1000.0F);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "time_window_us") return from_i(static_cast<int>(algo_.time_window_ms() * 1000.0F));
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
        // NOTE: do NOT call render() here — the 3D point cloud is rendered by
        // SpaceTimeDisplay which owns its own XYTVisualizer instance. This
        // backend only forwards events so the AlgoWindow status reflects the
        // ROI filter. Generating 50K OverlayPoints here would splatter 2D
        // points across the main display (wrong) and stall the GUI thread.
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // No overlay/frame: the 3D rendering is handled by SpaceTimeDisplay.
        r.status = "xyt: " + std::to_string(algo_.size()) + " buffered events" +
                   std::string(roi_.region.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override { algo_.clear(); passthrough_.clear(); roi_buf_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        // XYTVisualizer holds no sensor-sized state (a rolling event deque);
        // only the ROI geometry needs updating (audit §5-D2).
        roi_.set_sensor_dimensions(w, h);
    }
};


class OverlayBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    OverlayBackend(int w, int h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        roi_.set_param(k, v);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        // pull_result reads from passthrough_: if roi_.apply() filtered into
        // another buffer, copy the result back; otherwise it is already there.
        if (ev != as_events(passthrough_.data())) {
            passthrough_.assign(reinterpret_cast<const Metavision::EventCD*>(ev),
                                reinterpret_cast<const Metavision::EventCD*>(ev + n));
        } else {
            passthrough_.resize(n);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "overlay: pass-through" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { passthrough_.clear(); roi_buf_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
    }
};



// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_display_backend(const std::string& name,
                                          int width, int height) {
    if (name == "time_surface")                return std::make_unique<TimeSurfaceBackend>(width, height);
    if (name == "ultra_slow_motion")           return std::make_unique<UltraSlowMotionBackend>(width, height);
    if (name == "xyt_visualizer")              return std::make_unique<XYTVisualizerBackend>(width, height);
    if (name == "overlay")                     return std::make_unique<OverlayBackend>(width, height);
    return nullptr;
}

} // namespace gui
