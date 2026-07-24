// gui/algo_bridge/backends/filter_backends.cpp — OrientationFilter, DirectionSelective,
// BackgroundMask, BandpassFilter (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <string>
#include <vector>

#include "algo/cv/orientation_filter.h"
#include "algo/cv/direction_selective_filter.h"
#include "algo/cv/background_mask_filter.h"
#include "algo/cv/bandpass_filter.h"

using namespace gui::backend_detail;

namespace gui {

// ===========================================================================
// Group H: Misc filters
// ===========================================================================

/// OrientationFilter backend — orientation histogram as overlay text.
/// Supports ROI (design §5.6.6): only ROI events feed the histogram.
class OrientationFilterBackend final : public AlgoBackend {
    gui_algo::OrientationFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::vector<int> last_orientations_;  // per-event orientation labels
    // Decaying histogram (jAER uses a per-packet decay factor on the
    // orientation counters). A cumulative counter would grow without bound.
    float hist_[gui_algo::OrientationFilter::kNumOrientations]{};
    static constexpr float kHistDecay = 0.9F;  // per-packet decay
public:
    OrientationFilterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "min_dt_threshold_us") algo_.set_min_dt_threshold_us(to_i(v));
        else if (k == "use_average_dt") algo_.set_use_average_dt(to_b(v));
        else if (k == "ori_history_enabled") algo_.set_ori_history_enabled(to_b(v));
        else if (k == "dt_reject_threshold_us") algo_.set_dt_reject_threshold_us(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "min_dt_threshold_us") return from_i(algo_.min_dt_threshold_us());
        if (k == "use_average_dt") return from_b(algo_.use_average_dt());
        if (k == "ori_history_enabled") return from_b(algo_.ori_history_enabled());
        if (k == "dt_reject_threshold_us") return from_i(algo_.dt_reject_threshold_us());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        last_orientations_.resize(n);
        // Per-packet exponential decay (jAER oriCountsMap decay).
        for (int i = 0; i < gui_algo::OrientationFilter::kNumOrientations; ++i) {
            hist_[i] *= kHistDecay;
        }
        for (std::size_t i = 0; i < n; ++i) {
            last_orientations_[i] = algo_.classify(ev[i]);
            if (last_orientations_[i] >= 0 &&
                last_orientations_[i] < gui_algo::OrientationFilter::kNumOrientations) {
                hist_[last_orientations_[i]] += 1.0F;
            }
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Per-event colored events (jAER DvsOrientationEvent coloring).
        r.colored_events.reserve(last_orientations_.size());
        for (std::size_t i = 0; i < last_orientations_.size() && i < passthrough_.size(); ++i) {
            const int ori = last_orientations_[i];
            if (ori < 0) continue;
            const cv::Vec3b c = algo_.color(ori);
            ColoredEvent ce;
            ce.event = passthrough_[i];
            ce.r = c[2]; ce.g = c[1]; ce.b = c[0];  // BGR -> RGB
            r.colored_events.push_back(ce);
        }
        // Global orientation vector (from chip center, jAER computeGlobalOriVector).
        {
            float gx = 0, gy = 0;
            static constexpr float kCosTable[4] = {1.0F, 0.70710678F, 0.0F, -0.70710678F};
            static constexpr float kSinTable[4] = {0.0F, 0.70710678F, 1.0F, 0.70710678F};
            for (int i = 0; i < gui_algo::OrientationFilter::kNumOrientations; ++i) {
                gx += kCosTable[i] * hist_[i];
                gy += kSinTable[i] * hist_[i];
            }
            const int cx = algo_.width() / 2;
            const int cy = algo_.height() / 2;
            const float scale = 0.02F;
            OverlayLine gl;
            gl.x1 = cx - static_cast<int>(gx * scale);
            gl.y1 = cy - static_cast<int>(gy * scale);
            gl.x2 = cx + static_cast<int>(gx * scale);
            gl.y2 = cy + static_cast<int>(gy * scale);
            r.lines.push_back(gl);
        }
        // Histogram text + legend.
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "orient: 0=" + std::to_string(static_cast<int>(hist_[0])) +
                 " 45=" + std::to_string(static_cast<int>(hist_[1])) +
                 " 90=" + std::to_string(static_cast<int>(hist_[2])) +
                 " 135=" + std::to_string(static_cast<int>(hist_[3]));
        r.texts.push_back(t);
        r.status = "orient_filter: " + std::to_string(r.colored_events.size()) +
                   " colored events" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_orientations_.clear(); std::fill(hist_, hist_ + 4, 0.0F); }
};

/// DirectionSelectiveFilter backend — direction histogram as overlay text.
/// Supports ROI (design §5.6.6): only ROI events feed the histogram.
class DirectionSelectiveBackend final : public AlgoBackend {
    gui_algo::DirectionSelectiveFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::vector<int> last_dirs_;  // per-event direction labels
public:
    DirectionSelectiveBackend(int w, int h) : algo_(w, h) {
        roi_.init(w, h);
        algo_.set_enable_global_mode(true);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "tau_us") algo_.set_time_window_us(to_i(v));
        else if (k == "min_dt_us") algo_.set_min_dt_us(to_i(v));
        else if (k == "search_distance") algo_.set_search_distance(to_i(v));
        else if (k == "tau_low_ms") algo_.set_tau_low_ms(to_i(v));
        else if (k == "enable_global_mode") algo_.set_enable_global_mode(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "tau_us") return from_i(algo_.time_window_us());
        if (k == "min_dt_us") return from_i(algo_.min_dt_us());
        if (k == "search_distance") return from_i(algo_.search_distance());
        if (k == "tau_low_ms") return from_i(algo_.tau_low_ms());
        if (k == "enable_global_mode") return from_b(algo_.enable_global_mode());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        last_dirs_.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            last_dirs_[i] = algo_.classify(ev[i]);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Per-event colored events (jAER DirEvent coloring, 8 colors).
        // 8-direction palette: 0=E red, 1=NE orange, 2=N yellow, 3=NW green,
        // 4=W cyan, 5=SW blue, 6=S magenta, 7=SE white.
        static const std::uint8_t pal[8][3] = {
            {255, 0, 0}, {255, 165, 0}, {255, 255, 0}, {0, 255, 0},
            {0, 255, 255}, {0, 0, 255}, {255, 0, 255}, {255, 255, 255}};
        r.colored_events.reserve(last_dirs_.size());
        for (std::size_t i = 0; i < last_dirs_.size() && i < passthrough_.size(); ++i) {
            const int d = last_dirs_[i];
            if (d < 0 || d >= 8) continue;
            ColoredEvent ce;
            ce.event = passthrough_[i];
            ce.r = pal[d][0]; ce.g = pal[d][1]; ce.b = pal[d][2];
            r.colored_events.push_back(ce);
        }
        // Global motion vectors (jAER translation/rotation/expansion).
        // Translation arrow from chip center scaled for visibility.
        const int cx = algo_.width() / 2;
        const int cy = algo_.height() / 2;
        const auto tr = algo_.translation();
        const float tlen = std::hypot(tr.x, tr.y);
        if (tlen > 1.0F) {
            const float scale = 50.0F / std::max(1.0F, tlen);
            OverlayLine tl;
            tl.x1 = cx; tl.y1 = cy;
            tl.x2 = cx + static_cast<int>(tr.x * scale);
            tl.y2 = cy + static_cast<int>(tr.y * scale);
            r.lines.push_back(tl);
        }
        // Rotation indicator: an arc-like pair of segments at chip center.
        const float rot = algo_.rotation();
        if (std::fabs(rot) > 1e-3F) {
            const int R = 40;
            const float a0 = 0.0F;
            const float a1 = rot * 0.5F;
            OverlayLine rl;
            rl.x1 = cx + static_cast<int>(std::cos(a0) * R);
            rl.y1 = cy + static_cast<int>(std::sin(a0) * R);
            rl.x2 = cx + static_cast<int>(std::cos(a1) * R);
            rl.y2 = cy + static_cast<int>(std::sin(a1) * R);
            r.lines.push_back(rl);
        }
        // Direction histogram text.
        const auto& hist = algo_.global_histogram();
        OverlayText t;
        t.x = 10; t.y = 40;
        t.text = "dir: ";
        t.text.reserve(128);
        for (int i = 0; i < gui_algo::DirectionSelectiveFilter::kNumDirections; ++i) {
            t.text += std::to_string(i * 45) + "=" + std::to_string(hist[i]) + " ";
        }
        r.texts.push_back(t);
        // Motion text overlay.
        OverlayText mt;
        mt.x = 10; mt.y = 60;
        mt.text = "trans=(" + std::to_string(static_cast<int>(tr.x)) + "," +
                  std::to_string(static_cast<int>(tr.y)) + ") rot=" +
                  std::to_string(static_cast<int>(rot * 180.0F / static_cast<float>(kPiF))) +
                  " exp=" + std::to_string(algo_.expansion());
        r.texts.push_back(mt);
        const int dom = algo_.global_direction();
        r.status = "dir_selective: dominant=" + std::to_string(dom) +
                   " colored=" + std::to_string(r.colored_events.size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_dirs_.clear(); }
};

/// BackgroundMaskFilter backend — produces background mask frame.
/// Supports ROI (design §5.6.6): only ROI events update the mask; the mask
/// remains sensor-sized so only the ROI region is updated.
class BackgroundMaskBackend final : public AlgoBackend {
    gui_algo::BackgroundMaskFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    BackgroundMaskBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "learning_rate") algo_.set_learning_window_s(static_cast<float>(to_d(v)));
        else if (k == "threshold") algo_.set_background_rate_threshold_hz(static_cast<float>(to_d(v)));
        else if (k == "erosion_size") algo_.set_erosion_size(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "learning_rate") return from_d(algo_.learning_window_s());
        if (k == "threshold") return from_d(algo_.background_rate_threshold_hz());
        if (k == "erosion_size") return from_i(algo_.erosion_size());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_.mask().clone();
        r.status = "bg_mask: active" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); }
};

/// BandpassFilter backend — event-rate band-pass, text overlay.
/// Supports ROI (design §5.6.6): rate computed from ROI events only.
class BandpassFilterBackend final : public AlgoBackend {
    gui_algo::BandpassFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    Metavision::timestamp last_t_{0};
    double low_cutoff_hz_{1.0};
    double high_cutoff_hz_{10.0};
public:
    BandpassFilterBackend(int w, int h) : algo_(1.0F, 10.0F, 1, 0.033) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "low_cutoff_hz") {
            low_cutoff_hz_ = to_d(v);
            algo_.set_cutoffs(low_cutoff_hz_, high_cutoff_hz_);
        } else if (k == "high_cutoff_hz") {
            high_cutoff_hz_ = to_d(v);
            algo_.set_cutoffs(low_cutoff_hz_, high_cutoff_hz_);
        }
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "low_cutoff_hz") return from_d(low_cutoff_hz_);
        if (k == "high_cutoff_hz") return from_d(high_cutoff_hz_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        if (!passthrough_.empty()) {
            const auto t = passthrough_.back().t;
            algo_.add_events(n, t);
            last_t_ = t;
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        OverlayText t;
        t.x = 10; t.y = 60;
        t.text = "bp: " + std::to_string(algo_.value()) + " ev/s";
        r.texts.push_back(t);
        r.status = "bandpass: " + std::to_string(algo_.value()) + " ev/s" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); roi_buf_.clear(); last_t_ = 0; }
};


// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_filter_backend(const std::string& name,
                                          int width, int height) {
    if (name == "orientation_filter")          return std::make_unique<OrientationFilterBackend>(width, height);
    if (name == "direction_selective")         return std::make_unique<DirectionSelectiveBackend>(width, height);
    if (name == "background_mask")             return std::make_unique<BackgroundMaskBackend>(width, height);
    if (name == "bandpass_filter")             return std::make_unique<BandpassFilterBackend>(width, height);
    return nullptr;
}

} // namespace gui
