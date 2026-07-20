// gui/algo_bridge/backends/cv_backends.cpp — in-place filters + overlay detectors
// (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "algo/cv/hot_pixel_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/object_tracker.h"
#include "algo/cv/corner_detector.h"
#include "algo/cv/blob_detector.h"
#include "algo/cv/sparse_optical_flow.h"

using namespace gui::backend_detail;

namespace gui {

// ===========================================================================
// Group A: In-place event filters (compact / modify events)
// ===========================================================================

/// HotPixelFilter backend — learns hot-pixel mask + compacts events.
class HotPixelFilterBackend final : public AlgoBackend {
    gui_algo::HotPixelFilter algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::size_t last_kept_{0};
public:
    HotPixelFilterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "learning_window_s") algo_.set_learning_window_s(to_d(v));
        else if (k == "enable_fpn_correction") algo_.set_enable_fpn_correction(to_b(v));
        else if (k == "fpn_target_rate_hz") algo_.set_fpn_target_rate_hz(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "learning_window_s") return from_d(algo_.learning_window_s());
        if (k == "enable_fpn_correction") return from_b(algo_.enable_fpn_correction());
        if (k == "fpn_target_rate_hz") return from_d(algo_.fpn_target_rate_hz());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto [ev_c, n] = roi_.apply(as_events(buf_.data()), buf_.size(), roi_buf_);
        auto* ev = const_cast<gui_algo::Event*>(ev_c);
        algo_.learn(ev, n);
        last_kept_ = algo_.process(ev, n);
        // pull_result reads from buf_: if roi_.apply() filtered into another
        // buffer (roi_buf_ or Preprocessor::buf_), copy the compacted result
        // back; otherwise the in-place compact already happened in buf_.
        if (ev_c != as_events(buf_.data())) {
            buf_.assign(reinterpret_cast<const Metavision::EventCD*>(ev_c),
                        reinterpret_cast<const Metavision::EventCD*>(ev_c + last_kept_));
        } else {
            buf_.resize(last_kept_);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events.assign(buf_.data(), buf_.data() + last_kept_);
        r.status = "hot_pixel: " + std::to_string(algo_.hot_pixel_count()) + " hot px, kept " +
                   std::to_string(last_kept_) + std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); roi_buf_.clear(); last_kept_ = 0; }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // The algo holds a sensor-sized hot-pixel mask — rebuild it at the
        // new dimensions (params revert to ctor defaults; MainWindow resets
        // instances on source change anyway).
        algo_ = gui_algo::HotPixelFilter(w, h);
    }
};

/// OpticalGyro (EIS) backend — modifies event coordinates in place.
class OpticalGyroBackend final : public AlgoBackend {
    gui_algo::OpticalGyro algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    OpticalGyroBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "stabilize") algo_.set_stabilization_strength(to_b(v) ? 1.0F : 0.0F);
        else if (k == "smoothing_window_ms") algo_.set_smoothing_window_ms(static_cast<float>(to_d(v)));
        else if (k == "rotation_enabled") algo_.set_rotation_enabled(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "stabilize") return from_b(algo_.stabilization_strength() > 0.0F);
        if (k == "smoothing_window_ms") return from_d(algo_.smoothing_window_ms());
        if (k == "rotation_enabled") return from_b(algo_.rotation_enabled());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto [ev_c, n] = roi_.apply(as_events(buf_.data()), buf_.size(), roi_buf_);
        auto* ev = const_cast<gui_algo::Event*>(ev_c);
        gui_algo::MutableEventPacket pkt(ev, n);
        algo_.process(pkt);
        // pull_result reads from buf_: if roi_.apply() filtered into another
        // buffer, copy the processed events back; otherwise they are in buf_.
        if (ev_c != as_events(buf_.data())) {
            buf_.assign(reinterpret_cast<const Metavision::EventCD*>(ev_c),
                        reinterpret_cast<const Metavision::EventCD*>(ev_c + n));
        } else {
            buf_.resize(n);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = buf_;
        const auto m = algo_.smoothed_motion();
        // Translation vector (jAER OpticalGyro.drawVector):
        // arrow from chip center showing cumulative shift.
        const int cx = algo_.width() / 2;
        const int cy = algo_.height() / 2;
        const float scale = 5.0F;  // px/px for visibility
        OverlayLine tl;
        tl.x1 = cx; tl.y1 = cy;
        tl.x2 = cx + static_cast<int>(m.dx * scale);
        tl.y2 = cy + static_cast<int>(m.dy * scale);
        r.lines.push_back(tl);
        // Rotation indicator (jAER rotationAngle): an arc segment.
        if (std::fabs(m.dtheta) > 1e-3F) {
            const int R = 50;
            const float a0 = -static_cast<float>(kPiF) * 0.25F;
            const float a1 = a0 + m.dtheta * static_cast<float>(kPiF) / 180.0F * 2.0F;
            OverlayLine rl;
            rl.x1 = cx + static_cast<int>(std::cos(a0) * R);
            rl.y1 = cy + static_cast<int>(std::sin(a0) * R);
            rl.x2 = cx + static_cast<int>(std::cos(a1) * R);
            rl.y2 = cy + static_cast<int>(std::sin(a1) * R);
            r.lines.push_back(rl);
        }
        // Motion text overlay.
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "EIS trans=(" + std::to_string(static_cast<int>(m.dx)) + "," +
                  std::to_string(static_cast<int>(m.dy)) + ") rot=" +
                  std::to_string(static_cast<int>(m.dtheta)) + "deg";
        r.texts.push_back(t);
        r.status = "EIS: shift=(" + std::to_string(m.dx) + "," +
                   std::to_string(m.dy) + ") rot=" + std::to_string(m.dtheta) + "deg" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); roi_buf_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::OpticalGyro(w, h);  // sensor-sized motion maps
    }
};

// ===========================================================================
// Group B: Overlay detectors (process events, produce overlay data)
// ===========================================================================

/// ObjectTracker backend — tracked objects as overlay boxes + trajectories.
class ObjectTrackerBackend final : public AlgoBackend {
    gui_algo::ObjectTracker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    ObjectTrackerBackend(int w, int h)
        : algo_(w, h, gui_algo::ObjectTracker::Mode::RCT) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::ObjectTracker::Mode>(m));
        } else if (k == "cluster_size_px") algo_.set_cluster_size_px(to_i(v));
        else if (k == "cluster_time_us") algo_.set_cluster_time_us(to_i(v));
        else if (k == "min_cluster_events") algo_.set_min_cluster_events(to_i(v));
        else if (k == "max_lost_age_s") algo_.set_max_lost_age_s(to_d(v));
        else if (k == "enable_velocity_prediction") algo_.set_enable_velocity_prediction(to_b(v));
        else if (k == "location_mixing_factor") algo_.set_location_mixing_factor(static_cast<float>(to_d(v)));
        else if (k == "predictive_velocity_factor") algo_.set_predictive_velocity_factor(static_cast<float>(to_d(v)));
        else if (k == "mass_decay_tau_us") algo_.set_mass_decay_tau_us(to_i(v));
        else if (k == "threshold_mass_for_visible") algo_.set_threshold_mass_for_visible(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "cluster_size_px") return from_i(algo_.cluster_size_px());
        if (k == "cluster_time_us") return from_i(algo_.cluster_time_us());
        if (k == "min_cluster_events") return from_i(algo_.min_cluster_events());
        if (k == "max_lost_age_s") return from_d(algo_.max_lost_age_s());
        if (k == "enable_velocity_prediction") return from_b(algo_.enable_velocity_prediction());
        if (k == "location_mixing_factor") return from_d(algo_.location_mixing_factor());
        if (k == "predictive_velocity_factor") return from_d(algo_.predictive_velocity_factor());
        if (k == "mass_decay_tau_us") return from_i(algo_.mass_decay_tau_us());
        if (k == "threshold_mass_for_visible") return from_d(algo_.threshold_mass_for_visible());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const auto& objs = algo_.objects();
        r.boxes.reserve(objs.size());
        r.lines.reserve(objs.size());
        r.texts.reserve(objs.size());
        r.trajectories.reserve(objs.size());
        for (const auto& o : objs) {
            if (!o.visible) continue;
            OverlayBox box;
            box.x = o.bbox.x; box.y = o.bbox.y;
            box.w = o.bbox.width; box.h = o.bbox.height;
            box.id = o.id;
            r.boxes.push_back(box);
            // Velocity arrow (jAER Cluster.drawVelocityVector).
            OverlayLine v;
            v.x1 = static_cast<int>(o.x);
            v.y1 = static_cast<int>(o.y);
            // Scale velocity (px/s) to a visible length (assume 30ms window).
            v.x2 = v.x1 + static_cast<int>(o.vx * 0.03F);
            v.y2 = v.y1 + static_cast<int>(o.vy * 0.03F);
            r.lines.push_back(v);
            // Trajectory (jAER ClusterPath).
            if (!o.trajectory.empty()) {
                OverlayTrajectory tr;
                tr.id = o.id;
                tr.points.reserve(o.trajectory.size());
                for (const auto& p : o.trajectory) {
                    tr.points.emplace_back(static_cast<int>(p.x),
                                            static_cast<int>(p.y));
                }
                r.trajectories.push_back(tr);
            }
            OverlayText t;
            t.x = o.bbox.x; t.y = o.bbox.y > 12 ? o.bbox.y - 12 : 0;
            t.text = "#" + std::to_string(o.id) + " v=(" +
                     std::to_string(static_cast<int>(o.vx)) + "," +
                     std::to_string(static_cast<int>(o.vy)) + ")";
            r.texts.push_back(t);
        }
        r.status = "tracker: " + std::to_string(objs.size()) + " objects" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // Rebuild sensor-sized internal state, preserving the current mode
        // (other params revert to ctor defaults).
        algo_ = gui_algo::ObjectTracker(w, h, algo_.mode());
    }
};

/// CornerDetector backend — corners as overlay points.
class CornerDetectorBackend final : public AlgoBackend {
    gui_algo::CornerDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    CornerDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) algo_.set_mode(static_cast<gui_algo::CornerDetector::Mode>(m));
        } else if (k == "min_score") algo_.set_threshold(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "min_score") return from_d(algo_.threshold());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : algo_.corners()) {
            OverlayPoint p;
            p.x = static_cast<int>(c.x);
            p.y = static_cast<int>(c.y);
            p.strength = c.strength;
            r.points.push_back(p);
        }
        r.status = "corners: " + std::to_string(algo_.corners().size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::CornerDetector(w, h, algo_.mode());
    }
};

/// BlobDetector backend — blobs as overlay boxes.
class BlobDetectorBackend final : public AlgoBackend {
    gui_algo::BlobDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    BlobDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "threshold") algo_.set_threshold(static_cast<int>(to_d(v)));
        else if (k == "learning_rate") algo_.set_learning_rate(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "threshold") return from_d(algo_.threshold());
        if (k == "learning_rate") return from_d(algo_.learning_rate());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& blob : algo_.blobs()) {
            OverlayBox box;
            box.x = blob.bbox.x; box.y = blob.bbox.y;
            box.w = blob.bbox.width; box.h = blob.bbox.height;
            r.boxes.push_back(box);
        }
        r.status = "blobs: " + std::to_string(algo_.blobs().size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::BlobDetector(w, h);  // sensor-sized background map
    }
};

/// SparseOpticalFlow backend — flow vectors as overlay lines.
class SparseOpticalFlowBackend final : public AlgoBackend {
    gui_algo::SparseOpticalFlow algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::FlowVector> flows_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    SparseOpticalFlowBackend(int w, int h)
        : algo_(w, h, gui_algo::SparseOpticalFlow::Mode::LocalPlanes) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::SparseOpticalFlow::Mode>(m));
        } else if (k == "search_radius") algo_.set_search_radius_px(to_i(v));
        else if (k == "time_window_us") algo_.set_time_window_us(to_i(v));
        else if (k == "cluster_ema_alpha") algo_.set_cluster_ema_alpha(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "search_radius") return from_i(algo_.search_radius_px());
        if (k == "time_window_us") return from_i(algo_.time_window_us());
        if (k == "cluster_ema_alpha") return from_d(algo_.cluster_ema_alpha());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        flows_.clear();
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n, flows_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Optical-flow visualization: render each flow vector as a single
        // colored pixel/square at its origin. Color encodes direction (HSV
        // hue 0..360° mapped to the (vx,vy) angle) and brightness encodes
        // magnitude. This is the standard dense-flow visualization style
        // (Farneback/OpenCV hue-intensity scheme) and avoids the previous
        // "long lines all over the screen" problem caused by drawing
        // vx*10/vy*10 line segments.
        r.colored_points.reserve(flows_.size());
        for (const auto& f : flows_) {
            OverlayColoredPoint cp;
            cp.x = f.x;
            cp.y = f.y;
            // Direction -> hue (atan2 returns -pi..pi, map to 0..1).
            const float angle_rad = std::atan2(f.vy, f.vx);
            static constexpr float kInv2Pi = 1.0F / (2.0F * static_cast<float>(M_PI));
            float hue = (angle_rad + static_cast<float>(M_PI)) * kInv2Pi;
            if (hue < 0.0f) hue = 0.0f;
            if (hue > 1.0f) hue = 1.0f;
            // Magnitude -> value (normalize against a typical px/s reference).
            const float mag = std::sqrt(f.vx * f.vx + f.vy * f.vy);
            static constexpr float kInv1000 = 1.0F / 1000.0F;
            const float val = std::min(1.0f, mag * kInv1000);  // 1000 px/s -> full brightness
            hsv_to_rgb(hue, 1.0f, val, cp.r, cp.g, cp.b);
            r.colored_points.push_back(cp);
        }
        r.status = "flow: " + std::to_string(flows_.size()) + " vectors" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); flows_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::SparseOpticalFlow(w, h, algo_.mode());
    }
private:
    /// HSV (h,s,v in [0,1]) -> RGB (r,g,b in [0,255]).
    static void hsv_to_rgb(float h, float s, float v,
                           std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) {
        const float c = v * s;
        const float hp = h * 6.0f;
        const float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
        float rf = 0.0f, gf = 0.0f, bf = 0.0f;
        if (hp < 1.0f)      { rf = c; gf = x; bf = 0.0f; }
        else if (hp < 2.0f) { rf = x; gf = c; bf = 0.0f; }
        else if (hp < 3.0f) { rf = 0.0f; gf = c; bf = x; }
        else if (hp < 4.0f) { rf = 0.0f; gf = x; bf = c; }
        else if (hp < 5.0f) { rf = x; gf = 0.0f; bf = c; }
        else                { rf = c; gf = 0.0f; bf = x; }
        const float m = v - c;
        r = static_cast<std::uint8_t>((rf + m) * 255.0f + 0.5f);
        g = static_cast<std::uint8_t>((gf + m) * 255.0f + 0.5f);
        b = static_cast<std::uint8_t>((bf + m) * 255.0f + 0.5f);
    }
};


// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_cv_backend(const std::string& name,
                                          int width, int height) {
    if (name == "hot_pixel_filter")            return std::make_unique<HotPixelFilterBackend>(width, height);
    if (name == "optical_gyro")                return std::make_unique<OpticalGyroBackend>(width, height);
    if (name == "object_tracker")              return std::make_unique<ObjectTrackerBackend>(width, height);
    if (name == "corner_detector")             return std::make_unique<CornerDetectorBackend>(width, height);
    if (name == "blob_detector")               return std::make_unique<BlobDetectorBackend>(width, height);
    if (name == "sparse_optical_flow")         return std::make_unique<SparseOpticalFlowBackend>(width, height);
    return nullptr;
}

} // namespace gui
