// gui/algo_bridge/algo_backend.cpp — concrete backends wrapping real algo/ classes.
//
// 每个 backend 包装一个 algo/cv 或 algo/analytics 类：
//   - push_events: 零拷贝 reinterpret_cast EventCD→Event，调用 process()/filter()
//   - pull_result: 收集过滤事件 + 叠加层 + 帧
//   - set_param/get_param: 字符串↔类型化 setter 转换
//
// 分组模式：
//   A) In-place filter:  noise_filter, hot_pixel_filter, optical_gyro, perspective_undistort
//   B) Overlay detector: object_tracker, corner_detector, blob_detector, sparse_optical_flow
//   C) Result-vector:    hough_line, hough_circle, line_segment, orientation_cluster, cluster_lif
//   D) Frame producer:   time_surface, event_to_video, flow_statistics, isi_analyzer
//   E) Analyzer:         freq_detector, active_marker, particle_counter, auto_bias
//   F) Event-vector:     trigger_synced, ultra_slow_motion
//   G) Visualization:    xyt_visualizer, overlay
//   H) Misc filter:      orientation_filter, direction_selective, background_mask, bandpass

#include "algo_backend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

// algo/cv
#include "algo/cv/noise_filter.h"
#include "algo/cv/hot_pixel_filter.h"
#include "algo/cv/orientation_filter.h"
#include "algo/cv/direction_selective_filter.h"
#include "algo/cv/sparse_optical_flow.h"
#include "algo/cv/blob_detector.h"
#include "algo/cv/object_tracker.h"
#include "algo/cv/corner_detector.h"
#include "algo/cv/line_segment_detector.h"
#include "algo/cv/hough_line_tracker.h"
#include "algo/cv/hough_circle_tracker.h"
#include "algo/cv/orientation_cluster.h"
#include "algo/cv/cluster_lif.h"
#include "algo/cv/background_mask_filter.h"
#include "algo/cv/perspective_undistort.h"
#include "algo/cv/trigger_synced_filter.h"
#include "algo/cv/bandpass_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/ultra_slow_motion.h"
#include "algo/cv/xyt_visualizer.h"
#include "algo/cv/time_surface.h"
// NOTE: algo/cv/overlay.h intentionally NOT included — it redefines
// FlowVector/TrackedObject/Corner already defined in the per-algo headers
// above (sparse_optical_flow.h, object_tracker.h, corner_detector.h).
// OverlayBackend below is a pure pass-through; overlay drawing is handled
// by gui/display/frame_annotator from AlgoResult overlay vectors.

// algo/analytics
#include "algo/analytics/active_marker.h"
#include "algo/analytics/event_to_video.h"
#include "algo/analytics/flow_statistics.h"
#include "algo/analytics/isi_analyzer.h"
#include "algo/analytics/particle_counter.h"
#include "algo/analytics/auto_bias_controller.h"
#include "algo/analytics/freq_detector.h"

namespace gui {

namespace {

// ---- string ↔ value helpers -----------------------------------------------

int to_i(const std::string& s, int def = 0) {
    try { return std::stoi(s); } catch (...) { return def; }
}
double to_d(const std::string& s, double def = 0.0) {
    try { return std::stod(s); } catch (...) { return def; }
}
bool to_b(const std::string& s) {
    return s == "1" || s == "true" || s == "True" || s == "on" || s == "yes";
}
std::string from_i(int v) { return std::to_string(v); }
std::string from_d(double v) { return std::to_string(v); }
std::string from_b(bool v) { return v ? "true" : "false"; }

/// Zero-copy view of EventCD buffer as gui_algo::Event (layout-compatible).
const gui_algo::Event* as_events(const Metavision::EventCD* p) {
    static_assert(sizeof(gui_algo::Event) == sizeof(Metavision::EventCD),
                  "layout mismatch");
    return reinterpret_cast<const gui_algo::Event*>(p);
}

} // namespace

// ===========================================================================
// Group A: In-place event filters (compact / modify events)
// ===========================================================================

/// NoiseFilter backend — 8-mode denoiser, compacts events in place.
class NoiseFilterBackend final : public AlgoBackend {
    gui_algo::NoiseFilter algo_;
    std::vector<Metavision::EventCD> buf_;
    std::size_t last_kept_{0};
    double last_rate_{0.0};
public:
    NoiseFilterBackend(int w, int h)
        : algo_(w, h, gui_algo::NoiseFilter::Mode::STCF) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 7) algo_.set_mode(static_cast<gui_algo::NoiseFilter::Mode>(m));
        } else if (k == "correlation_time_s") algo_.set_correlation_time_s(to_d(v));
        else if (k == "min_neighbors") algo_.set_min_neighbors(to_i(v));
        else if (k == "baf_dt_us") algo_.set_baf_dt_us(to_i(v));
        else if (k == "refractory_us") algo_.set_refractory_period_us(to_i(v));
        else if (k == "filter_hot_pixels") algo_.set_filter_hot_pixels(to_b(v));
        else if (k == "adaptive_correlation_time") algo_.set_adaptive_correlation_time(to_b(v));
        else if (k == "line_freq_hz") algo_.set_line_freq(to_i(v) == 60 ? gui_algo::NoiseFilter::LineFreq::Hz60 : gui_algo::NoiseFilter::LineFreq::Hz50);
    }
    std::string get_param(const std::string& k) const override {
        if (k == "correlation_time_s") return from_d(algo_.correlation_time_s());
        if (k == "min_neighbors") return from_i(algo_.min_neighbors());
        if (k == "filter_hot_pixels") return from_b(algo_.filter_hot_pixels());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        last_kept_ = algo_.filter(ev, buf_.size());
        last_rate_ = algo_.filter_rate();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events.assign(buf_.data(), buf_.data() + last_kept_);
        r.status = "noise_filter: kept " + std::to_string(last_kept_) + "/" +
                   std::to_string(buf_.size()) + " (" + std::to_string(last_rate_ * 100) + "%)";
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); last_kept_ = 0; }
};

/// HotPixelFilter backend — learns hot-pixel mask + compacts events.
class HotPixelFilterBackend final : public AlgoBackend {
    gui_algo::HotPixelFilter algo_;
    std::vector<Metavision::EventCD> buf_;
    std::size_t last_kept_{0};
public:
    HotPixelFilterBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "learning_window_s") algo_.set_learning_window_s(to_d(v));
        else if (k == "n_sigma") algo_.set_n_sigma(to_d(v));
        else if (k == "enable_fpn_correction") algo_.set_enable_fpn_correction(to_b(v));
        else if (k == "fpn_target_rate_hz") algo_.set_fpn_target_rate_hz(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        if (k == "n_sigma") return from_d(algo_.n_sigma());
        if (k == "enable_fpn_correction") return from_b(algo_.enable_fpn_correction());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        algo_.learn(as_events(buf_.data()), buf_.size());
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        last_kept_ = algo_.process(ev, buf_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events.assign(buf_.data(), buf_.data() + last_kept_);
        r.status = "hot_pixel: " + std::to_string(algo_.hot_pixel_count()) + " hot px, kept " + std::to_string(last_kept_);
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); last_kept_ = 0; }
};

/// OpticalGyro (EIS) backend — modifies event coordinates in place.
class OpticalGyroBackend final : public AlgoBackend {
    gui_algo::OpticalGyro algo_;
    std::vector<Metavision::EventCD> buf_;
public:
    OpticalGyroBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "stabilize") algo_.set_stabilization_strength(to_b(v) ? 1.0F : 0.0F);
        else if (k == "smoothing_window_ms") algo_.set_smoothing_window_ms(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        if (k == "stabilize") return from_b(algo_.stabilization_strength() > 0.0F);
        if (k == "smoothing_window_ms") return from_d(algo_.smoothing_window_ms());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        gui_algo::MutableEventPacket pkt(ev, buf_.size());
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = buf_;
        const auto m = algo_.smoothed_motion();
        r.status = "EIS: shift=(" + std::to_string(m.dx) + "," +
                   std::to_string(m.dy) + ")";
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); }
};

/// PerspectiveUndistort backend — remaps event coordinates in place.
class PerspectiveUndistortBackend final : public AlgoBackend {
    gui_algo::PerspectiveUndistort algo_;
    std::vector<Metavision::EventCD> buf_;
public:
    PerspectiveUndistortBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "enable") algo_.set_undistort(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        if (k == "enable") return from_b(algo_.undistort());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto* ev = const_cast<gui_algo::Event*>(as_events(buf_.data()));
        gui_algo::MutableEventPacket pkt(ev, buf_.size());
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = buf_;
        r.status = std::string("undistort: ") + (algo_.undistort() ? "on" : "off");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); }
};

// ===========================================================================
// Group B: Overlay detectors (process events, produce overlay data)
// ===========================================================================

/// ObjectTracker backend — tracked objects as overlay boxes.
class ObjectTrackerBackend final : public AlgoBackend {
    gui_algo::ObjectTracker algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    ObjectTrackerBackend(int w, int h)
        : algo_(w, h, gui_algo::ObjectTracker::Mode::RCT) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::ObjectTracker::Mode>(m));
        } else if (k == "cluster_size_px") algo_.set_cluster_size_px(to_i(v));
        else if (k == "cluster_time_us") algo_.set_cluster_time_us(to_i(v));
        else if (k == "min_cluster_events") algo_.set_min_cluster_events(to_i(v));
        else if (k == "max_lost_age_s") algo_.set_max_lost_age_s(to_d(v));
        else if (k == "enable_velocity_prediction") algo_.set_enable_velocity_prediction(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        if (k == "cluster_size_px") return from_i(algo_.cluster_size_px());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const auto& objs = algo_.objects();
        for (const auto& o : objs) {
            if (!o.visible) continue;
            OverlayBox box;
            box.x = o.bbox.x; box.y = o.bbox.y;
            box.w = o.bbox.width; box.h = o.bbox.height;
            box.id = o.id;
            r.boxes.push_back(box);
            OverlayText t;
            t.x = o.bbox.x; t.y = o.bbox.y > 12 ? o.bbox.y - 12 : 0;
            t.text = "#" + std::to_string(o.id) + " v=(" +
                     std::to_string(static_cast<int>(o.vx)) + "," +
                     std::to_string(static_cast<int>(o.vy)) + ")";
            r.texts.push_back(t);
        }
        r.status = "tracker: " + std::to_string(objs.size()) + " objects";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// CornerDetector backend — corners as overlay points.
class CornerDetectorBackend final : public AlgoBackend {
    gui_algo::CornerDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    CornerDetectorBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) algo_.set_mode(static_cast<gui_algo::CornerDetector::Mode>(m));
        } else if (k == "min_score") algo_.set_threshold(to_d(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
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
        r.status = "corners: " + std::to_string(algo_.corners().size());
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// BlobDetector backend — blobs as overlay boxes.
class BlobDetectorBackend final : public AlgoBackend {
    gui_algo::BlobDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    BlobDetectorBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "threshold") algo_.set_threshold(static_cast<int>(to_d(v)));
        else if (k == "learning_rate") algo_.set_learning_rate(to_d(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
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
        r.status = "blobs: " + std::to_string(algo_.blobs().size());
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// SparseOpticalFlow backend — flow vectors as overlay lines.
class SparseOpticalFlowBackend final : public AlgoBackend {
    gui_algo::SparseOpticalFlow algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::FlowVector> flows_;
public:
    SparseOpticalFlowBackend(int w, int h)
        : algo_(w, h, gui_algo::SparseOpticalFlow::Mode::LocalPlanes) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::SparseOpticalFlow::Mode>(m));
        } else if (k == "search_radius") algo_.set_search_radius_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        flows_.clear();
        algo_.process(as_events(passthrough_.data()), passthrough_.size(), flows_);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& f : flows_) {
            OverlayLine line;
            line.x1 = f.x; line.y1 = f.y;
            line.x2 = f.x + static_cast<int>(f.vx * 10);
            line.y2 = f.y + static_cast<int>(f.vy * 10);
            r.lines.push_back(line);
        }
        r.status = "flow: " + std::to_string(flows_.size()) + " vectors";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); flows_.clear(); }
};

// ===========================================================================
// Group C: Result-vector detectors (process returns vector<Result>)
// ===========================================================================

/// HoughLineTracker backend — detected lines as overlay lines.
class HoughLineBackend final : public AlgoBackend {
    gui_algo::HoughLineTracker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::HoughLine> last_;
public:
    HoughLineBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "threshold") algo_.set_threshold(to_i(v));
        else if (k == "num_theta_bins") algo_.set_num_theta_bins(to_i(v));
        else if (k == "num_rho_bins") algo_.set_num_rho_bins(to_i(v));
        else if (k == "accumulator_decay_us")
            algo_.set_accumulator_decay_us(static_cast<Metavision::timestamp>(to_i(v)));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        gui_algo::EventPacket pkt(as_events(passthrough_.data()), passthrough_.size());
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& hl : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(hl.start.x); l.y1 = static_cast<int>(hl.start.y);
            l.x2 = static_cast<int>(hl.end.x); l.y2 = static_cast<int>(hl.end.y);
            r.lines.push_back(l);
        }
        r.status = "hough_line: " + std::to_string(last_.size()) + " lines";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// HoughCircleTracker backend — detected circles as overlay circles.
class HoughCircleBackend final : public AlgoBackend {
    gui_algo::HoughCircleTracker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::HoughCircle> last_;
public:
    HoughCircleBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "min_radius") algo_.set_min_radius_px(to_i(v));
        else if (k == "max_radius") algo_.set_max_radius_px(to_i(v));
        else if (k == "threshold") algo_.set_threshold(to_i(v));
        else if (k == "accumulator_decay_us")
            algo_.set_accumulator_decay_us(static_cast<Metavision::timestamp>(to_i(v)));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        gui_algo::EventPacket pkt(as_events(passthrough_.data()), passthrough_.size());
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayCircle oc;
            oc.cx = static_cast<int>(c.center.x); oc.cy = static_cast<int>(c.center.y); oc.r = static_cast<int>(c.radius);
            r.circles.push_back(oc);
        }
        r.status = "hough_circle: " + std::to_string(last_.size()) + " circles";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// LineSegmentDetector (ELiSeD) backend — detected segments as overlay lines.
class LineSegmentBackend final : public AlgoBackend {
    gui_algo::LineSegmentDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LineSegment> last_;
public:
    LineSegmentBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "min_length") algo_.set_min_line_length_px(to_i(v));
        else if (k == "gap") algo_.set_max_line_gap_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        gui_algo::EventPacket pkt(as_events(passthrough_.data()), passthrough_.size());
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& seg : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(seg.start.x); l.y1 = static_cast<int>(seg.start.y);
            l.x2 = static_cast<int>(seg.end.x); l.y2 = static_cast<int>(seg.end.y);
            r.lines.push_back(l);
        }
        r.status = "elised: " + std::to_string(last_.size()) + " segments";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// OrientationCluster backend — detected orientation clusters as overlay lines.
class OrientationClusterBackend final : public AlgoBackend {
    gui_algo::OrientationCluster algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::OrientationClusterResult> last_;
public:
    OrientationClusterBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "min_events") algo_.set_min_cluster_size(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        gui_algo::EventPacket pkt(as_events(passthrough_.data()), passthrough_.size());
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(c.centroid.x); l.y1 = static_cast<int>(c.centroid.y);
            l.x2 = static_cast<int>(c.centroid.x) + static_cast<int>(std::cos(c.orientation) * c.size);
            l.y2 = static_cast<int>(c.centroid.y) + static_cast<int>(std::sin(c.orientation) * c.size);
            r.lines.push_back(l);
        }
        r.status = "orient_cluster: " + std::to_string(last_.size()) + " clusters";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// ClusterLIF backend — LIF clusters as overlay boxes.
class ClusterLifBackend final : public AlgoBackend {
    gui_algo::ClusterLIF algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LifCluster> last_;
public:
    ClusterLifBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "tau_ms") algo_.set_tau_ms(static_cast<float>(to_d(v)));
        else if (k == "threshold") algo_.set_threshold(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        gui_algo::EventPacket pkt(as_events(passthrough_.data()), passthrough_.size());
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayBox box;
            box.x = static_cast<int>(c.position.x) - 5;
            box.y = static_cast<int>(c.position.y) - 5;
            box.w = 10; box.h = 10;
            r.boxes.push_back(box);
        }
        r.status = "lif: " + std::to_string(last_.size()) + " clusters";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

// ===========================================================================
// Group D: Frame producers (process events, produce cv::Mat frame)
// ===========================================================================

/// TimeSurface backend — produces a time-decay pseudo-color frame.
class TimeSurfaceBackend final : public AlgoBackend {
    gui_algo::TimeSurface algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    TimeSurfaceBackend(int w, int h)
        : algo_(w, h, gui_algo::TimeSurface::Channels::Merged,
                100000, gui_algo::TimeSurface::Palette::Hot, 30) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "decay_time_us") algo_.set_decay_time_us(to_i(v));
        else if (k == "palette") {
            int p = to_i(v);
            if (p >= 0 && p <= 3) algo_.set_palette(static_cast<gui_algo::TimeSurface::Palette>(p));
        } else if (k == "channels") {
            int c = to_i(v);
            algo_.set_channels(c == 2 ? gui_algo::TimeSurface::Channels::Split : gui_algo::TimeSurface::Channels::Merged);
        }
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_.render().clone();
        r.status = "time_surface: rendered";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// EventToVideo backend — produces reconstructed intensity frame.
class EventToVideoBackend final : public AlgoBackend {
    gui_algo::EventToVideo algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    EventToVideoBackend(int w, int h)
        : algo_(w, h, gui_algo::EventToVideo::Mode::BardowVariational) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) algo_.set_mode(static_cast<gui_algo::EventToVideo::Mode>(m));
        }
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_.get_frame().clone();
        r.status = "e2v: reconstructed";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// FlowStatistics backend — requires ground-truth flow samples (not available
/// in real-time). Counts events and reports a status; no frame is produced.
class FlowStatisticsBackend final : public AlgoBackend {
    gui_algo::FlowStatistics algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::size_t total_events_{0};
public:
    FlowStatisticsBackend(int /*w*/, int /*h*/) : algo_(gui_algo::FlowStatistics::Source::Synthetic, 5) {}
    void set_param(const std::string& k, const std::string& v) override {}
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        total_events_ += passthrough_.size();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "flow_stats: " + std::to_string(total_events_) +
                   " events (no GT — Passive mode)";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); total_events_ = 0; }
};

/// ISIAnalyzer backend — renders ISI histogram as frame.
class ISIAnalyzerBackend final : public AlgoBackend {
    gui_algo::ISIAnalyzer algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    ISIAnalyzerBackend(int w, int h) : algo_(w, h, 32, 100.0F, false) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "per_pixel") algo_.set_per_pixel(to_b(v));
        else if (k == "max_isi_ms") algo_.set_max_isi_ms(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_.render();
        r.status = "isi: histogram";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

// ===========================================================================
// Group E: Analyzers (process events, produce text/point overlay)
// ===========================================================================

/// FreqDetector backend — detected light sources as overlay circles + text.
class FreqDetectorBackend final : public AlgoBackend {
    gui_algo::FreqDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LightSource> last_;
public:
    FreqDetectorBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "update_interval_s") algo_.set_update_interval_s(static_cast<float>(to_d(v)));
        else if (k == "min_events") algo_.set_min_cc_area(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
        if (algo_.should_analyze()) last_ = algo_.analyze();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& src : last_) {
            OverlayCircle c;
            c.cx = src.u; c.cy = src.v; c.r = 8;
            r.circles.push_back(c);
            OverlayText t;
            t.x = src.u + 12; t.y = src.v;
            t.text = std::to_string(static_cast<int>(src.blink_freq_hz)) + "Hz";
            r.texts.push_back(t);
        }
        r.status = "freq: " + std::to_string(last_.size()) + " sources";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// ActiveMarker backend — detected markers as overlay circles + text.
class ActiveMarkerBackend final : public AlgoBackend {
    gui_algo::ActiveMarker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::ClusterAnnotation> last_;
public:
    ActiveMarkerBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "window_us") algo_.set_window_ms(static_cast<float>(to_i(v)) / 1000.0F);
        else if (k == "min_events") algo_.set_min_cluster_area(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
        last_ = algo_.analyze();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& m : last_) {
            OverlayCircle c;
            c.cx = m.cx; c.cy = m.cy; c.r = 6;
            r.circles.push_back(c);
            OverlayText t;
            t.x = m.cx + 10; t.y = m.cy;
            t.text = std::to_string(static_cast<int>(m.frequency_hz)) + "Hz";
            r.texts.push_back(t);
        }
        r.status = "marker: " + std::to_string(last_.size()) + " markers";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
};

/// ParticleCounter backend — count as overlay text.
class ParticleCounterBackend final : public AlgoBackend {
    gui_algo::ParticleCounter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    Metavision::timestamp last_t_{0};
public:
    ParticleCounterBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "line_y") algo_.set_counting_line_y(to_i(v));
        else if (k == "min_area") algo_.set_min_particle_size_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (!passthrough_.empty()) last_t_ = passthrough_.back().t;
        algo_.process(as_events(passthrough_.data()), passthrough_.size(), last_t_);
        algo_.detect_and_track();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "count: " + std::to_string(algo_.cumulative_count());
        r.texts.push_back(t);
        r.status = "counter: " + std::to_string(algo_.cumulative_count());
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_t_ = 0; }
};

/// AutoBiasController backend — bias command as overlay text.
class AutoBiasBackend final : public AlgoBackend {
    gui_algo::AutoBiasController algo_;
    std::vector<Metavision::EventCD> passthrough_;
    Metavision::timestamp last_t_{0};
    gui_algo::BiasCommand last_cmd_;
public:
    AutoBiasBackend(int w, int h) : algo_(5.0F, 0.5F, 0.01F, 0.0F, 10.0F) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "target_event_rate_mev") algo_.set_target_event_rate_mev(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        if (k == "target_event_rate_mev") return from_d(algo_.target_event_rate_mev());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        const auto count = passthrough_.size();
        Metavision::timestamp cur_t = passthrough_.empty() ? last_t_ : passthrough_.back().t;
        const auto dt = cur_t - last_t_;
        if (dt > 0) {
            const double rate_mev = static_cast<double>(count) / (static_cast<double>(dt) * 1e-6) / 1e6;
            last_cmd_ = algo_.update(rate_mev, dt);
        }
        last_t_ = cur_t;
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        OverlayText t;
        t.x = 10; t.y = 40;
        t.text = "bias: d_diff=" + std::to_string(last_cmd_.delta_diff) +
                 " d_on=" + std::to_string(last_cmd_.delta_on);
        r.texts.push_back(t);
        r.status = "auto_bias: target=" + std::to_string(algo_.target_event_rate_mev()) + "Mev";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_t_ = 0; }
};

// ===========================================================================
// Group F: Event-vector (process returns vector<Event>)
// ===========================================================================

/// TriggerSyncedFilter backend — outputs filtered event vector.
class TriggerSyncedBackend final : public AlgoBackend {
    gui_algo::TriggerSyncedFilter algo_;
    std::vector<Metavision::EventCD> last_out_;
public:
    TriggerSyncedBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "window_us") algo_.set_trigger_window_us(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        std::vector<Metavision::EventCD> inp(b, e);
        gui_algo::EventPacket pkt(as_events(inp.data()), inp.size());
        auto out = algo_.process(pkt);
        last_out_.assign(out.begin(), out.end());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = last_out_;
        r.status = "trigger_synced: " + std::to_string(last_out_.size()) + " events";
        return r;
    }
    void reset() override { algo_.reset(); last_out_.clear(); }
};

/// UltraSlowMotion backend — outputs time-dilated event vector.
class UltraSlowMotionBackend final : public AlgoBackend {
    gui_algo::UltraSlowMotion algo_;
    std::vector<Metavision::EventCD> last_out_;
public:
    UltraSlowMotionBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "factor") algo_.set_dilation_factor(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        std::vector<Metavision::EventCD> inp(b, e);
        auto out = algo_.process(as_events(inp.data()), inp.size());
        last_out_.assign(out.begin(), out.end());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = last_out_;
        r.status = "slow_motion: " + std::to_string(last_out_.size()) + " events";
        return r;
    }
    void reset() override { algo_.reset(); last_out_.clear(); }
};

// ===========================================================================
// Group G: Visualization
// ===========================================================================

/// XYTVisualizer backend — 3D point cloud data for space_time_display.
class XYTVisualizerBackend final : public AlgoBackend {
    gui_algo::XYTVisualizer algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::XYTPoint> last_points_;
    std::size_t max_points_{50000};
public:
    XYTVisualizerBackend(int /*w*/, int /*h*/)
        : algo_(1000.0f,
                gui_algo::XYTVisualizer::ColorMode::Polarity,
                2.5f,
                false,
                false) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "time_window_us") algo_.set_time_window_ms(static_cast<float>(to_i(v)) / 1000.0F);
        else if (k == "max_points") max_points_ = static_cast<std::size_t>(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_.process(as_events(passthrough_.data()), passthrough_.size());
        last_points_ = algo_.render();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const std::size_t limit = (std::min)(max_points_, last_points_.size());
        for (std::size_t i = 0; i < limit; ++i) {
            const auto& pt = last_points_[i];
            OverlayPoint p;
            p.x = pt.x; p.y = pt.y;
            p.strength = static_cast<float>(pt.r) / 255.0F;
            r.points.push_back(p);
        }
        r.status = "xyt: " + std::to_string(last_points_.size()) + " points";
        return r;
    }
    void reset() override { algo_.clear(); passthrough_.clear(); last_points_.clear(); }
};

// ===========================================================================
// Group H: Misc filters
// ===========================================================================

/// OrientationFilter backend — orientation histogram as overlay text.
class OrientationFilterBackend final : public AlgoBackend {
    gui_algo::OrientationFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    int hist_[gui_algo::OrientationFilter::kNumOrientations]{};
public:
    OrientationFilterBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "tau_us") algo_.set_time_window_us(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        std::vector<int> out;
        algo_.process(as_events(passthrough_.data()), passthrough_.size(), out);
        for (int v : out) {
            if (v >= 0 && v < gui_algo::OrientationFilter::kNumOrientations) ++hist_[v];
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "orient: 0=" + std::to_string(hist_[0]) +
                 " 45=" + std::to_string(hist_[1]) +
                 " 90=" + std::to_string(hist_[2]) +
                 " 135=" + std::to_string(hist_[3]);
        r.texts.push_back(t);
        r.status = "orient_filter: histogram updated";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); std::fill(hist_, hist_ + 4, 0); }
};

/// DirectionSelectiveFilter backend — direction histogram as overlay text.
class DirectionSelectiveBackend final : public AlgoBackend {
    gui_algo::DirectionSelectiveFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    DirectionSelectiveBackend(int w, int h) : algo_(w, h) {
        algo_.set_enable_global_mode(true);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "tau_us") algo_.set_time_window_us(to_i(v));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        std::vector<int> out;
        algo_.process(as_events(passthrough_.data()), passthrough_.size(), out);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const auto& hist = algo_.global_histogram();
        OverlayText t;
        t.x = 10; t.y = 40;
        t.text = "dir: ";
        for (int i = 0; i < gui_algo::DirectionSelectiveFilter::kNumDirections; ++i) {
            t.text += std::to_string(i * 45) + "=" + std::to_string(hist[i]) + " ";
        }
        r.texts.push_back(t);
        const int dom = algo_.global_direction();
        r.status = "dir_selective: dominant=" + std::to_string(dom);
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// BackgroundMaskFilter backend — produces background mask frame.
class BackgroundMaskBackend final : public AlgoBackend {
    gui_algo::BackgroundMaskFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
public:
    BackgroundMaskBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "learning_rate") algo_.set_learning_window_s(static_cast<float>(to_d(v)));
        else if (k == "threshold") algo_.set_background_rate_threshold_hz(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        gui_algo::EventPacket pkt(as_events(passthrough_.data()), passthrough_.size());
        algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_.mask().clone();
        r.status = "bg_mask: active";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
};

/// BandpassFilter backend — event-rate band-pass, text overlay.
class BandpassFilterBackend final : public AlgoBackend {
    gui_algo::BandpassFilter algo_;
    std::vector<Metavision::EventCD> passthrough_;
    Metavision::timestamp last_t_{0};
    double low_cutoff_hz_{1.0};
    double high_cutoff_hz_{10.0};
public:
    BandpassFilterBackend(int w, int h) : algo_(1.0F, 10.0F, 1, 0.033) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "low_cutoff_hz") {
            low_cutoff_hz_ = to_d(v);
            algo_.set_cutoffs(low_cutoff_hz_, high_cutoff_hz_);
        } else if (k == "high_cutoff_hz") {
            high_cutoff_hz_ = to_d(v);
            algo_.set_cutoffs(low_cutoff_hz_, high_cutoff_hz_);
        }
    }
    std::string get_param(const std::string& k) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        if (!passthrough_.empty()) {
            const auto t = passthrough_.back().t;
            algo_.add_events(passthrough_.size(), t);
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
        r.status = "bandpass: " + std::to_string(algo_.value()) + " ev/s";
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_t_ = 0; }
};

/// Overlay backend — pass-through (overlay is drawn by frame_annotator from other algos).
class OverlayBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
public:
    OverlayBackend(int, int) {}
    void set_param(const std::string&, const std::string&) override {}
    std::string get_param(const std::string&) const override { return {}; }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "overlay: pass-through";
        return r;
    }
    void reset() override { passthrough_.clear(); }
};

// ===========================================================================
// Factory
// ===========================================================================

std::unique_ptr<AlgoBackend> create_algo_backend(const std::string& name,
                                                  int width, int height) {
    // algo/cv filters
    if (name == "noise_filter")          return std::make_unique<NoiseFilterBackend>(width, height);
    if (name == "hot_pixel_filter")      return std::make_unique<HotPixelFilterBackend>(width, height);
    if (name == "orientation_filter")    return std::make_unique<OrientationFilterBackend>(width, height);
    if (name == "direction_selective")   return std::make_unique<DirectionSelectiveBackend>(width, height);
    if (name == "sparse_optical_flow")   return std::make_unique<SparseOpticalFlowBackend>(width, height);
    if (name == "blob_detector")         return std::make_unique<BlobDetectorBackend>(width, height);
    if (name == "object_tracker")        return std::make_unique<ObjectTrackerBackend>(width, height);
    if (name == "corner_detector")       return std::make_unique<CornerDetectorBackend>(width, height);
    if (name == "line_segment")          return std::make_unique<LineSegmentBackend>(width, height);
    if (name == "hough_line")            return std::make_unique<HoughLineBackend>(width, height);
    if (name == "hough_circle")          return std::make_unique<HoughCircleBackend>(width, height);
    if (name == "orientation_cluster")   return std::make_unique<OrientationClusterBackend>(width, height);
    if (name == "cluster_lif")           return std::make_unique<ClusterLifBackend>(width, height);
    if (name == "background_mask")       return std::make_unique<BackgroundMaskBackend>(width, height);
    if (name == "perspective_undistort") return std::make_unique<PerspectiveUndistortBackend>(width, height);
    if (name == "trigger_synced")        return std::make_unique<TriggerSyncedBackend>(width, height);
    if (name == "bandpass_filter")       return std::make_unique<BandpassFilterBackend>(width, height);
    if (name == "optical_gyro")          return std::make_unique<OpticalGyroBackend>(width, height);
    if (name == "ultra_slow_motion")     return std::make_unique<UltraSlowMotionBackend>(width, height);
    if (name == "xyt_visualizer")        return std::make_unique<XYTVisualizerBackend>(width, height);
    if (name == "time_surface")          return std::make_unique<TimeSurfaceBackend>(width, height);
    if (name == "overlay")               return std::make_unique<OverlayBackend>(width, height);
    // algo/analytics
    if (name == "active_marker")         return std::make_unique<ActiveMarkerBackend>(width, height);
    if (name == "event_to_video")        return std::make_unique<EventToVideoBackend>(width, height);
    if (name == "flow_statistics")       return std::make_unique<FlowStatisticsBackend>(width, height);
    if (name == "isi_analyzer")          return std::make_unique<ISIAnalyzerBackend>(width, height);
    if (name == "particle_counter")      return std::make_unique<ParticleCounterBackend>(width, height);
    if (name == "auto_bias")             return std::make_unique<AutoBiasBackend>(width, height);
    if (name == "freq_detector")         return std::make_unique<FreqDetectorBackend>(width, height);
    return nullptr;
}

} // namespace gui
